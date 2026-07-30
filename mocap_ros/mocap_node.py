#!/usr/bin/env python3
"""
mocap_ros – MocapNode
Streams 6-DoF pose data from a Qualisys Motion Capture system into ROS 2 TF
and, optionally, nav_msgs/Odometry for any number of tracked bodies.

Configuration is loaded from a plain YAML file and validated with Pydantic.
The config file path is passed as the ROS 2 parameter `config_file`.
"""

import asyncio
import math
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Annotated, Optional
import numpy as np
import yaml
import PyKDL
import qtm_rt
from pydantic import BaseModel, Field, field_validator, model_validator

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

# ---------------------------------------------------------------------------
# Pydantic config models
# ---------------------------------------------------------------------------

class BodyConfig(BaseModel):
    """Configuration for a single tracked rigid body."""

    name: str
    """Body name exactly as defined in the Qualisys project."""

    tf_frame_name: str = ""
    """TF child frame id. Defaults to '<name_lowercase>/base'."""

    publish_odometry: bool = False
    """Also publish a nav_msgs/Odometry message for this body."""

    odometry_topic: str = ""
    """Odometry topic name. Defaults to '<name_lowercase>/odometry/mocap'."""

    @model_validator(mode="after")
    def _set_defaults(self) -> "BodyConfig":
        slug = self.name.lower()
        if not self.tf_frame_name:
            self.tf_frame_name = f"{slug}/base"
        if self.publish_odometry and not self.odometry_topic:
            self.odometry_topic = f"{slug}/odometry/mocap"
        return self


class MocapConfig(BaseModel):
    """Top-level configuration for the mocap_ros node."""

    qualisys_ip: str = "192.168.75.4"
    """IP address of the Qualisys host machine."""

    publishing_freq: Annotated[int, Field(ge=1, le=300)] = 100
    """TF / Odometry publish rate in Hz (1 – 300)."""

    ref_frame: str = "world"
    """Fixed world frame id used as the TF parent."""

    pose_timeout: Annotated[float, Field(gt=0.0, le=60.0)] = 0.5
    """
    Maximum age (in seconds) of a body's last received pose before it is
    considered stale. Stale poses are no longer published (TF / Odometry)
    until fresh data arrives again, so a dropped QTM link doesn't silently
    freeze downstream consumers on an old position stamped with a fresh
    timestamp.
    """

    bodies: list[BodyConfig] = Field(default_factory=list)
    """List of rigid bodies to track."""

    @field_validator("bodies")
    @classmethod
    def _at_least_one_body(cls, v: list) -> list:
        if not v:
            raise ValueError("At least one body must be declared under 'bodies'.")
        return v


# ---------------------------------------------------------------------------
# Per-body runtime state (plain dataclass – not part of the Pydantic schema)
# ---------------------------------------------------------------------------

class BodyState:
    """Pre-allocated ROS messages + optional publisher for one body."""

    def __init__(self, config: BodyConfig, odometry_publisher=None):
        self.config = config
        self.transform_msg = TransformStamped()
        self.odometry_msg = Odometry()
        self.odometry_publisher = odometry_publisher

        # Timestamp (time.monotonic()) of the last warning logged for this
        # body being stale, used to throttle repeated log spam.
        self.last_stale_warning: float = 0.0

        # Odometry covariance: since MoCap gives no velocity estimate and we
        # report zero twist, mark twist covariance as "unknown / very large"
        # rather than zero (which downstream filters like robot_localization
        # would otherwise treat as a perfect, exact measurement).
        if odometry_publisher is not None:
            large = 1e6
            for i in range(6):
                self.odometry_msg.twist.covariance[i * 6 + i] = large
            # Pose covariance: MoCap is quite accurate, use a small but
            # non-zero value so it isn't mistaken for "perfectly known".
            small = 1e-4
            for i in range(6):
                self.odometry_msg.pose.covariance[i * 6 + i] = small


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class MocapNode(Node):
    """ROS 2 node that bridges Qualisys MoCap data to TF / Odometry."""

    # Minimum interval between repeated "pose is stale" log messages for the
    # same body, to avoid flooding the logs once publishing_freq is high.
    _STALE_WARNING_PERIOD = 5.0

    def __init__(self):
        super().__init__("mocap_node")

        # Single ROS 2 parameter: path to the YAML config file
        self.declare_parameter("config_file", "")
        config_path = self.get_parameter("config_file").value

        # Load & validate config with Pydantic
        try:
            self._cfg = self._load_config(config_path)
        except Exception as exc:
            self.get_logger().error(f"Invalid configuration: {exc}")
            raise RuntimeError(f"mocap_node: invalid configuration: {exc}") from exc

        self._publish_period: float = 1.0 / self._cfg.publishing_freq

        self.connection = None

        # == Publishers ====================================================
        self._tf_broadcaster = TransformBroadcaster(self)

        self._body_states: dict[str, BodyState] = {}
        for body_cfg in self._cfg.bodies:
            pub = None
            if body_cfg.publish_odometry:
                pub = self.create_publisher(Odometry, body_cfg.odometry_topic, 10)
            self._body_states[body_cfg.name] = BodyState(body_cfg, pub)

        # Shared state, protected by a lock, filled by the asyncio thread.
        # Each entry is (position, rotation, received_at) where received_at
        # is a time.monotonic() timestamp used for staleness detection.
        self._latest_poses: dict[str, tuple] = {}
        self._prev_pose: dict[str, tuple] = {}
        self._poses_lock = threading.Lock()

        # == QTM index maps (filled after self.connection) ======================
        self._body_index: dict[str, int] = {}      # name -> qtm index
        self._index_body: dict[int, str] = {}       # qtm index -> name

        # == Shutdown / lifecycle control ==================================
        self._stop_event = threading.Event()
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        self._log_startup()

        self._qtm_thread = threading.Thread(
            target=self._run_qtm_loop, name="mocap_qtm_thread", daemon=True
        )
        self._qtm_thread.start()
        # Pure ROS timer, drives publication.
        self.create_timer(self._publish_period, self._publish_tick)

    # -----------------------------------------------------------------
    # asyncio thread management
    # -----------------------------------------------------------------

    def _run_qtm_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        task = self._loop.create_task(self._connect_and_stream())
        task.add_done_callback(self._on_qtm_task_done)

        try:
            self._loop.run_forever()
        finally:
            self._loop.stop()
            self._loop.close()

    def _on_qtm_task_done(self, task: "asyncio.Task"):
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            self.get_logger().error(f"QTM streaming task terminated unexpectedly: {exc!r}")

    def destroy_node(self):
        """Ensure the asyncio thread is stopped before tearing down the node."""
        self._stop_event.set()
        if self._loop is not None and self._loop.is_running():
            # self.connection was created and is used exclusively from the
            # asyncio thread; disconnecting it must happen on that same
            # thread/loop rather than directly from this (ROS executor)
            # thread.
            if self.connection is not None:
                self._loop.call_soon_threadsafe(self.connection.disconnect)
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._qtm_thread.is_alive():
            self._qtm_thread.join(timeout=5.0)
        super().destroy_node()

    @staticmethod
    def _load_config(config_path: str) -> MocapConfig:
        """
        Load and validate the YAML config file.

        The YAML file must have a top-level key matching the node name, e.g.:

            mocap_node:
              qualisys_ip: "192.168.75.4"
              bodies:
                - name: "Go2"
                  ...
        """
        path = Path(config_path)
        if not path.is_file():
            raise FileNotFoundError(f"Config file not found: '{config_path}'")

        raw = yaml.safe_load(path.read_text())

        # Support both bare dict and ROS-style  {node_name: {params}}
        if isinstance(raw, dict) and len(raw) == 1 and not {"bodies", "qualisys_ip"} & raw.keys():
            raw = next(iter(raw.values()))  # unwrap single top-level key

        return MocapConfig.model_validate(raw)

    async def _connect_and_stream(self):
        """
        Connect to QTM, register the packet callback, and stream.
        """
        ip = self._cfg.qualisys_ip
        self.connection = await qtm_rt.connect(ip, version="1.24")

        if self.connection is None:
            raise ConnectionError(f"Could not connect to Qualisys at {ip}")

        self.get_logger().info(f"Connected to Qualisys at {ip}")

        xml_string = await self.connection.get_parameters(parameters=["6d"])
        self._body_index, self._index_body = self._parse_body_index(xml_string)

        for body_cfg in self._cfg.bodies:
            if body_cfg.name not in self._body_index:
                self.get_logger().warn(
                    f"Body '{body_cfg.name}' not found in Qualisys stream – will be ignored."
                )
        self.get_logger().info(f"Start streaming")
        await self.connection.stream_frames(components=["6d"], on_packet=self._on_packet)

    def _on_packet(self, packet):
        """Runs in the QTM thread: only updates state, no ROS publishing here."""
        _info, bodies = packet.get_6d()
        now = time.monotonic()
        with self._poses_lock :
            for qtm_index, body in enumerate(bodies):
                name = self._index_body[qtm_index]
                if name in self._body_states:
                    if name in self._latest_poses:
                        self._prev_pose[name] = self._latest_poses[name]
                    self._latest_poses[name] = (body[0], body[1], now, packet.timestamp * 1e-6)


    def _publish_tick(self):
        """Runs in the ROS executor, at publishing_freq, independent of the QTM network."""
        stamp = self.get_clock().now().to_msg()
        now = time.monotonic()
        with self._poses_lock :
            poses = dict(self._latest_poses)  # quick copy, release the lock fast
            prev_poses = dict(self._prev_pose)

        for name, (position, rotation, received_at, _) in poses.items():
            state = self._body_states[name]

            age = now - received_at
            if age > self._cfg.pose_timeout:
                if now - state.last_stale_warning >= self._STALE_WARNING_PERIOD:
                    self.get_logger().warn(
                        f"Pose for body '{name}' is stale ({age:.2f}s > "
                        f"{self._cfg.pose_timeout:.2f}s timeout) – not publishing."
                    )
                    state.last_stale_warning = now
                continue

            if any(math.isnan(v) for v in position):
                continue
            if any(math.isnan(v) for v in rotation[0]):
                continue
            # Convert QTM Column-Major matrix to PyKDL Row-Major.
            rot = PyKDL.Rotation(
                rotation[0][0], rotation[0][3], rotation[0][6],
                rotation[0][1], rotation[0][4], rotation[0][7],
                rotation[0][2], rotation[0][5], rotation[0][8],
            )
            qx, qy, qz, qw = rot.GetQuaternion()

            tf = state.transform_msg
            tf.header.stamp = stamp
            tf.header.frame_id = self._cfg.ref_frame
            tf.child_frame_id = state.config.tf_frame_name
            tf.transform.translation.x = position[0] * 1e-3
            tf.transform.translation.y = position[1] * 1e-3
            tf.transform.translation.z = position[2] * 1e-3
            tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self._tf_broadcaster.sendTransform(tf)

            if state.config.publish_odometry and state.odometry_publisher is not None:
                linear_vel, angular_vel = self.esti_twist(body_name=name,actual_pose=poses,prev_pose=prev_poses)
                odom = state.odometry_msg
                odom.header.stamp = stamp
                odom.header.frame_id = self._cfg.ref_frame
                odom.child_frame_id = state.config.tf_frame_name
                odom.pose.pose.position.x = position[0] * 1e-3
                odom.pose.pose.position.y = position[1] * 1e-3
                odom.pose.pose.position.z = position[2] * 1e-3
                odom.pose.pose.orientation.x = qx
                odom.pose.pose.orientation.y = qy
                odom.pose.pose.orientation.z = qz
                odom.pose.pose.orientation.w = qw
                # Velocities not measured by MoCap; twist covariance is set
                # to a large value at startup (see BodyState) so consumers
                # know not to trust it.
                odom.twist.twist.linear.x = linear_vel[0]
                odom.twist.twist.linear.y = linear_vel[1]
                odom.twist.twist.linear.z = linear_vel[2]
                odom.twist.twist.angular.x = angular_vel[0]
                odom.twist.twist.angular.y = angular_vel[1]
                odom.twist.twist.angular.z = angular_vel[2]
                state.odometry_publisher.publish(odom)

    def _parse_body_index(self, xml_string: str) -> tuple[dict[str, int], dict[int, str]]:
        name_to_index: dict[str, int] = {}
        index_to_name: dict[int, str] = {}
        xml = ET.fromstring(xml_string)
        for i, body in enumerate(xml.findall("*/Body/Name")):
            name = body.text.strip()
            name_to_index[name] = i
            index_to_name[i] = name
        self.get_logger().info(f"Bodies available in Qualisys: {list(name_to_index.keys())}")
        return name_to_index, index_to_name

    def _log_startup(self):
        self.get_logger().info("=== mocap_ros node starting ===")
        self.get_logger().info(f"  qualisys_ip    : {self._cfg.qualisys_ip}")
        self.get_logger().info(f"  ref_frame      : {self._cfg.ref_frame}")
        self.get_logger().info(f"  publishing_freq: {self._cfg.publishing_freq} Hz")
        self.get_logger().info(f"  pose_timeout   : {self._cfg.pose_timeout} s")
        for b in self._cfg.bodies:
            defaulted_frame = "" if b.tf_frame_name == f"{b.name.lower()}/base" else " (explicit)"
            line = f"  body '{b.name}' → frame '{b.tf_frame_name}'{defaulted_frame}"
            if b.publish_odometry:
                line += f", odometry on '{b.odometry_topic}'"
            self.get_logger().info(line)

    def esti_twist(
        self,
        body_name: str,
        actual_pose,
        prev_pose,
    ):
        """
        Estimate the spatial twist from two consecutive mocap poses.

        Parameters
        ----------
        body_name : str
            Name of the tracked rigid body.

        Returns
        -------
        linear_velocity : np.ndarray(3,) [m/s]
        angular_velocity : np.ndarray(3,) [rad/s]
        """
        if body_name not in prev_pose:
            return np.zeros(3), np.zeros(3)

        p0 = np.asarray(prev_pose[body_name][0], dtype=float) * 1e-3
        R = prev_pose[body_name][1]
        R0 = np.array(R[0], dtype=float).reshape((3, 3), order="F")
        t0 = prev_pose[body_name][3] # use mocap timestamp for mor precise speed estim

        p1 = np.asarray(actual_pose[body_name][0], dtype=float) * 1e-3
        R = actual_pose[body_name][1]
        R1 = np.array(R[0], dtype=float).reshape((3, 3), order="F")
        t1 = actual_pose[body_name][3]



        dt = t1 - t0
        if dt <= 1e-6:
            return np.zeros(3), np.zeros(3)
        if dt > self._cfg.pose_timeout:
            return np.zeros(3), np.zeros(3)

        # Linear velocity, expressed in the body (child) frame, per the
        # nav_msgs/Odometry convention — NOT in ref_frame. Using R0 here
        # (rather than R1) keeps it consistent with the angular velocity
        # below, which is also derived relative to R0.
        v_ref = (p1 - p0) / dt
        v = R0.T @ v_ref

        # Relative rotation
        R_rel = R0.T @ R1

        rot = PyKDL.Rotation(
            R_rel[0, 0], R_rel[0, 1], R_rel[0, 2],
            R_rel[1, 0], R_rel[1, 1], R_rel[1, 2],
            R_rel[2, 0], R_rel[2, 1], R_rel[2, 2],
        )

        angle, axis = rot.GetRotAngle()

        omega = np.array([axis[0], axis[1], axis[2]]) * angle / dt

        return v, omega


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MocapNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # Raised by MocapNode.__init__ on invalid configuration; the error
        # was already logged there.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()