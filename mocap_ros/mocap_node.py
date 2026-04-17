#!/usr/bin/env python3
"""
mocap_ros – MocapNode
Streams 6-DoF pose data from a Qualisys Motion Capture system into ROS 2 TF
and, optionally, nav_msgs/Odometry for any number of tracked bodies.

Configuration is loaded from a plain YAML file and validated with Pydantic.
The config file path is passed as the ROS 2 parameter `config_file`.
"""

import asyncio
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Annotated, Dict, List, Optional

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

    base_frame: str = ""
    """TF child frame id. Defaults to '<name_lowercase>/base'."""

    publish_odometry: bool = False
    """Also publish a nav_msgs/Odometry message for this body."""

    odometry_topic: str = ""
    """Odometry topic name. Defaults to '<name_lowercase>/odometry/filtered'."""

    @model_validator(mode="after")
    def _set_defaults(self) -> "BodyConfig":
        slug = self.name.lower()
        if not self.base_frame:
            self.base_frame = f"{slug}/base"
        if self.publish_odometry and not self.odometry_topic:
            self.odometry_topic = f"{slug}/odometry/filtered"
        return self


class MocapConfig(BaseModel):
    """Top-level configuration for the mocap_ros node."""

    qualisys_ip: str = "192.168.75.4"
    """IP address of the Qualisys host machine."""

    publishing_freq: Annotated[int, Field(ge=1, le=300)] = 100
    """TF / Odometry publish rate in Hz (1 – 300)."""

    ref_frame: str = "world"
    """Fixed world frame id used as the TF parent."""

    bodies: List[BodyConfig] = Field(default_factory=list)
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


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class MocapNode(Node):
    """ROS 2 node that bridges Qualisys MoCap data to TF / Odometry."""

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
            self.destroy_node()
            return

        self._publish_period: float = 1.0 / self._cfg.publishing_freq

        # ── Publishers ────────────────────────────────────────────────────
        self._tf_broadcaster = TransformBroadcaster(self)

        self._body_states: Dict[str, BodyState] = {}
        for body_cfg in self._cfg.bodies:
            pub = None
            if body_cfg.publish_odometry:
                pub = self.create_publisher(Odometry, body_cfg.odometry_topic, 10)
            self._body_states[body_cfg.name] = BodyState(body_cfg, pub)

        # ── QTM index map (filled after connection) ───────────────────────
        self._body_index: Dict[str, int] = {}

        # ── Timestamp throttling ──────────────────────────────────────────
        self._prec_timestamp: float = 0.0

        self._log_startup()

        # ── Start async MoCap loop ────────────────────────────────────────
        asyncio.ensure_future(self._connect_and_stream())
        asyncio.get_event_loop().run_forever()


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
        """Connect to QTM and register the packet callback."""
        ip = self._cfg.qualisys_ip
        connection = await qtm_rt.connect(ip, version="1.24")

        if connection is None:
            self.get_logger().error(f"Could not connect to Qualisys at {ip}")
            self.destroy_node()
            return

        self.get_logger().info(f"Connected to Qualisys at {ip}")

        xml_string = await connection.get_parameters(parameters=["6d"])
        self._body_index = self._parse_body_index(xml_string)

        for body_cfg in self._cfg.bodies:
            if body_cfg.name not in self._body_index:
                self.get_logger().warn(
                    f"Body '{body_cfg.name}' not found in Qualisys stream – will be ignored."
                )

        await connection.stream_frames(components=["6d"], on_packet=self._on_packet)


    def _on_packet(self, packet):
        """Called by qtm_rt for every incoming data frame."""
        new_ts: float = packet.timestamp * 1e-6  # µs → s

        if (new_ts - self._prec_timestamp) < self._publish_period:
            return

        _info, bodies = packet.get_6d()
        stamp = self.get_clock().now().to_msg()

        for qtm_index, body in enumerate(bodies):
            body_name = self._index_to_name(qtm_index)
            if body_name is None or body_name not in self._body_states:
                continue

            state = self._body_states[body_name]
            position, rotation = body

            # Rotation matrix (column-major from QTM) → quaternion via PyKDL
            rot = PyKDL.Rotation(
                rotation[0][0], rotation[0][3], rotation[0][6],
                rotation[0][1], rotation[0][4], rotation[0][7],
                rotation[0][2], rotation[0][5], rotation[0][8],
            )
            qx, qy, qz, qw = rot.GetQuaternion()

            # ── TF ────────────────────────────────────────────────────
            tf = state.transform_msg
            tf.header.stamp = stamp
            tf.header.frame_id = self._cfg.ref_frame
            tf.child_frame_id = state.config.base_frame
            tf.transform.translation.x = position[0] * 1e-3  # mm → m
            tf.transform.translation.y = position[1] * 1e-3
            tf.transform.translation.z = position[2] * 1e-3
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(tf)

            # ── Odometry (optional) ───────────────────────────────────
            if state.config.publish_odometry and state.odometry_publisher is not None:
                odom = state.odometry_msg
                odom.header.stamp = stamp
                odom.header.frame_id = self._cfg.ref_frame
                odom.child_frame_id = state.config.base_frame
                odom.pose.pose.position.x = position[0] * 1e-3
                odom.pose.pose.position.y = position[1] * 1e-3
                odom.pose.pose.position.z = position[2] * 1e-3
                odom.pose.pose.orientation.x = qx
                odom.pose.pose.orientation.y = qy
                odom.pose.pose.orientation.z = qz
                odom.pose.pose.orientation.w = qw
                # Velocities not measured by MoCap
                odom.twist.twist.linear.x = 0.0
                odom.twist.twist.linear.y = 0.0
                odom.twist.twist.linear.z = 0.0
                odom.twist.twist.angular.x = 0.0
                odom.twist.twist.angular.y = 0.0
                odom.twist.twist.angular.z = 0.0
                state.odometry_publisher.publish(odom)

        self._prec_timestamp = new_ts


    def _parse_body_index(self, xml_string: str) -> Dict[str, int]:
        index: Dict[str, int] = {}
        xml = ET.fromstring(xml_string)
        for i, body in enumerate(xml.findall("*/Body/Name")):
            index[body.text.strip()] = i
        self.get_logger().info(f"Bodies available in Qualisys: {list(index.keys())}")
        return index

    def _index_to_name(self, qtm_index: int) -> Optional[str]:
        for name, idx in self._body_index.items():
            if idx == qtm_index:
                return name
        return None

    def _log_startup(self):
        self.get_logger().info("=== mocap_ros node starting ===")
        self.get_logger().info(f"  qualisys_ip    : {self._cfg.qualisys_ip}")
        self.get_logger().info(f"  ref_frame     : {self._cfg.ref_frame}")
        self.get_logger().info(f"  publishing_freq: {self._cfg.publishing_freq} Hz")
        for b in self._cfg.bodies:
            line = f"  body '{b.name}' → frame '{b.base_frame}'"
            if b.publish_odometry:
                line += f", odometry on '{b.odometry_topic}'"
            self.get_logger().info(line)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MocapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
