# mocap_ros

ROS 2 package bridging a **Qualisys** Motion Capture system to TF and Odometry topics.  
Multiple rigid bodies can be tracked simultaneously; everything is driven by a single YAML config file.

---
> [!IMPORTANT]
> Please setup your QMT project to use **RealTime Streaming 1.24**

## Dependencies

| Dependency | Install |
|---|---|
| `qtm_rt` (Qualisys SDK) | `pip install qtm-rt` |
| `PyKDL` | `micromamba install python-orocos-kdl` |
| ROS 2 packages | `rclpy`, `tf2_ros`, `geometry_msgs`, `nav_msgs` |
 micromamba install pydantic
---

## Build

```bash
# From your ROS 2 workspace root
colcon build --packages-select mocap_ros
source install/setup.bash
```

---

## Configuration

Edit `config/bodies.yaml` to declare the bodies you want to track:

```yaml
mocap_node:
  ros__parameters:
    qualisys_ip: "192.168.75.4"
    publishing_freq: 110        # Hz, max 300
    ref_frame: "odom"

    bodies:
      - name: "Go2"             # must match the name in the Qualisys project
        base_frame: "go2/base"  # TF child frame that will be published
        publish_odometry: true  # also publish nav_msgs/Odometry
        odometry_topic: "go2/odometry/filtered"

      - name: "Cube"
        base_frame: "cube/base"
        publish_odometry: false

      - name: "Marker1"
        base_frame: "marker1/base"
        publish_odometry: false
```

### Parameter reference

| Parameter | Type | Default | Description |
|---|---|---|---|
| `qualisys_ip` | string | `192.168.75.4` | IP of the Qualisys machine |
| `publishing_freq` | int | `110` | TF / Odometry publish rate in Hz (1–300) |
| `odom_frame` | string | `odom` | Fixed world frame id |
| `bodies[i].name` | string | — | Body name **as set in Qualisys** |
| `bodies[i].base_frame` | string | `<name>/base` | TF child frame to publish |
| `bodies[i].publish_odometry` | bool | `false` | Publish `nav_msgs/Odometry` |
| `bodies[i].odometry_topic` | string | `<name>/odometry/filtered` | Odometry topic name |

---

## Run

```bash
# With the default config
ros2 launch mocap_ros mocap.launch.py

# Or with a custom config file
ros2 run mocap_ros mocap_node --ros-args --params-file /path/to/my_config.yaml
```

---

## Published topics & transforms

For each tracked body the node publishes:

| Output | Type | Condition |
|---|---|---|
| `TF: odom → <base_frame>` | `geometry_msgs/TransformStamped` | always |
| `<odometry_topic>` | `nav_msgs/Odometry` | `publish_odometry: true` |

---

## Architecture

```
Qualisys (UDP/RealTime 1.24)
        │
        ▼
  qtm_rt (async)
        │  on_packet callback
        ▼
  MocapNode
   ├── for each body in config
   │    ├── compute quaternion from rotation matrix (PyKDL)
   │    ├── broadcast TF  odom → base_frame
   │    └── (optional) publish Odometry
   └── YAML config loaded at startup
```
