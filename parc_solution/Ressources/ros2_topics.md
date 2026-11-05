# ROS 2 Topics and Message Structures

## Contents

1. [Topic overview](#topic-overview)
2. [Topic details](#topic-details)
3. [Message type reference](#message-type-reference)

## Topic overview

| Topic | Message type | Publishers | Subscribers |
| --- | --- | ---: | ---: |
| [`/accessory_view/command`](#topic-accessory-view-command) | [`caytu_msgs/msg/AccessoryCommand`](#type-caytu-msgs-msg-accessorycommand) | 0 | 1 |
| [`/accessory_view/state`](#topic-accessory-view-state) | [`std_msgs/msg/String`](#type-std-msgs-msg-string) | 1 | 0 |
| [`/auto_vel`](#topic-auto-vel) | [`geometry_msgs/msg/Twist`](#type-geometry-msgs-msg-twist) | 1 | 0 |
| [`/battery_level`](#topic-battery-level) | [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32) | 1 | 0 |
| [`/botwheel_explorer/cmd_vel`](#topic-botwheel-explorer-cmd-vel) | [`geometry_msgs/msg/TwistStamped`](#type-geometry-msgs-msg-twiststamped) | 1 | 1 |
| [`/botwheel_explorer/odom`](#topic-botwheel-explorer-odom) | [`nav_msgs/msg/Odometry`](#type-nav-msgs-msg-odometry) | 1 | 1 |
| [`/botwheel_explorer/transition_event`](#topic-botwheel-explorer-transition-event) | [`lifecycle_msgs/msg/TransitionEvent`](#type-lifecycle-msgs-msg-transitionevent) | 1 | 0 |
| [`/camera1/camera1/color/camera_info`](#topic-camera1-camera1-color-camera-info) | [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo) | 1 | 0 |
| [`/camera1/camera1/color/image_raw`](#topic-camera1-camera1-color-image-raw) | [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image) | 1 | 0 |
| [`/camera1/camera1/color/metadata`](#topic-camera1-camera1-color-metadata) | [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata) | 1 | 0 |
| [`/camera1/camera1/depth/camera_info`](#topic-camera1-camera1-depth-camera-info) | [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo) | 1 | 1 |
| [`/camera1/camera1/depth/image_rect_raw`](#topic-camera1-camera1-depth-image-rect-raw) | [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image) | 1 | 1 |
| [`/camera1/camera1/depth/metadata`](#topic-camera1-camera1-depth-metadata) | [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata) | 1 | 0 |
| [`/camera1/camera1/extrinsics/depth_to_color`](#topic-camera1-camera1-extrinsics-depth-to-color) | [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics) | 1 | 0 |
| [`/camera1/scan`](#topic-camera1-scan) | [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan) | 1 | 1 |
| [`/camera2/camera2/color/camera_info`](#topic-camera2-camera2-color-camera-info) | [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo) | 1 | 0 |
| [`/camera2/camera2/color/image_raw`](#topic-camera2-camera2-color-image-raw) | [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image) | 1 | 1 |
| [`/camera2/camera2/color/metadata`](#topic-camera2-camera2-color-metadata) | [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata) | 1 | 0 |
| [`/camera2/camera2/depth/camera_info`](#topic-camera2-camera2-depth-camera-info) | [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo) | 1 | 1 |
| [`/camera2/camera2/depth/image_rect_raw`](#topic-camera2-camera2-depth-image-rect-raw) | [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image) | 1 | 2 |
| [`/camera2/camera2/depth/metadata`](#topic-camera2-camera2-depth-metadata) | [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata) | 1 | 0 |
| [`/camera2/camera2/extrinsics/depth_to_color`](#topic-camera2-camera2-extrinsics-depth-to-color) | [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics) | 1 | 0 |
| [`/camera2/scan`](#topic-camera2-scan) | [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan) | 1 | 0 |
| [`/client_count`](#topic-client-count) | [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32) | 1 | 0 |
| [`/closest_gap`](#topic-closest-gap) | [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2) | 1 | 0 |
| [`/cmd_vel`](#topic-cmd-vel) | [`geometry_msgs/msg/Twist`](#type-geometry-msgs-msg-twist) | 0 | 1 |
| [`/collision_obstacles`](#topic-collision-obstacles) | [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2) | 1 | 0 |
| [`/connected_clients`](#topic-connected-clients) | [`rosbridge_msgs/msg/ConnectedClients`](#type-rosbridge-msgs-msg-connectedclients) | 1 | 0 |
| [`/controller_manager/activity`](#topic-controller-manager-activity) | [`controller_manager_msgs/msg/ControllerManagerActivity`](#type-controller-manager-msgs-msg-controllermanageractivity) | 1 | 0 |
| [`/controller_manager/introspection_data/full`](#topic-controller-manager-introspection-data-full) | [`pal_statistics_msgs/msg/Statistics`](#type-pal-statistics-msgs-msg-statistics) | 1 | 0 |
| [`/controller_manager/introspection_data/names`](#topic-controller-manager-introspection-data-names) | [`pal_statistics_msgs/msg/StatisticsNames`](#type-pal-statistics-msgs-msg-statisticsnames) | 1 | 0 |
| [`/controller_manager/introspection_data/values`](#topic-controller-manager-introspection-data-values) | [`pal_statistics_msgs/msg/StatisticsValues`](#type-pal-statistics-msgs-msg-statisticsvalues) | 1 | 0 |
| [`/controller_manager/statistics/full`](#topic-controller-manager-statistics-full) | [`pal_statistics_msgs/msg/Statistics`](#type-pal-statistics-msgs-msg-statistics) | 1 | 0 |
| [`/controller_manager/statistics/names`](#topic-controller-manager-statistics-names) | [`pal_statistics_msgs/msg/StatisticsNames`](#type-pal-statistics-msgs-msg-statisticsnames) | 1 | 0 |
| [`/controller_manager/statistics/values`](#topic-controller-manager-statistics-values) | [`pal_statistics_msgs/msg/StatisticsValues`](#type-pal-statistics-msgs-msg-statisticsvalues) | 1 | 0 |
| [`/desired_traj`](#topic-desired-traj) | [`geometry_msgs/msg/PoseArray`](#type-geometry-msgs-msg-posearray) | 1 | 0 |
| [`/diagnostics`](#topic-diagnostics) | [`diagnostic_msgs/msg/DiagnosticArray`](#type-diagnostic-msgs-msg-diagnosticarray) | 2 | 0 |
| [`/dynamic_joint_states`](#topic-dynamic-joint-states) | [`control_msgs/msg/DynamicJointState`](#type-control-msgs-msg-dynamicjointstate) | 1 | 0 |
| [`/footprint`](#topic-footprint) | [`visualization_msgs/msg/Marker`](#type-visualization-msgs-msg-marker) | 1 | 0 |
| [`/gaps`](#topic-gaps) | [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2) | 1 | 0 |
| [`/goal`](#topic-goal) | [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped) | 0 | 1 |
| [`/joint_state_broadcaster/transition_event`](#topic-joint-state-broadcaster-transition-event) | [`lifecycle_msgs/msg/TransitionEvent`](#type-lifecycle-msgs-msg-transitionevent) | 1 | 0 |
| [`/joint_states`](#topic-joint-states) | [`sensor_msgs/msg/JointState`](#type-sensor-msgs-msg-jointstate) | 1 | 1 |
| [`/nav_goal`](#topic-nav-goal) | [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped) | 1 | 0 |
| [`/network/command`](#topic-network-command) | [`caytu_msgs/msg/NetworkCommand`](#type-caytu-msgs-msg-networkcommand) | 0 | 1 |
| [`/network/state`](#topic-network-state) | [`caytu_msgs/msg/NetworkInfo`](#type-caytu-msgs-msg-networkinfo) | 1 | 0 |
| [`/parameter_events`](#topic-parameter-events) | [`rcl_interfaces/msg/ParameterEvent`](#type-rcl-interfaces-msg-parameterevent) | 31 | 19 |
| [`/person_following_robot/color_data`](#topic-person-following-robot-color-data) | [`person_following_robot/msg/TrackedObject`](#type-person-following-robot-msg-trackedobject) | 0 | 1 |
| [`/person_following_robot/follow_enable`](#topic-person-following-robot-follow-enable) | [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool) | 1 | 1 |
| [`/person_following_robot/gui_enable`](#topic-person-following-robot-gui-enable) | [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool) | 1 | 1 |
| [`/person_following_robot/gui_state`](#topic-person-following-robot-gui-state) | [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool) | 1 | 0 |
| [`/robot_description`](#topic-robot-description) | [`std_msgs/msg/String`](#type-std-msgs-msg-string) | 1 | 1 |
| [`/ros_to_iot`](#topic-ros-to-iot) | [`std_msgs/msg/String`](#type-std-msgs-msg-string) | 1 | 0 |
| [`/rosout`](#topic-rosout) | [`rcl_interfaces/msg/Log`](#type-rcl-interfaces-msg-log) | 32 | 0 |
| [`/safe_vel`](#topic-safe-vel) | [`geometry_msgs/msg/Twist`](#type-geometry-msgs-msg-twist) | 1 | 0 |
| [`/scan`](#topic-scan) | [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan) | 1 | 1 |
| [`/scan_filtered`](#topic-scan-filtered) | [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan) | 1 | 1 |
| [`/scan_merged`](#topic-scan-merged) | [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan) | 1 | 1 |
| [`/stamped_vel`](#topic-stamped-vel) | [`geometry_msgs/msg/TwistStamped`](#type-geometry-msgs-msg-twiststamped) | 1 | 1 |
| [`/tf`](#topic-tf) | [`tf2_msgs/msg/TFMessage`](#type-tf2-msgs-msg-tfmessage) | 2 | 2 |
| [`/tf_static`](#topic-tf-static) | [`tf2_msgs/msg/TFMessage`](#type-tf2-msgs-msg-tfmessage) | 8 | 2 |
| [`/tracked_person/position`](#topic-tracked-person-position) | [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped) | 1 | 1 |
| [`/virt_gaps`](#topic-virt-gaps) | [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2) | 1 | 0 |
| [`/volume/command`](#topic-volume-command) | [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32) | 0 | 1 |
| [`/volume/state`](#topic-volume-state) | [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32) | 1 | 0 |
| [`/webrtc/command`](#topic-webrtc-command) | [`caytu_msgs/msg/ActionCommand`](#type-caytu-msgs-msg-actioncommand) | 0 | 1 |
| [`/webrtc/state`](#topic-webrtc-state) | [`std_msgs/msg/String`](#type-std-msgs-msg-string) | 1 | 0 |

## Topic details

### `/accessory_view/command`

- **Message type:** [`caytu_msgs/msg/AccessoryCommand`](#type-caytu-msgs-msg-accessorycommand)
- **Publishers:** 0
- **Subscribers:** 1

### `/accessory_view/state`

- **Message type:** [`std_msgs/msg/String`](#type-std-msgs-msg-string)
- **Publishers:** 1
- **Subscribers:** 0

### `/auto_vel`

- **Message type:** [`geometry_msgs/msg/Twist`](#type-geometry-msgs-msg-twist)
- **Publishers:** 1
- **Subscribers:** 0

### `/battery_level`

- **Message type:** [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32)
- **Publishers:** 1
- **Subscribers:** 0

### `/botwheel_explorer/cmd_vel`

- **Message type:** [`geometry_msgs/msg/TwistStamped`](#type-geometry-msgs-msg-twiststamped)
- **Publishers:** 1
- **Subscribers:** 1

### `/botwheel_explorer/odom`

- **Message type:** [`nav_msgs/msg/Odometry`](#type-nav-msgs-msg-odometry)
- **Publishers:** 1
- **Subscribers:** 1

### `/botwheel_explorer/transition_event`

- **Message type:** [`lifecycle_msgs/msg/TransitionEvent`](#type-lifecycle-msgs-msg-transitionevent)
- **Publishers:** 1
- **Subscribers:** 0

### `/camera1/camera1/color/camera_info`

- **Message type:** [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo)
- **Publishers:** 1
- **Subscribers:** 0

### `/camera1/camera1/color/image_raw`

- **Message type:** [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image)
- **Publishers:** 1
- **Subscribers:** 0

### `/camera1/camera1/color/metadata`

- **Message type:** [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata)
- **Publishers:** 1
- **Subscribers:** 0

### `/camera1/camera1/depth/camera_info`

- **Message type:** [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo)
- **Publishers:** 1
- **Subscribers:** 1

### `/camera1/camera1/depth/image_rect_raw`

- **Message type:** [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image)
- **Publishers:** 1
- **Subscribers:** 1

### `/camera1/camera1/depth/metadata`

- **Message type:** [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata)
- **Publishers:** 1
- **Subscribers:** 0

### `/camera1/camera1/extrinsics/depth_to_color`

- **Message type:** [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics)
- **Publishers:** 1
- **Subscribers:** 0

### `/camera1/scan`

- **Message type:** [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan)
- **Publishers:** 1
- **Subscribers:** 1

### `/camera2/camera2/color/camera_info`

- **Message type:** [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo)
- **Publishers:** 1
- **Subscribers:** 0

### `/camera2/camera2/color/image_raw`

- **Message type:** [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image)
- **Publishers:** 1
- **Subscribers:** 1

### `/camera2/camera2/color/metadata`

- **Message type:** [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata)
- **Publishers:** 1
- **Subscribers:** 0

### `/camera2/camera2/depth/camera_info`

- **Message type:** [`sensor_msgs/msg/CameraInfo`](#type-sensor-msgs-msg-camerainfo)
- **Publishers:** 1
- **Subscribers:** 1

### `/camera2/camera2/depth/image_rect_raw`

- **Message type:** [`sensor_msgs/msg/Image`](#type-sensor-msgs-msg-image)
- **Publishers:** 1
- **Subscribers:** 2

### `/camera2/camera2/depth/metadata`

- **Message type:** [`realsense2_camera_msgs/msg/Metadata`](#type-realsense2-camera-msgs-msg-metadata)
- **Publishers:** 1
- **Subscribers:** 0

### `/camera2/camera2/extrinsics/depth_to_color`

- **Message type:** [`realsense2_camera_msgs/msg/Extrinsics`](#type-realsense2-camera-msgs-msg-extrinsics)
- **Publishers:** 1
- **Subscribers:** 0

### `/camera2/scan`

- **Message type:** [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan)
- **Publishers:** 1
- **Subscribers:** 0

### `/client_count`

- **Message type:** [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32)
- **Publishers:** 1
- **Subscribers:** 0

### `/closest_gap`

- **Message type:** [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2)
- **Publishers:** 1
- **Subscribers:** 0

### `/cmd_vel`

- **Message type:** [`geometry_msgs/msg/Twist`](#type-geometry-msgs-msg-twist)
- **Publishers:** 0
- **Subscribers:** 1

### `/collision_obstacles`

- **Message type:** [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2)
- **Publishers:** 1
- **Subscribers:** 0

### `/connected_clients`

- **Message type:** [`rosbridge_msgs/msg/ConnectedClients`](#type-rosbridge-msgs-msg-connectedclients)
- **Publishers:** 1
- **Subscribers:** 0

### `/controller_manager/activity`

- **Message type:** [`controller_manager_msgs/msg/ControllerManagerActivity`](#type-controller-manager-msgs-msg-controllermanageractivity)
- **Publishers:** 1
- **Subscribers:** 0

### `/controller_manager/introspection_data/full`

- **Message type:** [`pal_statistics_msgs/msg/Statistics`](#type-pal-statistics-msgs-msg-statistics)
- **Publishers:** 1
- **Subscribers:** 0

### `/controller_manager/introspection_data/names`

- **Message type:** [`pal_statistics_msgs/msg/StatisticsNames`](#type-pal-statistics-msgs-msg-statisticsnames)
- **Publishers:** 1
- **Subscribers:** 0

### `/controller_manager/introspection_data/values`

- **Message type:** [`pal_statistics_msgs/msg/StatisticsValues`](#type-pal-statistics-msgs-msg-statisticsvalues)
- **Publishers:** 1
- **Subscribers:** 0

### `/controller_manager/statistics/full`

- **Message type:** [`pal_statistics_msgs/msg/Statistics`](#type-pal-statistics-msgs-msg-statistics)
- **Publishers:** 1
- **Subscribers:** 0

### `/controller_manager/statistics/names`

- **Message type:** [`pal_statistics_msgs/msg/StatisticsNames`](#type-pal-statistics-msgs-msg-statisticsnames)
- **Publishers:** 1
- **Subscribers:** 0

### `/controller_manager/statistics/values`

- **Message type:** [`pal_statistics_msgs/msg/StatisticsValues`](#type-pal-statistics-msgs-msg-statisticsvalues)
- **Publishers:** 1
- **Subscribers:** 0

### `/desired_traj`

- **Message type:** [`geometry_msgs/msg/PoseArray`](#type-geometry-msgs-msg-posearray)
- **Publishers:** 1
- **Subscribers:** 0

### `/diagnostics`

- **Message type:** [`diagnostic_msgs/msg/DiagnosticArray`](#type-diagnostic-msgs-msg-diagnosticarray)
- **Publishers:** 2
- **Subscribers:** 0

### `/dynamic_joint_states`

- **Message type:** [`control_msgs/msg/DynamicJointState`](#type-control-msgs-msg-dynamicjointstate)
- **Publishers:** 1
- **Subscribers:** 0

### `/footprint`

- **Message type:** [`visualization_msgs/msg/Marker`](#type-visualization-msgs-msg-marker)
- **Publishers:** 1
- **Subscribers:** 0

### `/gaps`

- **Message type:** [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2)
- **Publishers:** 1
- **Subscribers:** 0

### `/goal`

- **Message type:** [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped)
- **Publishers:** 0
- **Subscribers:** 1

### `/joint_state_broadcaster/transition_event`

- **Message type:** [`lifecycle_msgs/msg/TransitionEvent`](#type-lifecycle-msgs-msg-transitionevent)
- **Publishers:** 1
- **Subscribers:** 0

### `/joint_states`

- **Message type:** [`sensor_msgs/msg/JointState`](#type-sensor-msgs-msg-jointstate)
- **Publishers:** 1
- **Subscribers:** 1

### `/nav_goal`

- **Message type:** [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped)
- **Publishers:** 1
- **Subscribers:** 0

### `/network/command`

- **Message type:** [`caytu_msgs/msg/NetworkCommand`](#type-caytu-msgs-msg-networkcommand)
- **Publishers:** 0
- **Subscribers:** 1

### `/network/state`

- **Message type:** [`caytu_msgs/msg/NetworkInfo`](#type-caytu-msgs-msg-networkinfo)
- **Publishers:** 1
- **Subscribers:** 0

### `/parameter_events`

- **Message type:** [`rcl_interfaces/msg/ParameterEvent`](#type-rcl-interfaces-msg-parameterevent)
- **Publishers:** 31
- **Subscribers:** 19

### `/person_following_robot/color_data`

- **Message type:** [`person_following_robot/msg/TrackedObject`](#type-person-following-robot-msg-trackedobject)
- **Publishers:** 0
- **Subscribers:** 1

### `/person_following_robot/follow_enable`

- **Message type:** [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool)
- **Publishers:** 1
- **Subscribers:** 1

### `/person_following_robot/gui_enable`

- **Message type:** [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool)
- **Publishers:** 1
- **Subscribers:** 1

### `/person_following_robot/gui_state`

- **Message type:** [`std_msgs/msg/Bool`](#type-std-msgs-msg-bool)
- **Publishers:** 1
- **Subscribers:** 0

### `/robot_description`

- **Message type:** [`std_msgs/msg/String`](#type-std-msgs-msg-string)
- **Publishers:** 1
- **Subscribers:** 1

### `/ros_to_iot`

- **Message type:** [`std_msgs/msg/String`](#type-std-msgs-msg-string)
- **Publishers:** 1
- **Subscribers:** 0

### `/rosout`

- **Message type:** [`rcl_interfaces/msg/Log`](#type-rcl-interfaces-msg-log)
- **Publishers:** 32
- **Subscribers:** 0

### `/safe_vel`

- **Message type:** [`geometry_msgs/msg/Twist`](#type-geometry-msgs-msg-twist)
- **Publishers:** 1
- **Subscribers:** 0

### `/scan`

- **Message type:** [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan)
- **Publishers:** 1
- **Subscribers:** 1

### `/scan_filtered`

- **Message type:** [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan)
- **Publishers:** 1
- **Subscribers:** 1

### `/scan_merged`

- **Message type:** [`sensor_msgs/msg/LaserScan`](#type-sensor-msgs-msg-laserscan)
- **Publishers:** 1
- **Subscribers:** 1

### `/stamped_vel`

- **Message type:** [`geometry_msgs/msg/TwistStamped`](#type-geometry-msgs-msg-twiststamped)
- **Publishers:** 1
- **Subscribers:** 1

### `/tf`

- **Message type:** [`tf2_msgs/msg/TFMessage`](#type-tf2-msgs-msg-tfmessage)
- **Publishers:** 2
- **Subscribers:** 2

### `/tf_static`

- **Message type:** [`tf2_msgs/msg/TFMessage`](#type-tf2-msgs-msg-tfmessage)
- **Publishers:** 8
- **Subscribers:** 2

### `/tracked_person/position`

- **Message type:** [`geometry_msgs/msg/PoseStamped`](#type-geometry-msgs-msg-posestamped)
- **Publishers:** 1
- **Subscribers:** 1

### `/virt_gaps`

- **Message type:** [`sensor_msgs/msg/PointCloud2`](#type-sensor-msgs-msg-pointcloud2)
- **Publishers:** 1
- **Subscribers:** 0

### `/volume/command`

- **Message type:** [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32)
- **Publishers:** 0
- **Subscribers:** 1

### `/volume/state`

- **Message type:** [`std_msgs/msg/Int32`](#type-std-msgs-msg-int32)
- **Publishers:** 1
- **Subscribers:** 0

### `/webrtc/command`

- **Message type:** [`caytu_msgs/msg/ActionCommand`](#type-caytu-msgs-msg-actioncommand)
- **Publishers:** 0
- **Subscribers:** 1

### `/webrtc/state`

- **Message type:** [`std_msgs/msg/String`](#type-std-msgs-msg-string)
- **Publishers:** 1
- **Subscribers:** 0

## Message type reference

### `caytu_msgs/msg/AccessoryCommand`

<a id="type-caytu-msgs-msg-accessorycommand"></a>

**Used by topics:** [`/accessory_view/command`](#topic-accessory-view-command)

```text
string url
string action
```

### `std_msgs/msg/String`

<a id="type-std-msgs-msg-string"></a>

**Used by topics:** [`/accessory_view/state`](#topic-accessory-view-state), [`/robot_description`](#topic-robot-description), [`/ros_to_iot`](#topic-ros-to-iot), [`/webrtc/state`](#topic-webrtc-state)

```text
string data
```

### `geometry_msgs/msg/Twist`

<a id="type-geometry-msgs-msg-twist"></a>

**Used by topics:** [`/auto_vel`](#topic-auto-vel), [`/cmd_vel`](#topic-cmd-vel), [`/safe_vel`](#topic-safe-vel)

```text
Vector3  linear
    float64 x
    float64 y
    float64 z
Vector3  angular
    float64 x
    float64 y
    float64 z
```

### `std_msgs/msg/Int32`

<a id="type-std-msgs-msg-int32"></a>

**Used by topics:** [`/battery_level`](#topic-battery-level), [`/client_count`](#topic-client-count), [`/volume/command`](#topic-volume-command), [`/volume/state`](#topic-volume-state)

```text
int32 data
```

### `geometry_msgs/msg/TwistStamped`

<a id="type-geometry-msgs-msg-twiststamped"></a>

**Used by topics:** [`/botwheel_explorer/cmd_vel`](#topic-botwheel-explorer-cmd-vel), [`/stamped_vel`](#topic-stamped-vel)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id
Twist twist
    Vector3  linear
        float64 x
        float64 y
        float64 z
    Vector3  angular
        float64 x
        float64 y
        float64 z
```

### `nav_msgs/msg/Odometry`

<a id="type-nav-msgs-msg-odometry"></a>

**Used by topics:** [`/botwheel_explorer/odom`](#topic-botwheel-explorer-odom)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

string child_frame_id

geometry_msgs/PoseWithCovariance pose
    Pose pose
        Point position
            float64 x
            float64 y
            float64 z
        Quaternion orientation
            float64 x 0
            float64 y 0
            float64 z 0
            float64 w 1
    float64[36] covariance

geometry_msgs/TwistWithCovariance twist
    Twist twist
        Vector3  linear
            float64 x
            float64 y
            float64 z
        Vector3  angular
            float64 x
            float64 y
            float64 z
    float64[36] covariance
```

### `lifecycle_msgs/msg/TransitionEvent`

<a id="type-lifecycle-msgs-msg-transitionevent"></a>

**Used by topics:** [`/botwheel_explorer/transition_event`](#topic-botwheel-explorer-transition-event), [`/joint_state_broadcaster/transition_event`](#topic-joint-state-broadcaster-transition-event)

```text
uint64 timestamp

Transition transition
    uint8 TRANSITION_CREATE = 0
    uint8 TRANSITION_CONFIGURE = 1
    uint8 TRANSITION_CLEANUP = 2
    uint8 TRANSITION_ACTIVATE = 3
    uint8 TRANSITION_DEACTIVATE = 4
    uint8 TRANSITION_UNCONFIGURED_SHUTDOWN  = 5
    uint8 TRANSITION_INACTIVE_SHUTDOWN = 6
    uint8 TRANSITION_ACTIVE_SHUTDOWN = 7
    uint8 TRANSITION_DESTROY = 8
    uint8 TRANSITION_ON_CONFIGURE_SUCCESS = 10
    uint8 TRANSITION_ON_CONFIGURE_FAILURE = 11
    uint8 TRANSITION_ON_CONFIGURE_ERROR = 12
    uint8 TRANSITION_ON_CLEANUP_SUCCESS = 20
    uint8 TRANSITION_ON_CLEANUP_FAILURE = 21
    uint8 TRANSITION_ON_CLEANUP_ERROR = 22
    uint8 TRANSITION_ON_ACTIVATE_SUCCESS = 30
    uint8 TRANSITION_ON_ACTIVATE_FAILURE = 31
    uint8 TRANSITION_ON_ACTIVATE_ERROR = 32
    uint8 TRANSITION_ON_DEACTIVATE_SUCCESS = 40
    uint8 TRANSITION_ON_DEACTIVATE_FAILURE = 41
    uint8 TRANSITION_ON_DEACTIVATE_ERROR = 42
    uint8 TRANSITION_ON_SHUTDOWN_SUCCESS = 50
    uint8 TRANSITION_ON_SHUTDOWN_FAILURE = 51
    uint8 TRANSITION_ON_SHUTDOWN_ERROR = 52
    uint8 TRANSITION_ON_ERROR_SUCCESS = 60
    uint8 TRANSITION_ON_ERROR_FAILURE = 61
    uint8 TRANSITION_ON_ERROR_ERROR = 62
    uint8 TRANSITION_CALLBACK_SUCCESS = 97
    uint8 TRANSITION_CALLBACK_FAILURE = 98
    uint8 TRANSITION_CALLBACK_ERROR = 99
    uint8 id
    string label

State start_state
    uint8 PRIMARY_STATE_UNKNOWN = 0
    uint8 PRIMARY_STATE_UNCONFIGURED = 1
    uint8 PRIMARY_STATE_INACTIVE = 2
    uint8 PRIMARY_STATE_ACTIVE = 3
    uint8 PRIMARY_STATE_FINALIZED = 4
    uint8 TRANSITION_STATE_CONFIGURING = 10
    uint8 TRANSITION_STATE_CLEANINGUP = 11
    uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
    uint8 TRANSITION_STATE_ACTIVATING = 13
    uint8 TRANSITION_STATE_DEACTIVATING = 14
    uint8 TRANSITION_STATE_ERRORPROCESSING = 15
    uint8 id
    string label

State goal_state
    uint8 PRIMARY_STATE_UNKNOWN = 0
    uint8 PRIMARY_STATE_UNCONFIGURED = 1
    uint8 PRIMARY_STATE_INACTIVE = 2
    uint8 PRIMARY_STATE_ACTIVE = 3
    uint8 PRIMARY_STATE_FINALIZED = 4
    uint8 TRANSITION_STATE_CONFIGURING = 10
    uint8 TRANSITION_STATE_CLEANINGUP = 11
    uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
    uint8 TRANSITION_STATE_ACTIVATING = 13
    uint8 TRANSITION_STATE_DEACTIVATING = 14
    uint8 TRANSITION_STATE_ERRORPROCESSING = 15
    uint8 id
    string label
```

### `sensor_msgs/msg/CameraInfo`

<a id="type-sensor-msgs-msg-camerainfo"></a>

**Used by topics:** [`/camera1/camera1/color/camera_info`](#topic-camera1-camera1-color-camera-info), [`/camera1/camera1/depth/camera_info`](#topic-camera1-camera1-depth-camera-info), [`/camera2/camera2/color/camera_info`](#topic-camera2-camera2-color-camera-info), [`/camera2/camera2/depth/camera_info`](#topic-camera2-camera2-depth-camera-info)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

uint32 height
uint32 width

string distortion_model

float64[] d

float64[9]  k

float64[9]  r

float64[12] p

uint32 binning_x
uint32 binning_y

RegionOfInterest roi
    uint32 x_offset
    uint32 y_offset
    uint32 height
    uint32 width
    bool do_rectify
```

### `sensor_msgs/msg/Image`

<a id="type-sensor-msgs-msg-image"></a>

**Used by topics:** [`/camera1/camera1/color/image_raw`](#topic-camera1-camera1-color-image-raw), [`/camera1/camera1/depth/image_rect_raw`](#topic-camera1-camera1-depth-image-rect-raw), [`/camera2/camera2/color/image_raw`](#topic-camera2-camera2-color-image-raw), [`/camera2/camera2/depth/image_rect_raw`](#topic-camera2-camera2-depth-image-rect-raw)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

uint32 height
uint32 width

string encoding

uint8 is_bigendian
uint32 step
uint8[] data
```

### `realsense2_camera_msgs/msg/Metadata`

<a id="type-realsense2-camera-msgs-msg-metadata"></a>

**Used by topics:** [`/camera1/camera1/color/metadata`](#topic-camera1-camera1-color-metadata), [`/camera1/camera1/depth/metadata`](#topic-camera1-camera1-depth-metadata), [`/camera2/camera2/color/metadata`](#topic-camera2-camera2-color-metadata), [`/camera2/camera2/depth/metadata`](#topic-camera2-camera2-depth-metadata)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id
string json_data
```

### `realsense2_camera_msgs/msg/Extrinsics`

<a id="type-realsense2-camera-msgs-msg-extrinsics"></a>

**Used by topics:** [`/camera1/camera1/extrinsics/depth_to_color`](#topic-camera1-camera1-extrinsics-depth-to-color), [`/camera2/camera2/extrinsics/depth_to_color`](#topic-camera2-camera2-extrinsics-depth-to-color)

```text
float64[9] rotation
float64[3] translation
```

### `sensor_msgs/msg/LaserScan`

<a id="type-sensor-msgs-msg-laserscan"></a>

**Used by topics:** [`/camera1/scan`](#topic-camera1-scan), [`/camera2/scan`](#topic-camera2-scan), [`/scan`](#topic-scan), [`/scan_filtered`](#topic-scan-filtered), [`/scan_merged`](#topic-scan-merged)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

float32 angle_min
float32 angle_max
float32 angle_increment

float32 time_increment
float32 scan_time

float32 range_min
float32 range_max

float32[] ranges
float32[] intensities
```

### `sensor_msgs/msg/PointCloud2`

<a id="type-sensor-msgs-msg-pointcloud2"></a>

**Used by topics:** [`/closest_gap`](#topic-closest-gap), [`/collision_obstacles`](#topic-collision-obstacles), [`/gaps`](#topic-gaps), [`/virt_gaps`](#topic-virt-gaps)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

uint32 height
uint32 width

PointField[] fields
    uint8 INT8    = 1
    uint8 UINT8   = 2
    uint8 INT16   = 3
    uint8 UINT16  = 4
    uint8 INT32   = 5
    uint8 UINT32  = 6
    uint8 FLOAT32 = 7
    uint8 FLOAT64 = 8
    string name      #
    uint32 offset    #
    uint8  datatype  #
    uint32 count     #

bool    is_bigendian
uint32  point_step
uint32  row_step
uint8[] data

bool is_dense
```

### `rosbridge_msgs/msg/ConnectedClients`

<a id="type-rosbridge-msgs-msg-connectedclients"></a>

**Used by topics:** [`/connected_clients`](#topic-connected-clients)

```text
ConnectedClient[] clients
    string ip_address
    builtin_interfaces/Time connection_time
        int32 sec
        uint32 nanosec
```

### `controller_manager_msgs/msg/ControllerManagerActivity`

<a id="type-controller-manager-msgs-msg-controllermanageractivity"></a>

**Used by topics:** [`/controller_manager/activity`](#topic-controller-manager-activity)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

NamedLifecycleState[] controllers
    string name
    lifecycle_msgs/State state
        uint8 PRIMARY_STATE_UNKNOWN = 0
        uint8 PRIMARY_STATE_UNCONFIGURED = 1
        uint8 PRIMARY_STATE_INACTIVE = 2
        uint8 PRIMARY_STATE_ACTIVE = 3
        uint8 PRIMARY_STATE_FINALIZED = 4
        uint8 TRANSITION_STATE_CONFIGURING = 10
        uint8 TRANSITION_STATE_CLEANINGUP = 11
        uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
        uint8 TRANSITION_STATE_ACTIVATING = 13
        uint8 TRANSITION_STATE_DEACTIVATING = 14
        uint8 TRANSITION_STATE_ERRORPROCESSING = 15
        uint8 id
        string label

NamedLifecycleState[] hardware_components
    string name
    lifecycle_msgs/State state
        uint8 PRIMARY_STATE_UNKNOWN = 0
        uint8 PRIMARY_STATE_UNCONFIGURED = 1
        uint8 PRIMARY_STATE_INACTIVE = 2
        uint8 PRIMARY_STATE_ACTIVE = 3
        uint8 PRIMARY_STATE_FINALIZED = 4
        uint8 TRANSITION_STATE_CONFIGURING = 10
        uint8 TRANSITION_STATE_CLEANINGUP = 11
        uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
        uint8 TRANSITION_STATE_ACTIVATING = 13
        uint8 TRANSITION_STATE_DEACTIVATING = 14
        uint8 TRANSITION_STATE_ERRORPROCESSING = 15
        uint8 id
        string label
```

### `pal_statistics_msgs/msg/Statistics`

<a id="type-pal-statistics-msgs-msg-statistics"></a>

**Used by topics:** [`/controller_manager/introspection_data/full`](#topic-controller-manager-introspection-data-full), [`/controller_manager/statistics/full`](#topic-controller-manager-statistics-full)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

Statistic[] statistics
    string name
    float64 value
```

### `pal_statistics_msgs/msg/StatisticsNames`

<a id="type-pal-statistics-msgs-msg-statisticsnames"></a>

**Used by topics:** [`/controller_manager/introspection_data/names`](#topic-controller-manager-introspection-data-names), [`/controller_manager/statistics/names`](#topic-controller-manager-statistics-names)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

string[] names
uint32 names_version
```

### `pal_statistics_msgs/msg/StatisticsValues`

<a id="type-pal-statistics-msgs-msg-statisticsvalues"></a>

**Used by topics:** [`/controller_manager/introspection_data/values`](#topic-controller-manager-introspection-data-values), [`/controller_manager/statistics/values`](#topic-controller-manager-statistics-values)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

float64[] values
uint32 names_version
```

### `geometry_msgs/msg/PoseArray`

<a id="type-geometry-msgs-msg-posearray"></a>

**Used by topics:** [`/desired_traj`](#topic-desired-traj)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

Pose[] poses
    Point position
        float64 x
        float64 y
        float64 z
    Quaternion orientation
        float64 x 0
        float64 y 0
        float64 z 0
        float64 w 1
```

### `diagnostic_msgs/msg/DiagnosticArray`

<a id="type-diagnostic-msgs-msg-diagnosticarray"></a>

**Used by topics:** [`/diagnostics`](#topic-diagnostics)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id
DiagnosticStatus[] status
    byte OK=0
    byte WARN=1
    byte ERROR=2
    byte STALE=3
    byte level
    string name
    string message
    string hardware_id
    KeyValue[] values
        string key
        string value
```

### `control_msgs/msg/DynamicJointState`

<a id="type-control-msgs-msg-dynamicjointstate"></a>

**Used by topics:** [`/dynamic_joint_states`](#topic-dynamic-joint-states)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

string[] joint_names
InterfaceValue[] interface_values
    string[] interface_names
    float64[] values
```

### `visualization_msgs/msg/Marker`

<a id="type-visualization-msgs-msg-marker"></a>

**Used by topics:** [`/footprint`](#topic-footprint)

```text
int32 ARROW=0
int32 CUBE=1
int32 SPHERE=2
int32 CYLINDER=3
int32 LINE_STRIP=4
int32 LINE_LIST=5
int32 CUBE_LIST=6
int32 SPHERE_LIST=7
int32 POINTS=8
int32 TEXT_VIEW_FACING=9
int32 MESH_RESOURCE=10
int32 TRIANGLE_LIST=11
int32 ARROW_STRIP=12

int32 ADD=0
int32 MODIFY=0
int32 DELETE=2
int32 DELETEALL=3

std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id
string ns
int32 id
int32 type
int32 action
geometry_msgs/Pose pose
    Point position
        float64 x
        float64 y
        float64 z
    Quaternion orientation
        float64 x 0
        float64 y 0
        float64 z 0
        float64 w 1
geometry_msgs/Vector3 scale
    float64 x
    float64 y
    float64 z
std_msgs/ColorRGBA color
    float32 r
    float32 g
    float32 b
    float32 a
builtin_interfaces/Duration lifetime
    int32 sec
    uint32 nanosec
bool frame_locked

geometry_msgs/Point[] points
    float64 x
    float64 y
    float64 z
std_msgs/ColorRGBA[] colors
    float32 r
    float32 g
    float32 b
    float32 a

string texture_resource
sensor_msgs/CompressedImage texture
    std_msgs/Header header #
        builtin_interfaces/Time stamp
            int32 sec
            uint32 nanosec
        string frame_id
    string format
    uint8[] data
UVCoordinate[] uv_coordinates
    float32 u
    float32 v

string text

string mesh_resource
MeshFile mesh_file
    string filename
    uint8[] data
bool mesh_use_embedded_materials
```

### `geometry_msgs/msg/PoseStamped`

<a id="type-geometry-msgs-msg-posestamped"></a>

**Used by topics:** [`/goal`](#topic-goal), [`/nav_goal`](#topic-nav-goal), [`/tracked_person/position`](#topic-tracked-person-position)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id
Pose pose
    Point position
        float64 x
        float64 y
        float64 z
    Quaternion orientation
        float64 x 0
        float64 y 0
        float64 z 0
        float64 w 1
```

### `sensor_msgs/msg/JointState`

<a id="type-sensor-msgs-msg-jointstate"></a>

**Used by topics:** [`/joint_states`](#topic-joint-states)

```text
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

string[] name
float64[] position
float64[] velocity
float64[] effort
```

### `caytu_msgs/msg/NetworkCommand`

<a id="type-caytu-msgs-msg-networkcommand"></a>

**Used by topics:** [`/network/command`](#topic-network-command)

```text
string action
string ssid
string password
```

### `caytu_msgs/msg/NetworkInfo`

<a id="type-caytu-msgs-msg-networkinfo"></a>

**Used by topics:** [`/network/state`](#topic-network-state)

```text
string ssid
string internal_ip
string external_ip
string gateway
string device_mac
string connection_status
WiFiNetwork[] available_networks
    string ssid
    int32 signal_strength
```

### `rcl_interfaces/msg/ParameterEvent`

<a id="type-rcl-interfaces-msg-parameterevent"></a>

**Used by topics:** [`/parameter_events`](#topic-parameter-events)

```text
builtin_interfaces/Time stamp
    int32 sec
    uint32 nanosec

string node

Parameter[] new_parameters
    string name
    ParameterValue value
        uint8 type
        bool bool_value
        int64 integer_value
        float64 double_value
        string string_value
        byte[] byte_array_value
        bool[] bool_array_value
        int64[] integer_array_value
        float64[] double_array_value
        string[] string_array_value

Parameter[] changed_parameters
    string name
    ParameterValue value
        uint8 type
        bool bool_value
        int64 integer_value
        float64 double_value
        string string_value
        byte[] byte_array_value
        bool[] bool_array_value
        int64[] integer_array_value
        float64[] double_array_value
        string[] string_array_value

Parameter[] deleted_parameters
    string name
    ParameterValue value
        uint8 type
        bool bool_value
        int64 integer_value
        float64 double_value
        string string_value
        byte[] byte_array_value
        bool[] bool_array_value
        int64[] integer_array_value
        float64[] double_array_value
        string[] string_array_value
```

### `person_following_robot/msg/TrackedObject`

<a id="type-person-following-robot-msg-trackedobject"></a>

**Used by topics:** [`/person_following_robot/color_data`](#topic-person-following-robot-color-data)

```text
string name
int32 id
bool success
float32[] position
```

### `std_msgs/msg/Bool`

<a id="type-std-msgs-msg-bool"></a>

**Used by topics:** [`/person_following_robot/follow_enable`](#topic-person-following-robot-follow-enable), [`/person_following_robot/gui_enable`](#topic-person-following-robot-gui-enable), [`/person_following_robot/gui_state`](#topic-person-following-robot-gui-state)

```text
bool data
```

### `rcl_interfaces/msg/Log`

<a id="type-rcl-interfaces-msg-log"></a>

**Used by topics:** [`/rosout`](#topic-rosout)

```text
uint8 DEBUG=10

uint8 INFO=20

uint8 WARN=30

uint8 ERROR=40

uint8 FATAL=50

builtin_interfaces/Time stamp
    int32 sec
    uint32 nanosec

uint8 level

string name

string msg

string file

string function

uint32 line
```

### `tf2_msgs/msg/TFMessage`

<a id="type-tf2-msgs-msg-tfmessage"></a>

**Used by topics:** [`/tf`](#topic-tf), [`/tf_static`](#topic-tf-static)

```text
geometry_msgs/TransformStamped[] transforms
    std_msgs/Header header
        builtin_interfaces/Time stamp
            int32 sec
            uint32 nanosec
        string frame_id
    string child_frame_id
    Transform transform
        Vector3 translation
            float64 x
            float64 y
            float64 z
        Quaternion rotation
            float64 x 0
            float64 y 0
            float64 z 0
            float64 w 1
```

### `caytu_msgs/msg/ActionCommand`

<a id="type-caytu-msgs-msg-actioncommand"></a>

**Used by topics:** [`/webrtc/command`](#topic-webrtc-command)

```text
string action
```
