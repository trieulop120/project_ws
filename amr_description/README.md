# amr_description

Project-owned description of the AMR exported from SolidWorks.

## Supported model

- Xacro: `urdf/robots/complete_robot_amr.urdf.xacro`
- CAD meshes: `meshes/complete_robot_amr/`
- simulation-only plugins: `urdf/macros/sim_gazebo.urdf.xacro`
- small inertial helpers: `urdf/macros/inertial.urdf.xacro`

The SolidWorks export is the source of truth for geometry, link origins,
joint axes, mass and inertia. ROS/Gazebo adaptations add `base_footprint`,
stable primitive collision geometry, sensor topics, mecanum drive and arm
command interfaces. Do not reintroduce generic sample robot profiles as a
second baseline.

## Stable RViz interfaces

- `rviz/slam.rviz`: map, TF, robot, LiDAR, RGB, depth and optional depth cloud.
- `rviz/navigation.rviz`: the full Nav2 panel, AMCL particles, global/local
  costmaps and plans, robot, LiDAR, RGB, depth and optional depth cloud.

The top-level runtime chooses one of these files deterministically. Use
**Save Config As** for experiments; do not overwrite the two tracked profiles.

## Frames and topics

The navigation chain is `map -> odom -> base_footprint -> base_link`. Sensors
use `laser_frame`, `camera_mount_link`, `camera_color_frame`,
`camera_depth_frame` and `imu_link`. Wheel joints remain independent for the
four mecanum wheels. Runtime sensor topics are `/scan`, `/imu/data`,
`/camera/color/image_raw`, `/camera/depth/image_rect_raw` and
`/camera/depth/color/points`.