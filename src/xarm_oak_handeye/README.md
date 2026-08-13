# xArm6 OAK-D-SR eye-in-hand calibration

The OAK-D-SR in the supplied photographs is fixed to the xArm6 end flange.
This package therefore calibrates **eye-in-hand**: it estimates the fixed
`link_tcp -> oak_rgb_camera_optical_frame` transform while the supplied board
is kept fixed in the work area. The scripts never command robot motion.

## Board configuration

The defaults describe the supplied board as a 6 x 6 AprilTag GridBoard:

- tag black-square side: 16.5 mm
- overall extent: 133.65 mm
- derived clear gap between tags: 6.93 mm

The supplied DFOPTIX `Tag6-150-16.5mm` board uses `APRILTAG_36h11`. Its IDs
run from `0..5` on the physical bottom row upward to `30..35` on the top row;
the collector models this real layout rather than OpenCV's default ordering.

The current collector uses the board as a **6 x 6 outer-square centre grid**.
The centre pitch is `16.5 + 6.93 = 23.43 mm`. It needs only a few decoded
`APRILTAG_36h11` tags to orient the grid, then it uses all 36 detected black
outer squares as PnP points. This avoids the frame-to-frame loss of individual
tag payloads at the installed working distance. The board precision is
therefore still fully used, without requiring a separate chessboard target.

## Collection

Start the xArm driver in one terminal. This loads the real OAK-D-SR flange STL
as a MoveIt collision object; the camera body envelope is added after a
calibration result exists:

```bash
cd ~/ros2_ws
./start_arm_system.sh moveit
```

Then, with the supplied board rigidly fixed to the workcell (not the robot),
run:

```bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=40
ros2 run xarm_oak_handeye collect_eye_in_hand
```

Before collection, this camera-only test window can verify framing and board
detection without reading robot TF or commanding the arm:

```bash
ros2 run xarm_oak_handeye test_apriltag_board
```

The normal test and collector use the supplied `ColorCamera` `CAM_B` 800P
preview with automatic exposure, matching the existing image-capture tool.
They enlarge only the detection image 3x and map detected points back to the
native `1280 x 800` image before PnP. A valid frame reports
`Outer-square grid=36/36`; the verified single-frame reprojection error is
about `0.2 px`. The OAK firmware AprilTag node is not used for calibration.

The CAD mount is already rotated clockwise by 90 degrees as viewed from the
outside of the flange. Fully stop an existing MoveIt launch and start it again
after rebuilding the workspace so RViz receives this updated robot model.

Move the arm using the pendant or RViz, stop fully at each pose, wait one
second, and press Space in the camera window. Each press takes one fresh
image and looks up the TCP TF at that image's exposure timestamp. Capture at least 12 poses: place the board through
the image centre and edges, and vary roll/pitch/yaw by 20 degrees or more.
Press `S` to save the data. The default output is under `~/oak_handeye/`.

## Solve and use

```bash
ros2 run xarm_oak_handeye solve_eye_in_hand \
  --samples ~/oak_handeye/session_YYYYMMDD_HHMMSS/eye_in_hand_samples.npz

ros2 launch xarm_oak_handeye handeye_target_transform.launch.py \
  result:=~/oak_handeye/session_YYYYMMDD_HHMMSS/eye_in_hand_result.json \
  target_label:=orange
```

The solver writes JSON and YAML results, rejects inconsistent samples, and
reports the PnP and hand-eye residuals. Treat a result with more than roughly
3 mm RMS translation residual or 1 degree RMS rotation residual as a reason to
recapture the data. The launch file publishes the calibrated TCP-to-camera TF
and transforms existing `/oak_result` XYZ millimetre detections into
`/target_pose_in_base` using the live TF tree.

To start MoveIt with both the CAD mount and the calibrated, conservative
camera-body collision envelope:

```bash
ros2 run xarm_oak_handeye launch_moveit_with_oak \
  --result ~/oak_handeye/session_YYYYMMDD_HHMMSS/eye_in_hand_result.json
```

The supplied CAD mount is anchored at the `link_eef` flange origin and rotated
clockwise 90 degrees as installed (local Z `-90 deg`). Its mesh extent is
75.0 x 133.5 x 4.0 mm; in CAD coordinates its Y range is -37.5 to +96.0 mm,
so the collision mesh includes the camera-side support arms visible in the
supplied installation photographs. The `65 mm` camera envelope can be made larger with
`--camera-safety-radius-m`; it protects the sensor body and connector, while
the cable route still needs a separate workspace keep-out volume.
