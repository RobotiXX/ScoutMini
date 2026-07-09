# Bag Inventory

Use these bags in order. Do not begin live robot testing until the bag gates pass.

| Purpose | Bag | Useful topics |
| --- | --- | --- |
| Fast YOLO smoke test | `/home/nvidia/ssd/bags_for_yolo/rosbag2_2026_03_29-16_37_05` | `/equirectangular/image` |
| 360 + ZED image validation candidate | `/home/nvidia/ssd/valicor_bags/rosbag2_2026_06_23-16_18_31` | Metadata lists `/equirectangular/image`, `/zed2/zed_node/left/color/rect/image/compressed`; direct echo did not receive images during playback in the shadow-validation pass |
| Full robot TF/range integration | `/home/nvidia/ssd/bags/test/rosbag2_2025_04_09-18_38_18` | `/insta360_x4/image_raw/compressed`, ZED RGB/depth, `/tf`, `/tf_static`, `/odom`, `/velodyne_points` |
| Full robot TF/range integration | `/home/nvidia/ssd/bags/test/rosbag2_2025_04_09-18_44_21` | `/insta360_x4/image_raw/compressed`, ZED RGB, `/tf`, `/tf_static`, `/odom`, `/velodyne_points` |
| Raw dual-fisheye stress test | `/home/nvidia/ssd/rosbag2_2026_03_27-17_25_52` | `/dual_fisheye/image` |

The first implementation target is `/equirectangular/image` because it lets YOLO and tracking be tested without modifying the Insta360 driver.

The current passing AdaSCoRe shadow validation target is
`/home/nvidia/ssd/bags_for_yolo/rosbag2_2026_03_29-16_37_05`.
