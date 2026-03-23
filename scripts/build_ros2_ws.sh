rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build ZED wrapper for SDK 5.2
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) --packages-select zed_components zed_wrapper zed_ros2 zed_debug

# build other packages, assuming ugv_sdk and ds4drv are installed
colcon build --symlink-install --packages-select scout_ros2 witmotion_ros scoutmini_nav2 scoutmini_description scoutmini_bringup ds4_driver