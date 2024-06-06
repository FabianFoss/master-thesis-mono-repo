### Running Ros2 environment 
To run the ros2 parts of the system, do the following:
1. Install and build dependencies
2. Glone this repo and submodules
3. Build packages in Ros2 folder (not libs and nav2_ws). Follow individual build instructions for each package.
4. Source `<package-name>/install/setup.bash` and add to `~/.bashrc`
5. Run `ros2/tello-ros2-driver/sripts/run.sh`, `rgb_to_depth_image`, `depthimage_to_laserscan` and ` cd ros2/nav2_ws/src & ros2 launch launch.py`
6. Configure Nav2 `ros2/nav2_ws/src/maps` and `ros2/nav2_ws/src/config` 
