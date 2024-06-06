# Investigating low-cost quadcopters for automated indoor Facility Maintainance Management

This is a code repository from a master thesis in computer science at NTNU.
In this repository we link to all relevant reositories and packages neccesary to replicate the results in the research.

## Research
In this research we investigate the potential of leveraing low-cost quadcopters for automation in indoor Facility Maintainance Mangement by utilizing a Tello drone and the Ros2 framework.

These are some notable contributions to the open-source community of aerial robotics:
- Enable autonomous flight in 2-dimensions by configuring Nav2 to work with the Tello drone
- Enable simple real-time object avoidance using depth estimation models
- Convert data from ORB-SLAM3 to PointCloud2 in order to populate costmaps in Nav2

## Problems
We struggle to accurately localize the drone, much due to the poor readings from the tello drone and lack of sensor technologies. it would be interesting to see of someone could apply a smarter drone and voxel layers to this approach.

## Project structure
This is the structure of the project:

```bash
├── deviation_detection
│   └── Drone-deviation-service # https://github.com/Carlvebbesen/Drone-deviation-service
├── ros2
│   ├── depthimage_to_laserscan # https://github.com/FabianFoss/depthimage_to_laserscan.git
│   ├── inspection_ws
│   ├── libs
│   │   ├── DistDepth # https://github.com:facebookresearch/DistDepth.git
│   │   └── MiDaS # https://github.com/FabianFoss/MiDaS.git
│   ├── nav2_ws
│   ├── opencv_ws
│   │   └── vision_opencv # https://github.com/ros-perception/vision_opencv.git
│   ├── ORB_SLAM3_ROS2 # https://github.com:FabianFoss/ORB_SLAM3_ROS2.git
│   ├── rgb-to-depth-converter # https://github.com/FabianFoss/rgb-to-depth-converter.git
│   └── tello-ros2-driver # https://github.com/FabianFoss/tello-ros2-driver.git
└── web
    └── Drone-frontend # https://github.com/Carlvebbesen/Drone-frontend.git

```

## Dependencies
- Ros2
- OpenCV
- Python and various python libraries
- Distdepth (https://github.com:facebookresearch/DistDepth.git)
- ORB_SLAM3 (https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git)
- Node

## Running
Follow instruction in respective folders for `deviation_detection`, `ros2` and `web` 
