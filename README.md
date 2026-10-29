# ROBOCON 2027 🤖⚡

This repository contains the code, PCB designs, CAD models, and documentation for Team Robocon's robots for ABU ROBOCON 2027.

# About 📌

ABU ROBOCON is an international robotics competition that challenges teams to design and build autonomous and manual robots capable of completing game-specific tasks with precision, speed, and strategy. This repository serves as the central hub for software, electronics, and mechanical design for our 2027 robots.

# Features 🚀

- 🤖 Autonomous Control – Path planning, motion control, and autonomous strategy execution.
- 🎮 Manual Control – Responsive teleoperation with custom controllers.
- 🎯 Computer Vision & AI – Object detection, localization, and intelligent decision-making.
- ⚡ Real-time Systems – Embedded control with microcontrollers and high-performance processors.
- 🔌 PCB Design – Custom electronics designed using Altium/KiCad for motor drivers, power management, and sensor integration.
- 🛠 Mechanical Design – SolidWorks-based CAD modeling, simulation, and prototyping of mechanisms.
- 🧪 Simulation & Testing – Tools and frameworks for debugging, validation, and iterative development before deployment.

## RealSense YOLO

`realsense/main.py` reads color and aligned depth images from ROS 2 topics. Start the RealSense ROS 2 camera driver first, then run the script in a second terminal. Both terminals must use the same ROS 2 distribution and domain.

### Terminal 1: start the camera

Source ROS 2, then launch the RealSense camera with depth aligned to color and color/depth synchronization enabled:

```bash
source /opt/ros/<ros-distro>/setup.bash
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_sync:=true align_depth.enable:=true
```

Confirm the expected topics are present:

```bash
ros2 topic list | grep -E '^/camera/(color/image_raw|aligned_depth_to_color/image_raw)$'
```

The script expects `/camera/color/image_raw` and `/camera/aligned_depth_to_color/image_raw`. If your camera uses a different namespace, update `COLOR_TOPIC` and `DEPTH_TOPIC` in `realsense/main.py`.

### Terminal 2: run YOLO

Source the same ROS 2 distribution (and your workspace overlay, if applicable), then run:

```bash
source /opt/ros/<ros-distro>/setup.bash
# source <workspace>/install/setup.bash  # if using a ROS 2 workspace overlay
python3 realsense/main.py
```

Use a Python environment where `cv_bridge`, OpenCV, NumPy, PyTorch, Ultralytics, and the required CUDA/TensorRT dependencies are available. Set `MODEL_PATH` in `realsense/main.py` to the location of `best.engine` on your machine. An OpenCV window shows the YOLO preview; press `q` to close it. Detection bounds and depth are published on `/bb_vals` as `geometry_msgs/Vector3` (`x` = left bound, `y` = right bound, `z` = depth in meters). Depth images with `16UC1` (millimeters) or `32FC1` (meters) encoding are supported.

# Tech Stack

- **Embedded:** Teensy 4.1
- **Processing:** NVIDIA Jetson, OpenCV, CUDA, ROS 2
- **Electronics:** Altium Designer
- **Mechanical:** SolidWorks for CAD and mechanism design
- **Languages:** C, C++, Python
- **Frameworks:** YOLO, PyTorch, TensorRT, Docker
- **Collaboration:** GitHub Projects, Issues, and CI/CD

# Goal 🏆

To engineer two highly efficient, robust, and innovative robots by integrating software, electronics, and mechanical engineering, with the objective of delivering outstanding performance at **ABU ROBOCON 2027** and representing the team at the highest international level.
