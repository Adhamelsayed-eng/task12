# Task 12.1 – Build Your Own Robot 🤖

## 📌 Description
In this task, I designed and simulated a **4-wheel robot** with a **LiDAR sensor** using **URDF/Xacro** and tested it in **RViz** and **Gazebo**.

## 🛠️ Features
- **Base Link**: Main body of the robot.  
- **Four Wheels**: Two on the left and two on the right.  
- **LiDAR Sensor**: Mounted on the base link.  
- **URDF/Xacro**: Defines links, joints, and robot structure.  
- **Launch Files**:  
  - `display.launch` → to view robot in RViz.  
  - `gazebo.launch` → to spawn robot in Gazebo (optional).  

## 🚀 How to Run
1. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
# Task-12
