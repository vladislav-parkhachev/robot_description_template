# TurtleBot3 Burger Description

This repository provides a **ROS 2 description package** for the TurtleBot3 Burger robot.  
It includes URDF/Xacro, RViz configuration, launch files, a component config, and Docker support through a `Makefile`.

---

## Branches

- **main** → template skeleton (minimal URDF, launch files, and RViz config)  
- **turtlebot3_burger** → TurtleBot3 Burger robot description with lidar and camera options  

---

## Package contents

- `urdf/turtlebot3_burger.urdf.xacro` → main robot description for TurtleBot3 Burger  
- `urdf/robot_arguments.xacro` → common Xacro arguments (sensors and options)  
- `urdf/rplidar_a2m8.xacro` → RPLIDAR A2 model  
- `urdf/realsense_d435.xacro` → Intel RealSense D435 model  
- `launch/robot_description.launch.py` → loads URDF only (no GUI)  
- `launch/robot_view.launch.py` → loads URDF, RViz, and optionally GUI sliders  
- `config/robot_view.rviz` → RViz configuration file  
- `config/robot_components.yaml` → configuration for enabling/disabling sensors  
- `Makefile` → simplified entrypoint for Docker build & run  

---

## 🛠️ Setup workspace (native)

Create a new ROS 2 workspace and clone this repository:

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone -b turtlebot3_burger https://github.com/vladislav-parkhachev/robot_description_template.git
cd ..
```

Install dependencies:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Usage with Makefile

To build and run the TurtleBot3 Burger model with RViz:

```bash
make run_robot_view
```

This command will:
1. Allow X11 access (`xhost +local:docker`)  
2. Build the Docker image `robot_description:latest`  
3. Run the container with `ros2 launch robot_description robot_view.launch.py`  
4. Revoke X11 access after exit (`xhost -local:docker`)  

---

## 🎛 Parameters

The TurtleBot3 Burger description supports the following components:

- **add_rplidar_a2**  
  Enable or disable the RPLIDAR A2 sensor.  
  ```yaml
  add_rplidar_a2: 'true'
  ```

- **add_realsense_d435**  
  Enable or disable the Intel RealSense D435 camera.  
  ```yaml
  add_realsense_d435: 'false'
  ```

These parameters can be set in `config/robot_components.yaml`.  

---

## ⚙️ Configuration

Example `config/robot_components.yaml`:

```yaml
add_rplidar_a2: 'true'
add_realsense_d435: 'true'
```

This allows enabling/disabling sensors without modifying the URDF/Xacro files.  

---

## 📌 Notes

- This description is based on the official **TurtleBot3 Burger** model.  
- By default, the robot includes **RPLIDAR A2** and **Intel RealSense D435**.  
- Sensors can be toggled via `config/robot_components.yaml`.  
- Docker + Makefile allow running the description without installing ROS 2 locally.  