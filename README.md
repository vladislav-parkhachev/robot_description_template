# Robot Description Template

This repository provides a **ROS 2 template package** for creating robot description packages (`*_description`).  
It includes a minimal URDF, RViz configuration, launch files, and optional Docker support.

---

## Branches

- **main** → template skeleton (minimal URDF, launch files, and RViz config)  
- **turtlebot3_burger** → example with a real robot description  

---

## Package contents

- `urdf/robot.urdf.xacro` → minimal URDF (blue base + green movable link)  
- `launch/robot_description.launch.py` → loads URDF only (no GUI)  
- `launch/robot_view.launch.py` → loads URDF, RViz, and either `joint_state_publisher` or `joint_state_publisher_gui`  
- `config/robot_view.rviz` → RViz configuration file  
- `Dockerfile` → container setup for ROS 2 + RViz  

---

## 🛠️ Setup workspace

Create a new ROS 2 workspace and clone this repository:

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/vladislav-parkhachev/robot_description_template.git
cd ..
```

---

## 🚀 Usage (native)

### Install dependencies
(recommended with `rosdep`):

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Build the workspace

```bash
colcon build
source install/setup.bash
```

### Launch robot description only

```bash
ros2 launch robot_description robot_description.launch.py
```

### Launch with RViz (default)

```bash
ros2 launch robot_description robot_view.launch.py
```

### Launch with GUI sliders

```bash
ros2 launch robot_description robot_view.launch.py jsp_gui:=true
```

---

## 🐳 Usage (Docker)

### Build image

```bash
docker build -t robot_description:latest .
```

### Run container with GUI (RViz)

```bash
xhost +local:docker

docker run -it --rm     --net=host     -e DISPLAY=$DISPLAY     -e QT_X11_NO_MITSHM=1     -v /tmp/.X11-unix:/tmp/.X11-unix:rw     robot_description:latest     ros2 launch robot_description robot_view.launch.py
```

👉 This will start RViz inside the container and show the minimal robot model.

---

## 📌 Notes

- The template shows a simple **blue base link** and a **green movable link**.  
- `base_footprint` is fixed on the ground.  
- The movable link can be rotated using `joint_state_publisher_gui` sliders.  
- Docker allows you to run the template without installing ROS 2 locally.  

Use this as a starting point for your own robot description package 🚀