# Gesture-Controlled Robot (ROS 2)

A modular ROS 2 project enabling robot control via hand gestures. The system integrates gesture recognition, motion translation, and robot control into a cohesive pipeline with RViz visualization support.

---

## 📦 Package Overview

### 1. `gesture_recognition`
- Detects hand gestures using computer vision.
- **Nodes:**
  - `gesture_node_v1.py` – basic version
  - `gesture_node_v2.py` – improved version
- **Launch File:**  
  `gesture_recognition/launch/gesture_recognition.launch.py`

### 2. `motion_translator`
- Translates detected gestures into velocity commands.
- **Node:**  
  `motion_translator_node.py`
- **Launch File:**  
  `motion_translator/launch/motion_translator.launch.py`

### 3. `robot_control`
- Receives motion commands and visualizes robot motion.
- **Node:**  
  `robot_node.py`
- **Visualization:**  
  RViz config file: `robot_control/rviz/robot_visual.rviz`
- **Launch File:**  
  `robot_control/launch/robot_control.launch.py`

### 4. `unified_launch`
- Launches the full pipeline: gesture detection, translation, control, and visualization.
- **Main Launch File:**  
  `unified_launch/launch/gesture_controlled_robot.launch.py`

---
## How to Run (Module-wise)

### 1. Gesture Node
```bash
ros2 run gesture_recognition gesture_node_v1
```
### 2. Motion Translator
```bash
ros2 run motion_translator motion_translator_node
```
### 3. Robot Control + RViz
```bash
ros2 launch robot_control robot_control.launch.py
```
## Full System Launch
```
ros2 launch unified_launch gesture_controlled_robot.launch.py
```