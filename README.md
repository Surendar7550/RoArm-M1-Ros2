RoArm-M1-Ros2
AI-Powered Control Suite for the RoArm-M1 Robotic Arm

(Hand Gesture • Voice Command • Human Pose Detection)

🌟 Overview

RoArm-M1-Ros2 is an open-source ROS2 project that allows you to control a robotic arm using:

✋ Hand Gestures (MediaPipe)

🗣 Voice Commands (VOSK Small English Model)

🕺 Human Pose Detection (MoveNet TFLite)

Each system contains two nodes:

1️⃣ A detection node
2️⃣ A controller node → sends commands to RoArm-M1 over serial

This makes the repo easy to understand, modify, and extend.

📁 Project Structure
RoArm-M1-Ros2/
│
├── Hand_Gesture/
│   ├── pick_drop/   → Detect hand gesture via camera
│   └── pick_con/    → Control RoArm-M1 via serial
│
├── Voice_control/
│   ├── voic_reg/    → Voice recognition (VOSK)
│   └── voic_con/    → Robot arm controller
│
├── Pose_Detection/
│   ├── human_pose_det/  → Human pose detection (MoveNet)
│   └── human_pose_con/  → Pose → robot motion controller

✋ 1. Hand Gesture Control
📌 Folders
Folder	Description
pick_drop	Detects hand gestures using camera
pick_con	Controls RoArm-M1 via serial
▶️ Run Commands
👉 Start hand gesture detection
ros2 run pick_drop pick_drop

👉 Start robotic arm controller (gesture → action)
ros2 run pick_con pick_con

🗣 2. Voice Control
📌 Folders
Folder	Description
voic_reg	Recognizes voice commands (VOSK small English model)
voic_con	Controls RoArm-M1 based on voice
▶️ Run Commands
👉 Start voice recognition
ros2 run voic_reg voic_reg

👉 Start voice → robot controller
ros2 run voic_con voic_con

🕺 3. Pose Detection Control
📌 Folders
Folder	Description
human_pose_det	Detects human pose using MoveNet TFLite
human_pose_con	Converts human pose → robot joint commands
▶️ Run Commands
👉 Start pose detection node
ros2 run human_pose_det human_pose_det

👉 Start pose → robotic arm controller
ros2 run human_pose_con human_pose_con

🔧 Dependencies
AI Models Required

(Not included in repo)

Feature	Model
Hand Gesture	MediaPipe Hands
Voice Recognition	VOSK vosk-model-small-en-in-0.4
Human Pose	MoveNet SinglePose (TFLite)

Place them inside a folder like:

models/

🔨 Build Instructions
cd ~/ros2_ws
colcon build
source install/setup.bash

⭐ Highlights

Runs completely offline

Works on ROS2 Humble

Minimal latency TFLite models

Clean modular architecture

Easy to add new controls

Ideal for robotics + AI demos

🤝 Contributing

Pull requests are welcome!
Feel free to improve detection accuracy, add new gestures, or create new robotic behaviors.

🧑‍💻 Author

Surendar K
Embedded IoT | Robotics | ROS2 | Computer Vision
GitHub: https://github.com/Surendar7550
