# indoor_navigation_system

In this pproject i have built the any system using camera that allows our system to know the excet location of our robot in a define arena. Arena is a predefine area whos corners are marked using Aruco marker. We have same mark on the robot too, the continueous processing on the feed of the camera is done to know the location of the robot. System is built in the ROS2 enviornment and the robot contain ESP32 as the microcontroller board, so we sended orentation and linear movment coordinates to the robt over wifi. All the computation is done on the laptop as we have created a local network over wifi for the data transfer.

This project implements an indoor autonomous navigation system using ROS2. 
All processing is done on a laptop, and movement commands are sent to the robot.


# Project Name

Short one-line description of the project.

Example:
A ROS2-based indoor mapping and vision system using smartphone camera streaming and OpenCV.

---

## 🚀 Features

- Real-time camera streaming
- ROS2 publisher/subscriber communication
- OpenCV image processing
- Indoor mapping support
- Easy to extend
- Works on Ubuntu + ROS2 Humble

---

## 🛠️ Technologies Used

- ROS2 Humble
- Python / C++
- OpenCV
- Ubuntu 22.04
- colcon build system

---

## 📂 Project Structure

```bash
project_name/
│── src/
│   ├── package_1/
│   ├── package_2/
│
│── launch/
│── scripts/
│── README.md
```

---

## ⚙️ Installation

### 1. Clone Repository

```bash
git clone https://github.com/yourusername/project_name.git
cd project_name
```

### 2. Build Workspace

```bash
colcon build
```

### 3. Source Workspace

```bash
source install/setup.bash
```

---

## ▶️ Running the Project

### Run Publisher Node

```bash
ros2 run package_name publisher_node
```

### Run Subscriber Node

```bash
ros2 run package_name subscriber_node
```

---

## 📸 Output

Add screenshots or GIFs here.

Example:

![Output Image](images/output.png)

---

## 🧠 How It Works

Explain the working flow briefly.

Example:

1. Smartphone camera streams video over WiFi.
2. OpenCV captures frames.
3. ROS2 node publishes frames.
4. Another node processes the data for mapping.

---

## 🔮 Future Improvements

- Add SLAM integration
- Improve object detection
- Add autonomous navigation
- Optimize performance

---

## 🤝 Contributing

Pull requests are welcome.

---

## 📜 License

This project is licensed under the MIT License.

---

## 👨‍💻 Author

Your Name  
Electronics & Communication Engineering  
Dharmsinh Desai University
