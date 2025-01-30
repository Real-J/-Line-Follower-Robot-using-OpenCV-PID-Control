# 🚀 Line Follower Robot using OpenCV & PID Control

## 🏎️ Overview
This project implements a **Line Follower Robot** using **OpenCV for vision processing** and **PID control** for smooth navigation. The robot follows a **black line on a white background**, using a **camera, Raspberry Pi, and motor driver**.

## 📺 How It Works
1. Captures real-time **video feed** using a camera.
2. Converts the frame to **grayscale** and applies **thresholding** to detect the black line.
3. Uses **moment analysis** to determine the position of the line.
4. Implements a **PID controller** to adjust motor speeds based on error.
5. The robot continuously adjusts its direction to stay on the line.

---

## 🔧 Hardware Requirements
- ✅ **Raspberry Pi** (or any compatible microcontroller)
- ✅ **Pi Camera / USB Webcam**
- ✅ **Motor Driver Module (L298N)**
- ✅ **Two DC Motors & Wheels**
- ✅ **Battery Pack**
- ✅ **Chassis & Line Track**

---

## 🖥️ Software Requirements
- Python 3.x
- OpenCV (`pip install opencv-python`)
- NumPy (`pip install numpy`)
- RPi.GPIO (`pip install RPi.GPIO` for Raspberry Pi)

---

## ⚡ Installation & Setup
### 1️⃣ Clone the Repository
```bash
git clone https://github.com/Real-J/line-follower-robot.git
cd line-follower-robot
```

### 2️⃣ Install Dependencies
```bash
pip install -r requirements.txt
```

### 3️⃣ Run the Line Follower Code
```bash
python line_follower.py
```

---

## 🎯 PID Controller
The **PID controller** ensures smooth motion by reducing oscillations and improving accuracy.

- **Kp (Proportional Gain):** Adjusts based on how far the robot is from the center.
- **Ki (Integral Gain):** Accumulates past errors (useful for long-term corrections).
- **Kd (Derivative Gain):** Predicts future errors and prevents overshooting.

🔧 **Tuning Tip:** Adjust **Kp, Ki, Kd** values based on your environment.

---

## 🚀 Future Improvements
- 🔹 **Deep Learning-based Path Detection** (Train CNN to detect different routes)
- 🔹 **Obstacle Avoidance** (Integrate Ultrasonic Sensors)
- 🔹 **Edge AI Optimization** (Deploy on NVIDIA Jetson Nano)
- 🔹 **Self-Supervised Learning** for complex navigation

---

## 🤝 Contributing
Contributions are welcome! To contribute:
1. Fork the repository 🍴
2. Create a new branch: `git checkout -b feature-name`
3. Commit changes: `git commit -m "Add new feature"`
4. Push to branch: `git push origin feature-name`
5. Open a **Pull Request**

---

## 🐜 License
This project is **open-source** under the **MIT License**. Feel free to use and modify it.

---

