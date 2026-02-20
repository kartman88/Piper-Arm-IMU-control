# 🤖 Piper Robot Arm - IMU Control (ST SensorTile.box)

This project provides a Python-based interface to control an **Agilex Piper** robotic arm using real-time motion data from an **STMicroelectronics SensorTile.box**. The system uses the onboard IMU to capture orientation (quaternions) and maps these movements to the robot's joints.

## 📋 Prerequisites

### 1. Hardware Configuration (ST SensorTile.box)

Before running the Python script, you must configure the SensorTile.box to stream data via Serial:

* **Tool:** Use the **ST BLE Sensor app** or **STM32CubeIDE**.
* **Output:** Configure the device to send **Inertial Measurement Unit (IMU)** data as **Quaternions** over the USB/Serial connection.

### 2. System Dependencies (Linux)

Since the Piper arm communicates via CAN-bus, you need `can-utils` installed on your Linux machine:

```bash
sudo apt-get update
sudo apt-get install can-utils
```

## ⚙️ Installation & Setup

1. **Clone the Repository:**

   ```bash
   git clone git@github.com:kartman88/Piper-Arm-IMU-control.git
   cd piper-imu-control
   ```

2. **Environment Setup:**
   Create a virtual environment and install the required Python packages:

   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

3. **Device Registration (Linux Specific):**
   On Linux, the serial port for the SensorTile may require specific registration to be visible to the script. Execute the provided setup script:

   ```bash
   chmod +x register_sensor_tile.sh
   ./register_sensor_tile.sh
   ```

   *(Note: This step is generally not required on Windows systems).*

## 🚀 How to Run

Execute the main control script:

```bash
python robot_imu_control.py
```

### ⚠️ Operational Workflow (Follow Carefully)

1. **Safety Release**
   The script will prompt you to press **Enter**.

   > **WARNING:** Upon pressing Enter, the robot's motors will release torque and the arm will fall. **Ensure the arm is in a safe position or manually supported** before proceeding.

2. **Initial Positioning**
   The arm will automatically move into a default outstretched (distended) position.

3. **IMU Calibration**
   The script will ask for another **Enter** to begin a quick calibration.

   * Hold the SensorTile in your preferred "zero" or "neutral" orientation before pressing Enter.

4. **Live Control**
   Once calibrated, the robot is live:

   * **Rotation:** Turning the SensorTile will rotate the robot arm.
   * **Elbow/Elevation:** Tilting the device will control the elbow and vertical movement.

## 🛠 Tech Stack

* **Robot:** Agilex Piper (CAN communication)
* **Sensor:** STMicroelectronics SensorTile.box
* **Firmware/SDK:** STM32CubeIDE / Piper SDK
* **Language:** Python 3
