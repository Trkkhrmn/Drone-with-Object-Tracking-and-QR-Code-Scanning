# Drone with Object Detection and QR Code Scanning

This project integrates object detection, QR code scanning, and laser control into a drone system using the **DroneKit** library for UAV control, **YOLOv8** for object detection, and **pyzbar** for QR code scanning. The drone can be controlled to follow a specific target (e.g., a red balloon) while also scanning for QR codes and performing actions based on detections.

## Features
- **Object Detection**: Uses YOLOv8 to detect specific objects (e.g., red balloons) in the camera feed.
- **QR Code Detection**: Detects and decodes QR codes in the camera feed. Displays the QR code data on the screen.
- **Laser Control**: Activates the drone's laser when the target is centered in the camera feed.
- **Autonomous Flight**: The drone can autonomously follow the detected target and return home when the battery is low.
- **Real-Time Video Stream**: Continuously processes the video feed to detect objects and QR codes.

## Requirements

### Hardware:
- **Drone**: A UAV with MAVLink compatibility.
- **Camera**: A camera connected to the Raspberry Pi (or other compatible system) to capture the video feed.
- **Laser Module**: Optional, controlled via GPIO pins on a Raspberry Pi (for target marking).

### Software:
- **Python 3.x**
- **Libraries**:
    - `opencv-python`: For video capture and image processing.
    - `ultralytics`: YOLOv8 object detection model.
    - `pyzbar`: For QR code scanning.
    - `dronekit`: For UAV communication.
    - `RPi.GPIO`: (Optional) For laser control via GPIO pins.
    - `numpy`: For numerical operations.
    - `time`, `math`, `os`: Standard Python libraries for timing, math operations, and file handling.

## Setup

### 1. Install Python Dependencies
To install the required Python packages, run the following command:

```bash
pip install opencv-python pyzbar dronekit ultralytics numpy
```

If you want to use laser control via **GPIO**, you'll need to install the `RPi.GPIO` library:

```bash
pip install RPi.GPIO
```

For YOLOv8 (object detection), you'll also need the **ultralytics** package:

```bash
pip install ultralytics
```

### 2. YOLOv8 Model
The project uses a pre-trained YOLO model (`best.pt`) located in the `train2/weights/` directory. If you do not have a model, you can train one using **YOLOv8** by following the instructions in the [YOLOv8 documentation](https://github.com/ultralytics/ultralytics).

### 3. Connecting the Drone
- Connect your drone via a serial connection (e.g., USB to serial or telemetry).
- Update the `DRONE_CONNECTION_STRING` variable with the appropriate connection string for your drone, such as `/dev/ttyACM0` or `udp:127.0.0.1:14550` for a simulator.

### 4. Video Stream
- The video stream will be captured from a camera (e.g., a USB camera). Ensure that your camera is properly connected and recognized.
- The video source is set by default to `/dev/video0`, but this can be changed based on your system.

### 5. GPIO for Laser Control
If you're using a Raspberry Pi with a connected laser module, the laser will be controlled via the **GPIO** pin. Modify the **`LASER_PIN`** to match your setup. By default, it's set to GPIO pin 17.

### 6. File Output
The captured frames from the video stream are saved in the `output_frames` directory, with filenames such as `frame_0001.jpg`, `frame_0002.jpg`, etc.

## Usage

### Running the Script
Once everything is set up, you can run the script using:

```bash
python drone_object_detection.py
```

### Drone Flight Modes
1. **Takeoff**: Upon starting the program, the drone will ask for the desired altitude and will take off automatically.
2. **Object Detection**: The YOLO model will continuously analyze the camera feed to detect the target object (e.g., a red balloon).
3. **Follow Mode**: Once a target is detected and centered in the camera feed, the drone will follow the target and activate the laser.
4. **QR Code Detection**: QR codes in the video feed are decoded and displayed on the screen, along with their data.
5. **Low Battery**: The drone monitors its battery level. If it drops below the specified threshold (default 20%), the drone will return to its home location.

### Keyboard Shortcuts
- **q**: Exit the program.
- **d**: Toggle debug mode (shows additional information in the terminal).

### Example Workflow
1. The drone takes off and starts analyzing the camera feed.
2. When a red balloon (or other target) is detected, the drone starts following the target.
3. If the QR code is detected, its data will be printed in the terminal, and the QR code will be highlighted on the screen.
4. If the target is centered and laser control is active, the drone will activate the laser.
5. The drone will return to its home location if the battery level is low.

## Troubleshooting

- **No Video Feed**: Ensure the camera is properly connected and the `VIDEO_SOURCE` is correctly set.
- **Connection Errors**: Double-check the drone's connection string and ensure the drone is powered on and ready to communicate.
- **QR Code Not Detected**: Make sure QR codes are clearly visible in the camera feed. You can adjust the camera settings (e.g., exposure, focus) to improve detection.
- **Low Battery**: Ensure that your drone is sufficiently charged. If running on simulation, monitor the simulated battery levels.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
