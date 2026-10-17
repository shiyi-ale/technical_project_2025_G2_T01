# Raspberry Pi Autonomous Robot Car
### Infrared & OpenCV Line Following · PID Control · Ultrasonic/HSV Obstacle Detection · YOLO Object Detection

**English** | [简体中文](./README.zh-CN.md) · [Technical Report (PDF)](./technical_project_2025_G2_T01.pdf) · [Demo Videos](#demo-videos)

> A hands-on autonomous mobile robot project combining embedded GPIO control, classical computer vision, sensor fusion experiments and deep-learning-based object detection.
>
> **Course project:** ESIGELEC Technical Project · G2 · Team 01 · September 2025

## Overview

We developed and tested a Raspberry Pi-based four-wheel robot car through **six experimental demonstrations**. Instead of relying on a single navigation approach, the project explores **two line-following pipelines** and **two obstacle-triggered stopping approaches**, followed by an independent **YOLO object-detection study**.

**What the project actually demonstrates**

- **Infrared line following:** 4-channel sensors, discrete position-error estimation, proportional (P) steering, sharp-turn handling and lost-line search.
- **Camera line following:** OpenCV preprocessing, six-strip line centroids, path-shape estimation, PID steering and a dedicated large-turn state.
- **Ultrasonic obstacle stopping:** distance measurements averaged over 15 valid samples, with a 30 cm trigger, motor braking and a manual resume prompt.
- **Vision-triggered stopping:** HSV detection of red/yellow/blue regions, combined with camera-based line following to trigger a stop.
- **YOLO detection experiments:** a 2-class person detector (man/woman) and a 5-class fine-grained clothing/body-part detector, using Roboflow datasets and an Ultralytics inference pipeline.
- **Real hardware demos:** six videos covering the separate tasks.

**Scope:** This is a course prototype, not a complete self-driving or motion-planning stack. The obstacle modules primarily **detect and stop**, rather than plan a path around an obstacle. Although the report sets out an obstacle-avoidance/parking objective, the published repository does not establish a complete autonomous parking implementation.

## At a Glance

| Area | Implementation |
| --- | --- |
| Controller | Raspberry Pi with Python and RPi.GPIO |
| Sensors | Four infrared line sensors; ultrasonic ranging module |
| Vision | Front camera, MJPG-Streamer, OpenCV, NumPy |
| Motor control | GPIO direction control and PWM-driven DC motors |
| Navigation | Proportional control; PID and turn-handling state machine |
| Obstacle events | Ultrasonic distance threshold; HSV color-ratio threshold |
| AI experiments | Ultralytics YOLO, Roboflow datasets, PyTorch |
| Deliverables | Python source, 2-class/5-class datasets and model weights, six videos, technical report |

## System Architecture

~~~mermaid
flowchart TD
    P["Raspberry Pi + Python"] --> M["GPIO / PWM / DC motor driver"]
    IR["4-channel IR line sensors"] --> S["P-control line following"]
    S --> M
    US["Ultrasonic sensor"] --> U["Distance filter + obstacle stop"]
    U --> M
    C["Front-facing camera / MJPG stream"] --> V["OpenCV grayscale / blur / Otsu"]
    V --> X["6-strip centroids + PID + turn state"]
    X --> M
    C --> H["HSV red / yellow / blue region detection"]
    H --> M
    C --> Y["Ultralytics YOLO inference experiment"]
    Y --> O["Detection overlays + saved video"]
~~~

The YOLO demonstration is a **separate detection pipeline** in the current repository. It is not documented as a closed-loop YOLO-based driving controller.

## Core Experiments

### 1. Infrared Sensor Line Following

**Source:** [`1.★sensor_line_follow/★sensor_line_follow.py`](<./1.★sensor_line_follow/★sensor_line_follow.py>)

Four digital infrared sensors detect the relative location of a black track. The controller maps sensor patterns to a signed error and adjusts left/right PWM duty cycles:

~~~text
correction = Kp × error
left_pwm   = clamp(base_speed + correction, 0, 100)
right_pwm  = clamp(base_speed - correction, 0, 100)
~~~

The control flow also handles sharp left/right turns, intersections and line loss by using the last known deviation. **This module uses P control, not a full PID controller.**

### 2. OpenCV Camera Line Following

**Source:** [`2.★Camera-Line‐Following/★Camera-Line‐Following.py`](<./2.★Camera-Line‐Following/★Camera-Line‐Following.py>)

1. Capture frames from an MJPG video stream.
2. Convert to grayscale, apply a Gaussian blur and Otsu inverse threshold.
3. Divide the binary frame into **six horizontal strips**, and calculate the line centroid in each strip.
4. Estimate line offset, turn direction and local curvature using the detected centroids.
5. Use a **PID controller** for ordinary corrections; enter a separate `BIG_TURN_EXECUTE` state for large turns.
6. Detect line-loss conditions for special handling.

The published script sets `FRAME_WIDTH=64` and `FRAME_HEIGHT=48`, while the report describes a 128×96 processing setup. Treat these as different recorded configurations; check the actual script before reproducing an experiment.

### 3. Ultrasonic Obstacle Detection + Line Following

**Source:** [`3.★Ultrasonic Obstacle Avoidance/★Ultrasonic Obstacle Avoidance.py`](<./3.★Ultrasonic Obstacle Avoidance/★Ultrasonic Obstacle Avoidance.py>)

- Separate sensor/ranging and line-following threads.
- Shared obstacle flag and motor-control lock.
- **15 valid ultrasonic samples** averaged for distance estimation.
- Obstacle trigger at **0 < distance < 30 cm**.
- Brake and prompt for keyboard confirmation (`q`) before attempting to continue.

**Implementation note:** The current line-following thread uses `break` when an obstacle is detected, so automatic tracking resumption is **not guaranteed** by this released script. The repeated-valid-sample loop can also wait indefinitely if the sensor never produces valid readings; a timeout would be a useful improvement.

### 4. HSV Vision Obstacle Event

**Source:** [`4.★Vision Obstacle Avoidance/★Vision Obstacle Avoidance.py`](<./4.★Vision Obstacle Avoidance/★Vision Obstacle Avoidance.py>)

OpenCV converts the incoming camera frame to HSV and extracts predefined **red, yellow and blue** regions. When the colored-pixel ratio exceeds the configured threshold (described as about **2%** in the technical report), the robot brakes and ends its navigation loop.

This is **color-based visual event detection**, not general pedestrian recognition or YOLO-driven obstacle avoidance.

### 5. YOLO Detection Experiments

**Source:** [`5.★Deep Learning/`](<./5.★Deep Learning/>)

The project includes annotated datasets, trained weight files and video inference scripts for two experimental class configurations:

| Experiment | Labels | Reported mAP@0.5 | Reported mAP@0.5:0.95 |
| --- | --- | ---: | ---: |
| 2-class object detection | `man`, `woman` | ~0.95 | ~0.82 |
| 5-class fine-grained detection | `man cloth`, `man head`, `trousers`, `woman dress`, `woman head` | ~0.80* | ~0.75 |

*The report's training comparison also mentions ~0.90 mAP@0.5 for the 5-class setup; the table above uses its **inference-results summary** (~0.80). These are **report-provided approximate values**, not independently rerun benchmarks.*

**Research workflow:** data collection → Roboflow annotation/augmentation → YOLO-format export → training → validation → live video inference → visualization/video export.

**Version caveat:** The report refers to **YOLOv11**, whereas the committed 5-class training entry point initializes `yolov8n.pt` and points to a local pretrained checkpoint. Verify the specific model architecture using its weight/configuration files before reproducing or attributing results to one YOLO version.

## Demo Videos

Each demonstration is already included in the repository; click to view it on GitHub.

| # | Demonstration |
| --- | --- |
| 1 | [Infrared sensor line following](<./6.demo_video/1.★sensor_line_follow.mp4>) |
| 2 | [OpenCV camera line following](<./6.demo_video/2.★Camera-Line‐Following.mp4>) |
| 3 | [Ultrasonic obstacle stop](<./6.demo_video/3.★SensorUltrasonic Obstacle Avoidance.mp4>) |
| 4 | [HSV vision obstacle stop](<./6.demo_video/4.★Vision Obstacle Avoidance.mp4>) |
| 5 | [YOLO detection: man / woman](<./6.demo_video/5.★Deep Learning-Detection(man_woman).mp4>) |
| 6 | [YOLO fine-grained detection: five classes](<./6.demo_video/6.★Deep Learning – Classfication((man_headman_clothtrouserswoman_headwoman_dress)).mp4>) |

## Repository Layout

~~~text
technical_project_2025_G2_T01/
├── 1.★sensor_line_follow/
│   └── ★sensor_line_follow.py
├── 2.★Camera-Line‐Following/
│   └── ★Camera-Line‐Following.py
├── 3.★Ultrasonic Obstacle Avoidance/
│   └── ★Ultrasonic Obstacle Avoidance.py
├── 4.★Vision Obstacle Avoidance/
│   └── ★Vision Obstacle Avoidance.py
├── 5.★Deep Learning/
│   ├── Classficatiom/
│   │   ├── Predict/predict.py
│   │   ├── datasetPreparation/Dataset/
│   │   └── trainModel/
│   └── Detection(man_woman)/
│       ├── Predict/predict.py
│       ├── datasetPreparation/
│       └── trainModel/
├── 6.demo_video/             # Six MP4 demonstrations
├── schedule/schedule.png     # Project Gantt chart
├── technical_project_2025_G2_T01.pdf
└── README.md
~~~

**Path compatibility:** Several committed filenames contain spaces, the `★` character, and a Unicode hyphen. The `Classficatiom` spelling is also present in the actual directory. Paths are reproduced exactly to keep links working.

## Getting Started

### Hardware

Use a Raspberry Pi with the expected motor driver, four infrared tracking sensors, ultrasonic sensor and camera. **Verify the wiring and BCM GPIO assignments in each script before powering motors.** Run the robot on a test stand first and keep an emergency stop available.

### Python dependencies

On the target device, prepare a Python environment with the required libraries, for example:

~~~bash
python3 -m pip install numpy opencv-python
~~~

Motor-control scripts also require `RPi.GPIO` and a compatible Raspberry Pi GPIO environment. YOLO inference additionally needs `ultralytics` and its compatible PyTorch dependencies:

~~~bash
python3 -m pip install ultralytics
~~~

Choose a PyTorch build suitable for your actual CPU/GPU platform. The project does **not** currently provide a pinned, one-command environment or validated clean-install instructions.

### Run a robot module

From the repository root **on the correctly wired Raspberry Pi**:

~~~bash
python3 "1.★sensor_line_follow/★sensor_line_follow.py"
python3 "2.★Camera-Line‐Following/★Camera-Line‐Following.py"
python3 "3.★Ultrasonic Obstacle Avoidance/★Ultrasonic Obstacle Avoidance.py"
python3 "4.★Vision Obstacle Avoidance/★Vision Obstacle Avoidance.py"
~~~

Run **one motor-driving script at a time**. Camera scripts currently use `http://192.168.50.1:8080/?action=stream`; edit `STREAM_URL` to match the active MJPG-Streamer endpoint. If you are connected by SSH without a desktop, review `SHOW_VIDEO`.

### Run a YOLO experiment

The two `Predict/predict.py` examples require local configuration before execution:

- Replace the hard-coded Windows `H:\TechProject\...` model path with the checkpoint stored in this repository (or your own compatible checkpoint).
- Change the camera/stream address if necessary.
- Fix the device selection: the current inference call hard-codes `device='cuda:0'` even though startup checks whether CUDA is available.
- Check the class mapping and dependency versions against the chosen model weights.

The included scripts are **experimental reference implementations**, not a ready-to-run deployment package.

## Engineering Iterations & Lessons

The 39-page technical report records an iterative process:

- **Ultrasonic stop:** sequential ranging delayed steering; concurrent threads initially introduced stopping-priority problems; later filtering addressed noisy readings (versions 1–4).
- **Vision turns:** simple curve classification struggled with sharp turns; the team refined the large-turn state and its exit conditions over six versions.
- **Integration:** camera thresholds and image resolution, sensor validity, motor-command arbitration and error handling all affect real-world performance.

Remaining work includes safer line-loss braking, bounded ultrasonic sampling, more reliable thread lifecycle control, portable model paths, version-pinned dependencies and rigorous end-to-end tests.

## Project-Based Learning Value

This project can be decomposed into reproducible maker/robotics exercises: **GPIO and PWM → infrared sensing → P control → OpenCV segmentation → PID navigation → ultrasonic ranging → concurrent safety control → YOLO data annotation/training → integrated robot demonstration**. Each module has source code and a corresponding demo, making it suitable as a reference for explaining engineering trade-offs and debugging.

## Team & Documentation

**Team 01 · Group G2 · September 2025**

Zhao Hangjian · Zhou Hangyu · Jia Zihan

This is a **team project**. The repository and report do not establish exclusive ownership of every module by any one contributor.

- [Full technical report (39 pages)](./technical_project_2025_G2_T01.pdf)
- [Project schedule](./schedule/schedule.png)
- [中文完整说明](./README.zh-CN.md)

> Hardware motion must be tested in a controlled setting. Reported performance figures are taken from the 2025 technical report and have not been independently rerun for this README.