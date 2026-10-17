# 树莓派自主导航智能小车
### 红外 / OpenCV 双方案循迹 · PID 运动控制 · 超声波与视觉障碍检测 · YOLO 目标识别

[English](./README.md) | **简体中文** · [39 页技术报告](./technical_project_2025_G2_T01.pdf) · [演示视频](#演示视频)

> 一个融合树莓派 GPIO 嵌入式控制、计算机视觉、传感器数据处理与深度学习的移动机器人综合实践项目。
>
> **项目背景：** ESIGELEC Technical Project · G2 组第 01 队 · 2025 年 9 月

## 一、项目概述

本项目基于 **Raspberry Pi（树莓派）四轮智能车**，围绕自主循迹、障碍检测与目标识别，设计、实现并演示了六项技术实验。

项目不仅使用传统红外传感器进行循迹，还采用摄像头与 OpenCV 建立视觉循迹方案；在障碍检测方面，分别探索超声波测距与 HSV 颜色识别；最后通过 Roboflow 数据集和 Ultralytics YOLO 完成两组目标检测实验。

**已公开的六类实验：**

1. **红外传感器循迹：** 四路红外检测、偏差量化、P 比例控制、急转弯与丢线找回策略。
2. **摄像头视觉循迹：** OpenCV 图像预处理、六段质心提取、PID 校正及大弯道状态机。
3. **超声波障碍检测：** 15 次有效测距平均值滤波、30 cm 停车阈值、多线程协作与人工恢复提示。
4. **HSV 视觉停车：** 检测红 / 黄 / 蓝目标色域面积比例，触发停车响应。
5. **YOLO 两类别检测：** `man` 和 `woman` 的目标检测。
6. **YOLO 五类别细粒度检测：** 人体头部、服装、裤子等类别的目标检测。

**实现边界：** 这是课程工程原型，而不是完整的无人驾驶产品。公开的“避障”模块主要完成**检测障碍并停车**，没有证明可以自主绕过障碍并重新规划路径。尽管报告提出“避障泊车”的项目目标，当前仓库也不足以证明完整自主泊车功能已经交付。

## 二、系统软硬件架构

| 层次 | 主要配置 | 功能 |
| --- | --- | --- |
| 主控制器 | Raspberry Pi、Python、RPi.GPIO | 传感器读取、决策逻辑、GPIO/PWM 控制 |
| 运动系统 | 直流电机、驱动板、四轮底盘 | 前进、后退、原地转弯和制动 |
| 红外感知 | 四路红外循迹传感器 | 黑线位置与偏差估计 |
| 距离感知 | 超声波测距模块 | 前方距离采样与停车触发 |
| 视觉感知 | 前置摄像头、MJPG-Streamer | 视频图像输入 |
| 传统视觉算法 | OpenCV、NumPy | 二值化、质心分析、曲率估计、HSV 阈值检测 |
| 运动控制算法 | P、PID、状态机 | 速度差调整、转向控制与大弯道处理 |
| 深度学习 | Roboflow、Ultralytics、PyTorch | 数据标注、目标检测、推理可视化 |

### 系统数据流

~~~mermaid
flowchart TD
    A["Raspberry Pi + Python"] --> G["GPIO / PWM → 电机驱动"]
    B["4 路红外传感器"] --> C["偏差计算 + P 比例控制"]
    C --> G
    D["超声波测距"] --> E["测距滤波 / 停车优先"]
    E --> G
    F["摄像头 / MJPG 视频流"] --> H["OpenCV 图像预处理"]
    H --> I["六段质心 + PID / 大弯道状态机"]
    I --> G
    F --> J["HSV 目标颜色检测"]
    J --> G
    F --> K["YOLO 独立目标检测实验"]
    K --> L["识别框 / 标签 / 视频输出"]
~~~

**特别说明：** 仓库中的 YOLO 模型识别与机器人运动控制是独立实验模块，尚不能表述为“YOLO 检测直接驱动小车自主避障”。

## 三、核心算法实现

### 3.1 四路红外循迹：P 比例控制

[查看源代码](<./1.★sensor_line_follow/★sensor_line_follow.py>) · [观看演示](<./6.demo_video/1.★sensor_line_follow.mp4>)

四路数字红外传感器对黑线进行采样。程序根据传感器状态组合判断小车相对黑线的位置，将误差映射到左右电机 PWM 占空比：

~~~text
correction = Kp × error

left_speed  = clamp(base_speed + correction, 0, 100)
right_speed = clamp(base_speed - correction, 0, 100)
~~~

正常巡线时采用比例控制；当检测到急弯、路口或丢线时，进入相应特殊处理逻辑。最后一次有效偏差 `last_error` 用于丢线后的找回方向判断。

注意：这一模块采用的是 **P 控制**，并非完整的 PID 控制。

### 3.2 基于 OpenCV 的视觉循迹：质心 + PID + 状态机

[查看源代码](<./2.★Camera-Line‐Following/★Camera-Line‐Following.py>) · [观看演示](<./6.demo_video/2.★Camera-Line‐Following.mp4>)

核心处理流程：

1. 通过网络视频流读取摄像头帧。
2. 使用灰度化、5×5 高斯模糊和 Otsu 反向二值化提取黑色路径。
3. 将图像划分为 **6 个水平条带**，分别计算黑线的横向质心。
4. 根据有效质心点的位置、局部夹角和整体偏移，估计左右方向及转弯程度。
5. 正常情况下通过 PID 控制左右电机差速。
6. 大幅转弯时切换到 `BIG_TURN_EXECUTE` 状态，使用自适应原地转向，并依据质心排列退出大弯道状态。

该方案的核心价值，是把“图像中的路径位置”转化成“机器人能够执行的速度控制量”。

**参数注意：** 已提交代码实际使用 `64×48` 图像处理分辨率，PDF 报告描述的是 `128×96` 的配置。复现应以实际脚本参数为准，不应把两者当作同一次实验配置。

### 3.3 超声波测距与多线程停车

[查看源代码](<./3.★Ultrasonic Obstacle Avoidance/★Ultrasonic Obstacle Avoidance.py>) · [观看演示](<./6.demo_video/3.★SensorUltrasonic Obstacle Avoidance.mp4>)

为减少同步测距造成的巡线响应延迟，项目尝试将**超声波距离检测**与**红外循迹**放在不同线程中，并用共享标志和线程锁协调底层电机控制。

- 通过 Trig/Echo 引脚计算回声往返时间，换算目标距离。
- 连续收集 **15 次有效距离** 并取平均，降低偶发错误数据的影响。
- 当 `0 < distance < 30 cm` 时设置障碍标志并执行制动。
- 程序提示用户输入 `q`，作为人工恢复操作的一部分。

**代码限制：** 当前提交版本的循迹线程在障碍标志被设置时执行 `break`，所以不能保证在按 `q` 后自动继续巡线。此外，如果始终得不到有效测距数据，收集 15 个有效值的循环可能长期阻塞。后续应加入超时、完整的暂停/恢复状态机及异常处理。

### 3.4 HSV 颜色检测与视觉停车

[查看源代码](<./4.★Vision Obstacle Avoidance/★Vision Obstacle Avoidance.py>) · [观看演示](<./6.demo_video/4.★Vision Obstacle Avoidance.mp4>)

在摄像头视觉巡线流程中加入基于 HSV 色彩空间的事件触发：

1. 将摄像头图像从 BGR 转换至 HSV。
2. 分别建立红色、黄色、蓝色范围的二值 Mask。
3. 合并 Mask，计算指定颜色像素在整帧中的占比。
4. 当颜色占比超过设定阈值时，调用 `brake()` 并终止当前控制循环。

PDF 报告给出的默认颜色面积占比阈值约为 **2%**。

这是一种通过**颜色特征模拟障碍事件**的视觉停车方案，不等同于对任意行人或障碍物的通用语义识别。

## 四、YOLO 深度学习实验

[查看 Deep Learning 目录](<./5.★Deep Learning/>)

项目采用 Roboflow 管理图像、标注和数据增强，构建并评估两套基于 Ultralytics 推理接口的目标检测实验。

### 4.1 数据集与识别任务

| 实验 | 类别数 | 类别定义 |
| --- | ---: | --- |
| 人员检测 | 2 | `man`、`woman` |
| 细粒度人体/服饰检测 | 5 | `man cloth`、`man head`、`trousers`、`woman dress`、`woman head` |

虽然仓库将第二项实验命名为 `Classficatiom`，但从数据集 YOLO 边界框标注和推理输出看，实际是**五类别目标检测任务**，而非单纯的整图分类。

### 4.2 实验报告结果

| 指标 | 2 类检测 | 5 类细粒度检测 |
| --- | ---: | ---: |
| Precision | 约 0.93 | 约 0.88 |
| Recall | 约 0.90 | 约 0.85 |
| mAP@0.5 | 约 0.95 | 约 0.80* |
| mAP@0.5:0.95 | 约 0.82 | 约 0.75 |

*注：报告的训练对比表中，五类别实验 mAP@0.5 也曾写为约 0.90。以上统一采用 PDF 第 38 页的“推理结果汇总”口径（约 0.80）。这里展示的是**报告记录的数据**，并非本次重新运行训练所得。

数据流程为：**视频/图片采集 → Roboflow 标注与增强 → YOLO 数据格式导出 → 模型训练和验证 → 视频推理 → 绘制检测框与输出视频。**

报告将训练路线描述为 **YOLOv11**，但公开的五类别 `train.py` 以 `yolov8n.pt` 初始化模型，同时引用外部本地权重。因此，复现时需要核验实际权重对应的模型版本，不能仅根据报告文字认定所有权重均由 YOLOv11 训练。

### 4.3 视频推理功能

- 优先读取 MJPG-Streamer 网络视频流，失败时尝试本地摄像头。
- 调用 Ultralytics YOLO 输出类别、置信度和边界框。
- 通过 OpenCV 绘制检测结果。
- 统计逐帧推理时间、显示 FPS，并保存 AVI 视频。
- 按 `q` 退出并释放设备资源。

对应演示：[2 类人员检测](<./6.demo_video/5.★Deep Learning-Detection(man_woman).mp4>) · [5 类细粒度检测](<./6.demo_video/6.★Deep Learning – Classfication((man_headman_clothtrouserswoman_headwoman_dress)).mp4>)

## 五、演示视频

仓库已包含六段真实实验视频，可以直接查看，不依赖额外的视频网站。

| 序号 | 实验名称 | 演示 |
| --- | --- | --- |
| 01 | 红外四路循迹 | [查看 MP4](<./6.demo_video/1.★sensor_line_follow.mp4>) |
| 02 | OpenCV 摄像头巡线 | [查看 MP4](<./6.demo_video/2.★Camera-Line‐Following.mp4>) |
| 03 | 超声波检测与停车 | [查看 MP4](<./6.demo_video/3.★SensorUltrasonic Obstacle Avoidance.mp4>) |
| 04 | HSV 视觉检测与停车 | [查看 MP4](<./6.demo_video/4.★Vision Obstacle Avoidance.mp4>) |
| 05 | YOLO 两类别检测 | [查看 MP4](<./6.demo_video/5.★Deep Learning-Detection(man_woman).mp4>) |
| 06 | YOLO 五类别细粒度检测 | [查看 MP4](<./6.demo_video/6.★Deep Learning – Classfication((man_headman_clothtrouserswoman_headwoman_dress)).mp4>) |

## 六、项目目录结构

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
├── 6.demo_video/                   # 6 段演示视频
├── schedule/schedule.png           # 项目甘特图
├── technical_project_2025_G2_T01.pdf
├── README.md
└── README.zh-CN.md
~~~

注意：目录名中的空格、`★`、特殊连字符及 `Classficatiom` 拼写均与实际仓库保持一致，没有擅自修改原文件路径。

## 七、运行与复现

### 7.1 环境准备

需要具备符合脚本 GPIO 引脚映射的树莓派、小车电机驱动板、四路红外传感器、超声波传感器和可用摄像头。

基础视觉依赖（示例）：

~~~bash
python3 -m pip install numpy opencv-python
~~~

GPIO 控制需要在兼容的树莓派环境中安装 `RPi.GPIO`；YOLO 推理另外需要安装 `ultralytics` 及兼容的 PyTorch：

~~~bash
python3 -m pip install ultralytics
~~~

实际 PyTorch 版本应与运行设备的 CPU/GPU 相适配。本项目当前没有完整的锁定依赖文件及可保证一键安装成功的环境配置。

### 7.2 运行四项机器人实验

在对应**树莓派硬件上**、进入仓库根目录后，按需要选择**其中一项**执行：

~~~bash
python3 "1.★sensor_line_follow/★sensor_line_follow.py"
python3 "2.★Camera-Line‐Following/★Camera-Line‐Following.py"
python3 "3.★Ultrasonic Obstacle Avoidance/★Ultrasonic Obstacle Avoidance.py"
python3 "4.★Vision Obstacle Avoidance/★Vision Obstacle Avoidance.py"
~~~

**切勿同时启动多个直接控制电机的脚本。** 接通电机前务必对照源码核实 BCM GPIO 引脚及驱动板供电。首次测试建议架空车轮并准备物理断电方式。

视觉脚本当前默认读取：

~~~text
http://192.168.50.1:8080/?action=stream
~~~

需要根据实际摄像头或 MJPG-Streamer 地址修改 `STREAM_URL`。在纯 SSH 无图形环境下还应检查 `SHOW_VIDEO` 相关设置。

### 7.3 YOLO 实验运行注意事项

仓库包含两套预测脚本和对应模型权重，但仍需修正运行环境：

- 将脚本中写死的 Windows `H:\TechProject\...` 模型路径，改为仓库实际模型权重的位置。
- 修改摄像头数据流 URL。
- 预测脚本虽然检测 CUDA 是否可用，但推理调用仍写死 `cuda:0`，纯 CPU 环境应先修正。
- 五类别训练脚本的初始权重与报告描述的 YOLO 版本不完全一致，需先核实权重架构。
- 依据设备、数据集和实际软件版本重新测试，不应直接把报告近似值当作复现实测结果。

当前代码定位是**课程实验原型与可参考实现**，不是可直接部署的商业产品。

## 八、开发迭代与工程经验

39 页报告记录了项目的重要调试过程：

**超声波版本迭代（VER1–VER4）：** 初期同步测距导致运动控制响应变慢；引入多线程后出现控制优先级问题；随后通过多次测距平均减少异常跳变，报告最终记录了更稳定的停车响应。

**视觉转弯版本迭代（v1–v6）：** 最初的大弯判断在直角转弯时不稳定；团队逐步修改转向参数、质心排列判断和状态退出条件，最终形成区分正常 PID 巡线与大转弯执行的算法结构。

从工程角度看，值得继续完善的部分包括：丢线后的真正制动处理、超声波采样超时、线程暂停/恢复机制、统一配置管理、YOLO 版本与依赖锁定、端到端自动化测试。

## 九、STEM / 创客课程转化思路

这套项目能够被拆解为不同学习阶段的工程任务：

| 学习阶段 | 对应模块 | 可交付成果 |
| --- | --- | --- |
| 基础 | GPIO、PWM、红外传感器 | 小车控制和黑线检测 |
| 进阶一 | P 比例控制、循迹状态识别 | 稳定红外循迹 |
| 进阶二 | OpenCV、图像二值化、质心 | 摄像头识别路径 |
| 进阶三 | PID、弯道状态机 | 视觉循迹和参数调试记录 |
| 综合实践 | 超声波、线程锁、停车逻辑 | 带障碍检测的机器人 |
| AI 拓展 | Roboflow、YOLO、可视化 | 自定义检测模型及演示视频 |

可以按 **需求定义 → BOM 与接线 → 单模块实现 → 功能测试 → 参数调试 → 集成演示 → 实验报告与答辩** 的完整工程周期组织教学。

**说明：** 本节是从现有项目衍生的潜在课程设计方案，不代表团队已实际完成青少年课堂授课或竞赛辅导。

## 十、项目成员与文档

**ESIGELEC · Technical Project 2025 · Group G2 · Team 01**

- Zhao Hangjian（赵杭键）
- Zhou Hangyu
- Jia Zihan

本项目为**团队合作完成**。公开报告没有明确列出每一模块的个人独立贡献，因此不将全部软硬件工作归为单人完成。

**项目资料：** [英文 README](./README.md) · [完整 PDF 技术报告](./technical_project_2025_G2_T01.pdf) · [项目甘特图](./schedule/schedule.png)

> 本项目涉及真实电机与机器人硬件，运行前务必检查接线与安全措施；报告中的性能数据均为 2025 年技术报告记录值，本次 README 更新未重新训练或测试模型。