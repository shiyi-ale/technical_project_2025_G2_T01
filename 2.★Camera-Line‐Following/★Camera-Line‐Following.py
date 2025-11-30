# -*- coding:UTF-8 -*-
# 会识别不到直角弯，并且错把钝角弯识别成大弯

# =============================================================================
# --- 1. 导入所需库 ---
# =============================================================================
import RPi.GPIO as GPIO
import cv2
import time
import numpy as np

# =============================================================================
# --- 2. 硬件定义与控制函数 (源自您的代码) ---
# =============================================================================
# --- 引脚定义 ---
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13
# (摄像头巡线不再需要按键和红外传感器，但保留定义以备将来使用)
key = 8
TrackSensorLeftPin1  = 3
TrackSensorLeftPin2  = 5
TrackSensorRightPin1 = 4
TrackSensorRightPin2 = 18

# --- GPIO 设置 ---
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# --- 全局 PWM 变量 ---
pwm_ENA = None
pwm_ENB = None

# --- 硬件初始化 ---
def init_hardware():
    global pwm_ENA, pwm_ENB
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    
    pwm_ENA = GPIO.PWM(ENA, 1000)
    pwm_ENB = GPIO.PWM(ENB, 1000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    print("硬件初始化完成。")

# --- 核心运动函数 ---
def run(leftspeed, rightspeed):
    """前进，可通过速度差进行微调转弯"""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

def spin_left(leftspeed, rightspeed):
    """原地左转"""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

def spin_right(leftspeed, rightspeed):
    """原地右转"""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

def brake():
    """停止/刹车"""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(0)
    pwm_ENB.ChangeDutyCycle(0)


# =============================================================================
# --- 3. 摄像头巡线逻辑与参数 ---
# =============================================================================
# -- 视频流与显示 --
STREAM_URL = 'http://192.168.50.1:8080/?action=stream'
FRAME_WIDTH = 64
FRAME_HEIGHT = 48
SHOW_VIDEO = True  # 在树莓派桌面运行时设为True，如果用SSH则设为False

# -- 图像处理 --
NUM_STRIPS = 6

# -- PID 控制器参数 --
KP = 1.2  # 比例系数，在64x48分辨率下，偏移量较小，可能需要稍大的Kp
KI = 0.02 # 积分系数
KD = 0.01  # 微分系数
K_turn = 1.5

# -- 车辆控制参数 --
BASE_SPEED = 10                 # PID巡线时的基础速度 (0-100)
BIG_TURN_SPEED = 45            # 执行大转弯时的速度
CENTER_TOLERANCE = 2            # 中心容忍度（像素）
STABLE_FRAMES_REQUIREMENT = 4   # 退出大弯需要连续稳定在中心的帧数

# --- PID 控制器类 ---
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.last_error, self.integral = 0, 0
        self.last_time = time.time()

    def update(self, current_value):
        dt = time.time() - self.last_time
        if dt == 0: return 0
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        self.last_time = time.time()
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative
    
    def reset(self):
        self.last_error, self.integral = 0, 0

# --- 图像分析函数 ---
def process_image_for_line(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary_image = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    return binary_image

def analyze_line_and_get_state(binary_image):
    """
    改进的线条分析函数，参考第一个程序的完整逻辑
    """
    h, w = binary_image.shape
    strip_height = h // NUM_STRIPS
    midpoint = w // 2
    centroids = []   # 每条带的黑线质心x（相对于整帧）或None
    line_none = 0
    max_local_ang = 0.0
    dx_ratio = 0.0

    # 分条带检测质心
    for n in range(NUM_STRIPS):
        y0 = n * strip_height
        y1 = y0 + strip_height
        strip_bin = binary_image[y0:y1, :]
        # 统计白色像素（黑线在二值图中为白255）的位置质心
        ys, xs = np.where(strip_bin == 255)
        if xs.size == 0:
            centroids.append(None)
            line_none += 1
        else:
            cx = int(xs.mean())  # 条带中线质心x（相对条带）
            # 转为整帧坐标
            centroids.append((cx, y0 + (y1 - y0)//2))

    # 获取有效质心并按y从近到远排序（底部到顶部）
    valid_centroids = [c for c in centroids if c is not None]
    valid_centroids.sort(key=lambda p: p[1], reverse=True)

    # ====== 方向与弯度判定（完全参考第一个程序） ======
    turn_direction = 'STRAIGHT'
    turn_size = 'STRAIGHT'
    
    if len(valid_centroids) >= 3:
        # 使用滑动窗口计算局部夹角（连续三点），取最大局部角度以识别极大转弯
        def angle_between(a, b):
            na = a / (np.linalg.norm(a) + 1e-6)
            nb = b / (np.linalg.norm(b) + 1e-6)
            return np.arccos(np.clip(np.dot(na, nb), -1.0, 1.0))

        local_angles = []
        for i in range(len(valid_centroids) - 2):
            pa = np.array(valid_centroids[i])
            pb = np.array(valid_centroids[i+1])
            pc = np.array(valid_centroids[i+2])
            va = pb - pa
            vb = pc - pb
            local_ang = angle_between(va.astype(float), vb.astype(float))
            local_angles.append(local_ang)

        max_local_ang = max(local_angles) if local_angles else 0.0

        # 近端到远端的总横向偏移（稳定判断左右）
        p0 = np.array(valid_centroids[0])
        p_last = np.array(valid_centroids[-1])
        dx_delta = float(p_last[0] - p0[0])
        if abs(dx_delta) > 5:
            turn_direction = 'RIGHT' if dx_delta > 0 else 'LEFT'

        # 使用局部最大角度和位移比来判断大弯
        dx_ratio = abs(dx_delta) / float(w + 1e-6)
        # 阈值：若某段局部角度非常大（例如 >35°）则视为极大弯；或位移比极大时也判为大弯
        if max_local_ang > np.deg2rad(35) or dx_ratio > 0.5:
            turn_size = 'BIG'
        elif max_local_ang > np.deg2rad(8) or dx_ratio > 0.15:
            turn_size = 'SMALL'
        else:
            turn_size = 'STRAIGHT'

    # 丢线判定（若绝大多数条带无质心则认为丢线）
    if line_none < (NUM_STRIPS - 1):
        lost = False
    else:
        lost = True

    # 底部偏移计算（用于PID控制）
    bottom_offset = 0
    # 从下往上找最近的有效质心（靠近机器人/底部）
    offsets = []
    for n in range(NUM_STRIPS-1, -1, -1):
        if centroids[n] is not None:
            cx = centroids[n][0]
            offset = cx - midpoint
            offsets.append(offset)
            if bottom_offset == 0:  # 取最近的（底部的）偏移
                bottom_offset = offset
    
    # 若有多条有效质心，也可以取平均作为更稳定的偏移值
    if offsets:
        avg_offset = float(np.mean(offsets))
    else:
        avg_offset = 0

    return {
        "centroids": centroids,
        "turn_size": turn_size,
        "turn_direction": turn_direction,
        "bottom_offset": bottom_offset,
        "avg_offset": avg_offset,  # 新增：平均偏移值，可选用于更稳定的PID控制
        "lost": lost,
        "max_local_ang": max_local_ang,
        "dx_ratio": dx_ratio
    }

# =============================================================================
# --- 6. 主程序 ---
# =============================================================================
def main():
    # 初始化摄像头
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        print(f"错误：无法打开视频流: {STREAM_URL}"); return

    # 初始化PID控制器和状态机
    pid_controller = PIDController(KP, KI, KD)
    current_mode = 'PID_FOLLOW'
    stable_center_counter = 0

    # --- 新增的状态变量 ---
    # 用于在自适应转弯模式下跟踪状态
    last_bottom_centroid_x = None # 上一帧底部质心的x坐标
    last_motor_command = (0, 0)   # 上一次的电机指令 (left_speed, right_speed)
    turn_context = None           # 记录当前是在进行'LEFT'还是'RIGHT'的大转弯
    
    print("程序启动... 按 'q' 键退出。")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法接收帧，结束循环。"); break

        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        binary_image = process_image_for_line(frame)
        state = analyze_line_and_get_state(binary_image)
        if current_mode == 'BIG_TURN_EXECUTE':
            
            if turn_context == 'RIGHT':
                # 检查退出条件：从下往上各条带的质心依次从左往右
                centroids = state['centroids']
                centroid_xs = [(i, c[0]) for i, c in enumerate(centroids) if c is not None]
                exit_big_turn = False
                if len(centroid_xs) >= 3:
                    centroid_xs.sort(key=lambda x: -x[0])
                    xs = [x for i, x in centroid_xs]
                    if all(xs[i] < xs[i+1] for i in range(len(xs)-1)):
                        exit_big_turn = True
                if exit_big_turn:
                    print("右转退出条件满足，停止spin_right，切回PID_FOLLOW")
                    current_mode = 'PID_FOLLOW'
                else:
                    if state["max_local_ang"] > np.deg2rad(65) or state["dx_ratio"] > 0.5:
                        spin_right(50, 10)
                    else: spin_right(50, 20)
                    continue
            else: # turn_context == 'LEFT'
                # 检查退出条件：从下往上各条带的质心依次从右往左
                centroids = state['centroids']
                centroid_xs = [(i, c[0]) for i, c in enumerate(centroids) if c is not None]
                exit_big_turn = False
                if len(centroid_xs) >= 3:
                    centroid_xs.sort(key=lambda x: -x[0])
                    xs = [x for i, x in centroid_xs]
                    if all(xs[i] > xs[i+1] for i in range(len(xs)-1)):
                        exit_big_turn = True
                if exit_big_turn:
                    print("左转退出条件满足，停止spin_left，切回PID_FOLLOW")
                    current_mode = 'PID_FOLLOW'
                else:
                    if state["max_local_ang"] > np.deg2rad(65) or state["dx_ratio"] > 0.5:
                        spin_left(10, 50)
                    else: spin_left(20, 50)
                    continue
        # 安全检查：如果丢线，则停车
        if state['lost']:
            # brake()
            # 重置状态，以便找到线后能重新开始
            # current_mode = 'PID_FOLLOW'
            # pid_controller.reset()
            continue
        
        bottom_offset = state['bottom_offset']
        
        # 从最底部两个条带找到一个有效的质心用于自适应转弯
        # (从下往上找，找到第一个就用)
        current_bottom_centroid_x = None
        for i in range(NUM_STRIPS - 1, max(0, NUM_STRIPS - 3), -1):
             if state['centroids'][i] is not None:
                 current_bottom_centroid_x = state['centroids'][i][0]
                 break

        # --- 核心状态机逻辑 ---
        if current_mode == 'PID_FOLLOW':
            if state['turn_size'] == 'BIG':
                # --- 切换到自适应转弯模式 ---
                current_mode = 'BIG_TURN_EXECUTE'
                turn_context = state['turn_direction'] # 记录是左转还是右转
                last_bottom_centroid_x = current_bottom_centroid_x # 初始化上一帧位置
                print(f"!! STATE CHANGE: BIG {turn_context} TURN DETECTED -> EXECUTING...")
            else:
                # 正常的PID巡线
                correction = pid_controller.update(-bottom_offset)
                left_speed = BASE_SPEED + correction
                right_speed = BASE_SPEED - correction
                left_speed = max(0, min(100, left_speed))
                right_speed = max(0, min(100, right_speed))
                run(left_speed, right_speed)
                last_motor_command = (left_speed, right_speed)

        

        
        # ... (可视化调试部分保持不变) ...

if __name__ == '__main__':
    try:
        init_hardware()
        run(10, 10)
        time.sleep(1)
        main()
    except KeyboardInterrupt:
        print("程序中断")
    finally:
        print("\n--- 执行最终清理 ---")
        brake()
        GPIO.cleanup()
        cv2.destroyAllWindows()
        print("资源已释放，程序安全退出。")