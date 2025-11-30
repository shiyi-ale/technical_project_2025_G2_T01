#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

# --- 引脚定义 ---
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13
key = 8
TrackSensorLeftPin1  = 3
TrackSensorLeftPin2  = 5
TrackSensorRightPin1 = 4
TrackSensorRightPin2 = 18

# --- GPIO 设置 ---
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# --- 全局变量 ---
pwm_ENA = None
pwm_ENB = None

# --- 硬件初始化 ---
def init():
    global pwm_ENA, pwm_ENB
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(key, GPIO.IN)
    GPIO.setup(TrackSensorLeftPin1, GPIO.IN)
    GPIO.setup(TrackSensorLeftPin2, GPIO.IN)
    GPIO.setup(TrackSensorRightPin1, GPIO.IN)
    GPIO.setup(TrackSensorRightPin2, GPIO.IN)
    
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)

# --- 运动函数 (保持不变) ---
def run(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

def back(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

def left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

def right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

def spin_left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

def spin_right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

def brake():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def key_scan():
    while GPIO.input(key):
        pass
    while not GPIO.input(key):
        time.sleep(0.01)
        if not GPIO.input(key):
            time.sleep(0.01)
            while not GPIO.input(key):
                pass

# --- 主程序 ---
try:
    init()
    key_scan()

    # --- PID 控制参数 (简化版) ---
    # 基础速度
    base_speed = 25
    # 比例常数 Kp，这是最关键的调节参数。
    # 如果小车摇摆太厉害，减小它；如果转弯不及时，增大它。
    Kp = 15
    # 记录上一次的误差，用于处理“丢失路线”的情况
    last_error = 0

    while True:
        # 读取传感器值 (0代表黑线, 1代表白底)
        sensors = [
            GPIO.input(TrackSensorLeftPin1),
            GPIO.input(TrackSensorLeftPin2),
            GPIO.input(TrackSensorRightPin1),
            GPIO.input(TrackSensorRightPin2)
        ]

        # --- 特殊情况处理 (如十字路口或急转弯) ---
        # 全黑，可能是十字路口或终点
        if sensors == [0, 0, 0, 0]:
            run(base_speed, base_speed) # 默认直行通过
            time.sleep(0.1) # 穿越一小段距离
            continue # 跳过本次循环的PID计算

        # 左侧两个传感器压线，左急转弯
        elif sensors[0] == 0 and sensors[1] == 0:
            spin_left(40, base_speed + 15) # 右轮加速，左轮停止，实现快速流畅左转
            last_error = -4 # 记录一个大的左偏误差
            continue

        # 右侧两个传感器压线，右急转弯
        elif sensors[2] == 0 and sensors[3] == 0:
            spin_right(base_speed + 15, 40) # 左轮加速，右轮停止，实现快速流畅右转
            last_error = 4 # 记录一个大的右偏误差
            continue

        # --- 比例控制计算 ---
        # 根据传感器状态计算位置误差
        # 我们为传感器分配权重: [-3, -1, 1, 3]
        error = 0
        # 仅当传感器在白底上(值为1)时，我们不计算它。在黑线上(值为0)时才计算。
        # 为了简化，我们只用内侧两个传感器做比例控制，外侧的用于急转弯判断
        if sensors[1] == 0 and sensors[2] == 1: # 车身偏右
            error = -1 
        elif sensors[1] == 1 and sensors[2] == 0: # 车身偏左
            error = 1
        elif sensors[1] == 0 and sensors[2] == 0: # 在线中间
            error = 0
        # 如果内侧传感器都在白底上，则使用外侧传感器
        elif sensors[0] == 0: # 严重偏右
            error = -2
        elif sensors[3] == 0: # 严重偏左
            error = 2
        # 全白，丢失路线
        elif sum(sensors) == 4:
            # 如果丢失路线，根据上一次的误差方向转弯寻找
            if last_error > 0: # 上次是向右偏
                spin_right(base_speed, 30) # 向右转寻找
            else: # 上次是向左偏
                spin_left(30, base_speed) # 向左转寻找
            continue # 跳过PID计算
        
        # 记录当前误差，以便“丢失路线”时使用
        if error != 0:
            last_error = error

        # --- 速度计算 ---
        # 核心公式：校正量 = Kp * 误差
        correction = Kp * error

        left_speed = base_speed + correction
        right_speed = base_speed - correction

        # 限制速度范围，防止速度过高或变为负数
        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))

        # 执行动作
        run(left_speed, right_speed)
        
except KeyboardInterrupt:
    print("程序中断")
finally:
    if pwm_ENA:
        pwm_ENA.stop()
    if pwm_ENB:
        pwm_ENB.stop()
    GPIO.cleanup()

