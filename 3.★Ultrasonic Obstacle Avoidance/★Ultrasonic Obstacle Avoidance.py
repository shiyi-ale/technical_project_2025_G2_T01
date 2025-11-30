#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
import threading

# --- 引脚定义 ---
# 电机
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

# 传感器和按键
key = 8
TrackSensorLeftPin1  = 3
TrackSensorLeftPin2  = 5
TrackSensorRightPin1 = 4
TrackSensorRightPin2 = 18

# 超声波传感器
EchoPin = 0
TrigPin = 1

# --- GPIO设置 ---
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# --- 全局变量 ---
pwm_ENA = None
pwm_ENB = None
obstacle_detected = False  # 障碍物检测标志
running = True  # 程序运行标志
lock = threading.Lock()  # 线程锁，用于保护共享资源

# --- 硬件初始化 ---
def init():
    """初始化所有GPIO引脚用于电机和传感器"""
    global pwm_ENA, pwm_ENB
    # 电机引脚
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    
    # 巡线传感器
    GPIO.setup(key, GPIO.IN)
    GPIO.setup(TrackSensorLeftPin1, GPIO.IN)
    GPIO.setup(TrackSensorLeftPin2, GPIO.IN)
    GPIO.setup(TrackSensorRightPin1, GPIO.IN)
    GPIO.setup(TrackSensorRightPin2, GPIO.IN)
    
    # 超声波传感器引脚
    GPIO.setup(EchoPin, GPIO.IN)
    GPIO.setup(TrigPin, GPIO.OUT)
    
    # PWM设置
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)

# --- 运动函数 ---
def run(leftspeed, rightspeed):
    """前进"""
    with lock:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

def back(leftspeed, rightspeed):
    """后退"""
    with lock:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

def spin_left(leftspeed, rightspeed):
    """左转"""
    with lock:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

def spin_right(leftspeed, rightspeed):
    """右转"""
    with lock:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

def brake():
    """刹车"""
    with lock:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)

def key_scan():
    """等待按键按下以启动程序"""
    while GPIO.input(key):
        pass
    while not GPIO.input(key):
        time.sleep(0.01)
        if not GPIO.input(key):
            time.sleep(0.01)
            while not GPIO.input(key):
                pass

# --- 超声波距离测量函数 ---
def Distance():
    """测量一次距离"""
    GPIO.output(TrigPin, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(TrigPin, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin, GPIO.LOW)

    t3 = time.time()
    while not GPIO.input(EchoPin):
        t4 = time.time()
        if (t4 - t3) > 0.03:
            return -1  # 超时
    
    t1 = time.time()
    while GPIO.input(EchoPin):
        t5 = time.time()
        if (t5 - t1) > 0.03:
            return -1  # 超时

    t2 = time.time()
    distance = ((t2 - t1) * 340 / 2) * 100
    return distance

def Distance_test():
    """执行15次测量并返回有效值的平均值"""
    num = 0
    ultrasonic = []
    while num < 15:
        distance = Distance()
        # 忽略错误测量(-1)或不现实的值
        if distance != -1 and distance < 500:
            ultrasonic.append(distance)
            num += 1
        time.sleep(0.01)

    if len(ultrasonic) > 0:
        distance = sum(ultrasonic) / len(ultrasonic)
        print("平均距离: {:.2f} cm".format(distance))
        return distance
    else:
        return -1 # 没有有效测量

# --- 避障线程函数 ---
def obstacle_avoidance_thread():
    """避障线程：持续监测前方障碍物"""
    global obstacle_detected, running
    
    while running:
        distance = Distance_test()
        # 如果障碍物太近（例如 < 20厘米）或测量失败
        if 0 < distance < 30:
            print("检测到障碍物！停止... 按下键盘q后继续")
            obstacle_detected = True
            brake()
            # 等待用户按下q键
            while running:
                try:
                    # Windows下input会阻塞，只有检测到q才继续
                    user_input = input("请按q后回车继续：")
                    if user_input.strip().lower() == 'q':
                        break
                except Exception:
                    pass
                time.sleep(0.1)
            if running:
                print("继续巡线。")
                obstacle_detected = False
        else:
            obstacle_detected = False
        time.sleep(0.05)  # 避障检测频率

# --- 巡线线程函数 ---
def line_following_thread():
    """巡线线程：执行巡线逻辑"""
    global obstacle_detected, running
    
    # PID控制参数（简化版）
    base_speed = 30  # 稍高的基础速度
    Kp = 15          # 比例常数
    last_error = 0

    while running:
        # 如果检测到障碍物，暂停巡线
        if obstacle_detected:
            time.sleep(0.05)
            # continue
            break

        # 读取传感器值（0 = 黑线，1 = 白色背景）
        sensors = [
            GPIO.input(TrackSensorLeftPin1),
            GPIO.input(TrackSensorLeftPin2),
            GPIO.input(TrackSensorRightPin1),
            GPIO.input(TrackSensorRightPin2)
        ]

        # 特殊情况处理（例如十字路口或急转弯）
        if sensors == [0, 0, 0, 0]: # 全黑，十字路口
            run(base_speed, base_speed)
            time.sleep(0.1)
            continue

        elif sensors[0] == 0 and sensors[1] == 0: # 急左转
            spin_left(40, base_speed + 10)
            last_error = -4
            
            continue

        elif sensors[2] == 0 and sensors[3] == 0: # 急右转
            spin_right(base_speed + 10, 40)
            last_error = 4
            
            continue

        # 比例控制计算
        error = 0
        if sensors[1] == 0 and sensors[2] == 1: # 偏右
            error = -1 
        elif sensors[1] == 1 and sensors[2] == 0: # 偏左
            error = 1
        elif sensors[1] == 0 and sensors[2] == 0: # 在线上居中
            error = 0
        elif sensors[0] == 0: # 远偏右
            error = -2
        elif sensors[3] == 0: # 远偏左
            error = 2
        elif sum(sensors) == 4: # 失去线路
            if last_error > 0:
                spin_right(base_speed, 20)
            else:
                spin_left(20, base_speed)
            
            continue
        
        if error != 0:
            last_error = error

        # 速度计算
        correction = Kp * error
        left_speed = base_speed + correction
        right_speed = base_speed - correction

        # 限制速度在0到100之间
        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))

        run(left_speed, right_speed)
       

# --- 主程序 ---
try:
    init()
    # key_scan()
    time.sleep(1)

    print("启动多线程巡线避障系统...")
    
    # 创建并启动线程
    obstacle_thread = threading.Thread(target=obstacle_avoidance_thread, name="避障线程")
    line_thread = threading.Thread(target=line_following_thread, name="巡线线程")
    
    # 设置为守护线程，主程序结束时自动结束
    obstacle_thread.daemon = True
    line_thread.daemon = True
    
    # 启动线程
    obstacle_thread.start()
    line_thread.start()
    
    # 主线程保持运行，等待子线程
    try:
        while running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
        running = False

except Exception as e:
    print(f"发生错误: {e}")
    running = False
finally:
    print("正在清理资源...")
    running = False
    
    # 等待线程结束
    if 'obstacle_thread' in locals():
        obstacle_thread.join(timeout=1)
    if 'line_thread' in locals():
        line_thread.join(timeout=1)
    
    # 停止PWM和清理GPIO
    if pwm_ENA:
        pwm_ENA.stop()
    if pwm_ENB:
        pwm_ENB.stop()
    GPIO.cleanup()
    print("清理完成")