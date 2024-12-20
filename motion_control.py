from machine import PWM, Pin, Timer
import utime
import math


class motor:
    
    def __init__(self, pin_1=7, pin_2=8, pin_pwm=6, pin_pwm_in=21):
        
        self.pin_1 = pin_1
        self.pin_2 = pin_2
        self.pin_pwm = pin_pwm
        self.pin_pwm_in = pin_pwm_in
        
        self.pin1 = Pin(pin_1,Pin.OUT)
        self.pin2 = Pin(pin_2,Pin.OUT)
                
        self.pwm = PWM(Pin(pin_pwm,Pin.OUT), freq = 1000)

#         self.Kp = 30
#         self.Ki = 100
#         self.Kd = 10

        self.Kp = 30
        self.Ki = 50
        self.Kd = 20

        self.pid = PID(self.Kp, self.Ki, self.Kd, 0)
                
        # stop the motor
        self.pin1.value(0)
        self.pin2.value(0)
    
    def forward(self, duty):
        self.pwm.init()
        self.pwm.duty_u16(int(max(0,min(65535,duty))))  # 避免超出调节区间
        self.pin1.value(1)
        self.pin2.value(0)
        
    def backward(self, duty):
        self.pwm.init()
        self.pwm.duty_u16(int(max(0,min(65535,duty))))  # 避免超出调节区间
        self.pin1.value(0)
        self.pin2.value(1)
        
    def stop(self):
        self.pin1.value(0)
        self.pin2.value(0)
        self.pwm.deinit()


class Servo:
    def __init__(self, pin):
        # Set the default angle
        self.default_ang=89
        self.pwm = PWM(pin)
        duty_ns = 500000 + int((self.default_ang / 180) * 2000000)
        self.pwm.init(freq=50, duty_ns=duty_ns)


    # 先设置default angle，再启动.默认89
    def set_default_angle(self,default_ang=89):
        self.pwm.deinit()
        self.default_ang=default_ang
        duty_ns = 500000 + int((self.default_ang / 180) * 2000000)
        self.pwm.init(freq=50, duty_ns=duty_ns)


    def set_angle(self, angle):
        self.pwm.deinit()
        """
        设置舵机角度
        :param angle: -90~90° 的角度
        """
        # 将角度转换为占空比 (1ms~2ms)
        ang = self.default_ang - angle
        if ang > 180:
            ang=180
        elif ang < 0:
            ang=0

        duty_ns = 500000 + int((ang / 180) * 2000000)
        self.pwm.init(freq=50, duty_ns=duty_ns)
        

    def servo_deinit(self):
        self.pwm.deinit()



class wheel:
    '''
    控制servo指定角度转弯
    '''
    def __init__(self, pwm_pin):
        # Servo object
        self.servo=Servo(pwm_pin)
        self.servo.set_default_angle()

    def wheel_particular_ang(self,ang):
        '''
        @notice: 还要注意一下角度正负号的问题
        '''
        self.servo.set_angle(-1*ang)
        
    def wheel_deinit(self):
        self.servo.servo_deinit()

    def set_servo_straight(self):
        self.servo.set_default_angle()




class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def set_target(self,target):
        self.setpoint = target

    def set_param(self,kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute(self, measurement):
        error = self.setpoint - measurement
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        result = self.kp * error + self.ki * self.integral + self.kd * derivative
#         print(result)
        return result
    
    def compute_2(self, error):
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        result = self.kp * error + self.ki * self.integral + self.kd * derivative
#         print(result)
        return result

class counter:
    def __init__(self, pin_num, interval=50):
        self.pin = Pin(pin_num,Pin.IN)
        self.interval = interval
        self.timer = Timer()
        self.time_start = utime.ticks_us()
        self.cycle = 0  # 指定时间内周期计数器
        self.freq = 0


    def start_count(self):
        # 计时器闭包
        def partial_callback(timer):
            elapsed_time = utime.ticks_us() - self.time_start
            if elapsed_time > 0:
                self.freq = self.cycle * 1000000 / elapsed_time
                self.cycle = 0
                self.time_start = utime.ticks_us()

        # 计数器闭包
        def partial_cntpulse(p):
            self.cycle += 1

        # 计时器开始
        self.time_start = utime.ticks_us()
        self.timer.init(mode=Timer.PERIODIC, period=self.interval, callback=partial_callback)

        # 计数器开始
        self.pin.irq(trigger=Pin.IRQ_RISING, handler=partial_cntpulse)

    # 终止计时器
    def end_count(self):
        self.timer.deinit()



class line_follower:
    def __init__(self, p1, p2, p3, p4, p5):
        self.pin1=Pin(p1,Pin.IN)
        self.pin2=Pin(p2,Pin.IN)
        self.pin3=Pin(p3,Pin.IN)
        self.pin4=Pin(p4,Pin.IN)
        self.pin5=Pin(p5,Pin.IN, Pin.PULL_UP)
        self.timer = Timer()

    def detect_main(self):
        level1=self.pin1.value()
        level2=self.pin2.value()
        level3=self.pin3.value()
        level4=self.pin4.value()
        level5=self.pin5.value()
        
        if level1==0 or level2==0 or level3==0 or level4==0 or level5==0:
            print('reached')
            return False
        else:
            return True

    def follow_main(self):
        level1=self.pin1.value()
        level2=self.pin2.value()
        level3=self.pin3.value()
        level4=self.pin4.value()
        level5=self.pin5.value()

        if level1==0 and level2==1 and level3==1 and level4==1 and level5==1:
            print('left left') 
            return 1

        if level1==1 and level2==0 and level3==1 and level4==1 and level5==1:
            print('left')
            return 2
        
        if level1==0 and level2==0 and level3==1 and level4==1 and level5==1:
            print('left 1')
            return 12
        
        if level1==0 and level2==0 and level3==0 and level4==1 and level5==1:
            print('left 2')
            return 123

        if level1==1 and level2==1 and level3==0 and level4==1 and level5==1:
            print('forward')
            return 3

        if level1==1 and level2==1 and level3==1 and level4==0 and level5==1:
            print('right')
            return 4

        if level1==1 and level2==1 and level3==1 and level4==1 and level5==0:
            print('right right')
            return 5
        
        if level1==1 and level2==1 and level3==1 and level4==0 and level5==0:
            print('right 1')
            return 45
        
        if level1==1 and level2==1 and level3==0 and level4==0 and level5==0:
            print('right 2')
            return 345

    def start_following(self):
        def callback0(timer):
            self.follow_main()

        # 开始计时器
        self.timer.init(mode=Timer.PERIODIC, period=100, callback=callback0)
    

    def end_following(self):
        self.timer.deinit()









