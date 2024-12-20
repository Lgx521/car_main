from motion_control import motor, PID, counter, wheel, line_follower
from machine import Pin, PWM, Timer, I2C
import utime
from jy61p import jy61p_i2c


## 常数
LEFT = -1
RIGHT = 1

## 所有引脚定义
# I2C输入
OLED_SDA = 0
OLED_SCL = 1
GYRO_SDA = 2
GYRO_SCL = 3

# Motor1 所有控制
PWM_OUT_1 = 6
M1 = 7
M2 = 8
PWM_IN_1 = 21
PWM_IN_2 = 20

# Motor2 所有控制
PWM_OUT_2 = 10
M3 = 12
M4 = 11
PWM_IN_3 = 19
PWM_IN_4 = 18

# Servo PWM 输出
SERVO = 15

# 外部中断
EXT_IRT_START = 16
EXT_IRT_INIT = 22

# 程序状态指示
STATUS_LED = 17

# 红外模块数字输入
D_1 = 26
D_2 = 27
D_3 = 28
D_4 = 4
D_5 = 5


key = Pin(EXT_IRT_START, Pin.IN, Pin.PULL_UP)
def external_interrupt(key):
    # 消除抖动
    utime.sleep_ms(150)
    # 再次判断按键是否被按下
    if key.value() == 0:
        print('The button is pressed')
        return True
    return False



class Car:
    '''
    car类，包含电机，陀螺仪，巡线等所有原件
    '''

    def __init__(self):
        self.gyro = jy61p_i2c()
        self.gyro_pid = PID(15,0,50,0)
        self.gyro.init()

        self.start_time=0
        self.timer_calc=0

        self.m1 = motor(M1, M2, PWM_OUT_1, PWM_IN_1)
        self.m2 = motor(M3, M4, PWM_OUT_2, PWM_IN_3)

        self.timer_dutyset = Timer()
        self.gyro_timer = Timer()

        self.target=0

        self.dir_target=0
        self.dir_inprogress=0
        self.servo_ang=0
        self.gyro_status=0
        self.dir_target=0

        # 实例化转弯类的对象
        self.wheeling = wheel(SERVO)

        # 电机编码器返回频率计算值
        self.freq_1 = 0
        self.freq_2 = 0

        # 计数器（频率计算器）对象
        self.counter1 = counter(PWM_IN_1)
        self.counter2 = counter(PWM_IN_3)
        
        print('sensor start')

    # 计算目标转速对应的频率
    def rpm_to_freq(self,rpm):
        return rpm * 3.095
    # 计算目标频率对应的转速
    def freq_to_rpm(self,freq):
        return freq / 3.095

    # 启动电机   ### 先启动电机，再motor_run()
    def motor_start(self):
        self.start_time = utime.ticks_us()
        self.counter1.start_count()
        self.counter2.start_count()

        ## 电机控制回调函数
        def duty_set(timer):
            # 计算返回的频率
            self.freq_1 = self.counter1.freq
            self.freq_2 = self.counter2.freq

            # 对电机进行pid控制
            if self.target == 0:
                self.m1.stop()
                self.m2.stop()
            elif self.target > 0:
                self.m1.pid.set_target(self.target)
                self.m2.pid.set_target(self.target)
                duty_1 = self.m1.pid.compute(self.freq_1)
                duty_2 = self.m2.pid.compute(self.freq_2)
                self.m1.forward(duty_1)
                self.m2.forward(duty_2)
            else:
                self.m1.pid.set_target(abs(self.target))
                self.m2.pid.set_target(abs(self.target))
                duty_1 = self.m1.pid.compute(self.freq_1)
                duty_2 = self.m2.pid.compute(self.freq_2)
                self.m1.backward(duty_1)
                self.m2.backward(duty_2)
                
            print('m1 freq=%.2f rpm=%.2f;  m2 freq=%.2f rpm=%.2f' % (self.freq_1,self.freq_to_rpm(self.freq_1), self.freq_2,self.freq_to_rpm(self.freq_2)) )
            print('m1&2 target=%.2f'%(self.target))

        # 开始计时
        self.timer_dutyset.init(mode=Timer.PERIODIC, period=50, callback=duty_set)

    # 电机设定函数
    def motor_run(self,rpm):
        self.target = self.rpm_to_freq(rpm)

    # 角度差异计算
    def angle_difference(self, angle1, angle2):
        # 确保角度在 0～360 范围内
        angle1 = angle1 % 360
        angle2 = angle2 % 360

        # 计算直接差值
        diff = angle2 - angle1

        # 确保差值在 -180 到 180 范围
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360

        return diff

    # 陀螺仪控制计时器闭包启动函数
    def gyro_start(self):
        
        # 先初始化陀螺仪
        self.gyro.init()
        # 闭包回调函数
        def gyro_callback(t):
            
            self.dir_inprogress=float(self.gyro.read_ang()[2])
            
            error=self.angle_difference(self.dir_target,self.dir_inprogress)

            # PID计算需要转弯的角速度值
            self.servo_ang = self.gyro_pid.compute_2(error)/5  # 这里的输出是舵机角度，调参，下面准直里也有一个
            
            if abs(error) < 5:
                # 角度差距误差阈值：5 degree 后停止计时器运行
                self.gyro_timer.deinit()
            
            if self.servo_ang > 30:  # 设定最大转弯的角为30度
                self.servo_ang=30
            elif self.servo_ang < -30:
                self.servo_ang = -30
            print('angle=%.2f, tar=%.2f, dirnow=%.2f'%(self.servo_ang, self.dir_target, self.dir_inprogress))
            
            # 操作进行转弯
            self.wheeling.wheel_particular_ang(self.servo_ang)

        self.gyro_timer.init(mode=Timer.PERIODIC, period=50, callback=gyro_callback)

    # 陀螺仪转弯(particular angle)
    def proceed_gyro(self,angle):
        '''
        angle 为一个增量，范围为-180～180，负号表示右转，正号左转
        陀螺仪仅作转弯完成度判断，实际转弯靠PID确定角速度完成
        '''

        # 设定目标方向
        self.dir_now = float(self.gyro.read_ang()[2])
        
        self.dir_target = self.dir_now + angle
        if self.dir_target > 180:
            self.dir_target -= 360
        elif self.dir_target < -180:
            self.dir_target += 360
            
        # pid设置目标
        self.gyro_pid.set_target(self.dir_target)

        # 开始陀螺仪转弯
        self.gyro_start()

    # 终止整个程序进程
    def end_process(self):
        self.motor_run(0)
        self.m1.stop()
        self.m2.stop()
        self.timer_dutyset.deinit()
        self.gyro_timer.deinit()
        self.counter1.end_count()
        self.counter2.end_count()
        Pin(STATUS_LED,Pin.OUT).value(0)

    # 开始整个程序进程
    def start_process(self):
        self.timer_dutyset.init()
        self.gyro_timer.init()
        self.counter1.start_count()
        self.counter2.start_count()
        Pin(STATUS_LED,Pin.OUT).value(1)


    def servo_initialization(self):
        '''
        此方法用于使小车行进8s，小车直线行驶后确定servo的角度值
        '''

        self.motor_run(180)

        self.proceed_gyro(0)

        utime.sleep(8)
            
        default_ang = self.servo_ang

        self.wheeling.set_servo_straight(default_ang)
        print(default_ang)
        print('\n\n')

        self.motor_run(0)

        # 再一次重启
        utime.sleep(1)
        self.end_process()
        utime.sleep(1)
        self.start_process()


###############################################################



class FollowLineClass:


    # 构造方法
    def __init__(self):
        # 用于巡线的计时器对象
        self.line_follow_timer=Timer()
        
    def deinit(self):
        self.line_follow_timer.deinit()

    # 巡线方法，按半圆弧段巡线
    '''
    确认转了180度之后再拐弯，而不是寻线结束后直接拐弯
    '''
    def follow_line_segment(self, direction_of_rotation, after_rotation_angle):
        '''
        @param: direction_of_rotation：相对半圆弧的运动方向：1->逆时针，-1->顺时针
        @param: after_rotation_angle：在完成巡线后转弯的角度

        完成180度的转弯，巡线转弯（巡线，陀螺仪校准）
        大致思路：巡线先跑，然后快转够180的时候换陀螺仪完成180度，之后再选择转弯45度或者直行
        实现1：应该还是需要计时器，回调函数里面进行控制
        %%实现2：最后总的运行区域里死循环，到了之后退出循环
        '''
        linefollower = line_follower(D_1,D_2,D_3,D_4,D_5)

        # 确定当前小车的方向
        initial_direction = float(s.gyro.read_ang()[2])

        # 确定理论转弯180后的方向  注意这里可能顺时针可能逆时针
        target_direction = initial_direction + 180 * direction_of_rotation
    
        # 将结果限制在-180度到180度的范围内
        if target_direction > 180:
            target_direction -= 360
        elif target_direction < -180:
            target_direction += 360
    
        # 巡线开始跑，检测当前的角度
        # 设定一个阈值，角度快到了就停止，然后转弯
        def line_follow_callback(t):
            dir_now = float(s.gyro.read_ang()[2])

            # 目标角度与当前角度差距大于5度，巡线
            if abs(target_direction-dir_now) > 5:
                status=linefollower.follow_main()
                if status == 1:
                    s.wheeling.wheel_particular_ang(30)
                elif status == 2:
                    s.wheeling.wheel_particular_ang(15)
                elif status == 3:
                    s.wheeling.wheel_particular_ang(0)
                elif status == 4:
                    s.wheeling.wheel_particular_ang(-15)
                elif status == 5:
                    s.wheeling.wheel_particular_ang(-30)

            # 目标角度与当前角度小于五度时，转特定的弯角
            else:
                s.wheeling.wheel_particular_ang(after_rotation_angle)


        self.line_follow_timer.init(mode=Timer.PERIODIC, period=50, callback=line_follow_callback)




    














def task1():
    '''
    任务1，空线直行，遇线停止
    '''
    
    s=Car()
    s.motor_start()
    
    linefollower = line_follower(D_1,D_2,D_3,D_4,D_5)

    s.motor_run(180)
    
    task1_timer=Timer()
    
    def callback_task_1(t):
        if linefollower.detect_main() == False:
            print('stop')
            s.motor_run(0)
            s.end_process()
            task1_timer.deinit()
            
    task1_timer.init(mode=Timer.PERIODIC, period=50, callback=callback_task_1)
    



# def task2():
#     '''
#     任务2，直线行驶+巡线
#     '''


    



if __name__ == '__main__':
    task1()
    
    


