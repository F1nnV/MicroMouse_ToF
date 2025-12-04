#Motors.py
from machine import Pin, PWM #65535 max value
import encoders
from vl6180x import read_right, read_left, read_middle, setup_sensors
import math
import time


#Motors
AIN1 = Pin(2, Pin.OUT)
AIN2 = Pin(3, Pin.OUT)
BIN1 = Pin(4, Pin.OUT)
BIN2 = Pin(5, Pin.OUT)

#Motor PWM
PWMA = PWM(Pin(0), freq = 20000)
PWMB = PWM(Pin(1), freq = 20000)

cell_mm =180
ticks_per_mm = 2
dt_loop_length = 0.01 #10ms loop for PD controller

def motor_speed_a(speed): # -100 to 100
    if speed > 0:
        AIN1.high() #dir set forward
        AIN2.low()
        PWMA.duty_u16(int(speed / 100 * 65535)) #convert to PWM
    elif speed < 0:
        AIN1.low() #dir set rev
        AIN2.high()
        PWMA.duty_u16(int(-speed / 100 * 65535)) #convert to PWM
        
    else:
        AIN1.low()
        AIN2.low()
        PWMA.duty_u16(0)
        print("Motor A set to stationary")
 
def motor_speed_b(speed): # -100 to 100 
    if speed > 0:
        BIN1.high() #dir set forward
        BIN2.low()
        PWMB.duty_u16(int(speed / 100 * 65535)) #convert to PWM
    elif speed < 0:
        BIN1.low() #dir set rev
        BIN2.high()
        PWMB.duty_u16(int(-speed / 100 * 65535)) #convert to PWM
        
    else:
        BIN1.low()
        BIN2.low()
        PWMB.duty_u16(0)
        print("Motor B set to stationary")
        
def motorstop(): #stop all motion
    BIN1.low()
    BIN2.low()
    AIN1.low()
    AIN2.low()
    PWMA.duty_u16(0)
    PWMB.duty_u16(0)




def map_turn_by(degrees):
    times = 0.50
    
    if degrees == 90:
        motorstop()
        motor_speed_a(-10)
        motor_speed_b(10)
        time.sleep(times)
        motorstop()
    elif degrees == -90:
        motorstop()
        motor_speed_a(10)
        motor_speed_b(-10)
        time.sleep(times)
        motorstop()

def right_wall_follow(sensor_map):
    prev_error_alignment = 0
    drive_scale = 1
    kp_alignment = 0.10
    kd_alignment = 0
    speed = 8
    sleep_time = 0
    prev_time = time.ticks_ms()
    side_wall_value_threshold = 60
    wall_target = 42  
    boot = 0
    
    min_speed = 3
    max_speed = 40

    while True:
    
        now = time.ticks_ms()
        dt = (time.ticks_diff(now, prev_time)) / 1000.0
        if dt <= 0:
              dt = 0.001
        map_speed = speed

        left_value = read_left(sensor_map)
        print("l",left_value)
    
        right_value = read_right(sensor_map)
        print("r",right_value)

        left_wall_present = left_value < side_wall_value_threshold
        right_wall_present = right_value < side_wall_value_threshold
        if left_wall_present and right_wall_present:
            error_alignment = right_value - left_value
        elif left_wall_present and not right_wall_present:
            error_alignment = left_value - wall_target
        elif right_wall_present and not left_wall_present:
            error_alignment = wall_target - right_value
        else: #no side walls present
            error_alignment = 0 

        #pd calcs
        alignment_derivative = (error_alignment - prev_error_alignment) / dt
        u_alignment = (kp_alignment * error_alignment) + (kd_alignment * alignment_derivative)
        
        #motor commands
        left_command = map_speed + u_alignment
        right_command = map_speed - u_alignment

        motor_speed_a(left_command)
        motor_speed_b(right_command)

        front_value = read_middle(sensor_map)

        if front_value <= 80:
            speed = 5
            motor_speed_a(5)
            motor_speed_b(5)

            if (front_value <= 45) and (left_value <= 55):
                motorstop()
                time.sleep(0.2)
                map_turn_by(90)
                motorstop()
                time.sleep(1)
                boot = 0
                speed = 8
                front_value = 100
                left_value = 100
                right_value = 100
                motor_speed_a(8)
                motor_speed_b(8)
            elif(front_value <= 45) and (right_value <= 55):
                motorstop()
                time.sleep(0.2)
                map_turn_by(-90)
                motorstop()
                time.sleep(1)
                boot = 0
                speed = 8
                front_value = 100
                left_value = 100
                right_value = 100
                
            elif (front_value <= 45) and (left_value >= 100) and (right_value >= 100):
                map_turn_by(90)
                speed = 8
                boot = 0
            else: 
                speed = 8
                motor_speed_a(8)
                motor_speed_b(8)
                boot = 0
        elif right_value > 180:
                boot = boot + 1
                if right_value <= 79:
                    boot = 0 
                elif (boot > 18) and (front_value >= 180):
                    motorstop()
                    time.sleep(0.2)
                    map_turn_by(90)
                    boot = 0
                    time.sleep(1) 
 

            #add solid wall counter too 

        if front_value < 18:
            motorstop()
            time.sleep(0.2)
            motor_speed_a(-8)
            motor_speed_b(-8)
            time.sleep(1)
            motorstop()
        
        prev_error_alignment = error_alignment
        prev_time = now
        time.sleep_ms(sleep_time)
