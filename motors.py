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
    """Turn the robot in place by the given degree amount."""
    track_width = 82
    kp_turn = 0.4
    kd_turn = 0.01
    drive_scale_turn = 0.8
    sleep_time_turn = 25
    end_value = 5

    if degrees == 0:
        return

    print("TURN_START degrees:", degrees)

    direction = 1 if degrees > 0 else -1
    degrees_abs = abs(degrees)

    def value_limits(value, low, high):
        if value < low:
            return low
        if value > high:
            return high
        return value

    arc_mm = (degrees_abs / 360) * math.pi * track_width
    target_ticks = arc_mm / ticks_per_mm
    print("TURN_PARAMS arc_mm:", arc_mm, "target_ticks:", target_ticks)
    enc1_turn_start = encoders.left_ticks
    enc2_turn_start = encoders.right_ticks
    prev_error_turn = 0
    prev_time = time.ticks_ms()

    while True:
        now = time.ticks_ms()
        dt = time.ticks_diff(now, prev_time) / 1000.0
        if dt <= 0:
            dt = 0.001

        enc1_turn = encoders.left_ticks
        enc2_turn = encoders.right_ticks
        progress = abs((enc2_turn - enc2_turn_start - (enc1_turn - enc1_turn_start)) / 2)

        error_turn = target_ticks - progress
        turn_derivative = (error_turn - prev_error_turn) / dt
        u_turn = (kp_turn * error_turn) + (kd_turn * turn_derivative)
        left_motor_scale = -direction * value_limits(u_turn * drive_scale_turn, 10, 15)
        right_motor_scale = direction * value_limits(u_turn * drive_scale_turn, 10, 15)

        print(
            "TURN_LOOP enc_left:", enc1_turn,
            "enc_right:", enc2_turn,
            "progress:", progress,
            "error:", error_turn,
            "u_turn:", u_turn,
            "left_cmd:", left_motor_scale,
            "right_cmd:", right_motor_scale,
        )

        motor_speed_a(left_motor_scale)
        motor_speed_b(right_motor_scale)

        prev_error_turn = error_turn

        if error_turn <= end_value:
            print("TURN_END reached target, error:", error_turn, "progress:", progress)
            break

        prev_time = now
        time.sleep_ms(sleep_time_turn)

    motorstop()

def right_wall_follow(sensor_map):
    prev_error_alignment = 0
    drive_scale = 1
    kp_alignment = 0.1
    kd_alignment = 0.000001
    speed = 10
    sleep_time = 0
    prev_time = time.ticks_ms()
    side_wall_value_threshold = 55 
    wall_target = 41  
    boot = 0

    def value_limits(value, low, high):
        if value <= low:
            return low
        if value >= high:
            return high
        return value
    
    min_speed = 5
    max_speed = 40

    while True:
    
        now = time.ticks_ms()
        dt = (time.ticks_diff(now, prev_time)) / 1000.0
        if dt <= 0:
              dt = 0.001
        map_speed = speed

        front_value = read_middle(sensor_map)

        left_value = read_left(sensor_map)
    
        right_value = read_right(sensor_map)

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
        left_command_scaled = left_command * drive_scale
        right_command_scaled = right_command * drive_scale
        left_command_limited = value_limits(left_command_scaled, min_speed, max_speed)
        right_command_limited = value_limits(right_command_scaled, min_speed, max_speed)

        motor_speed_a(left_command_limited)
        motor_speed_b(right_command_limited)



        if (front_value <= 30) and (left_value <= 45):
            motorstop()
            time.sleep(0.2)
            map_turn_by(90)
            time.sleep(0.2)
            boot = 0
        elif(front_value <= 30) and (right_value <= 45):
            motorstop()
            time.sleep(0.2)
            map_turn_by(-90)
            time.sleep(0.2)
            boot = 0   
        else:
            if right_value > 130:
                boot = boot + 1
                if boot > 22:
                    motorstop()
                    time.sleep(0.2)
                    map_turn_by(90)
                    boot = 0
                    time.sleep(0.2)  
            elif right_value <= 80:
                boot = 0  


        prev_error_alignment = error_alignment
        prev_time = now
        time.sleep_ms(sleep_time)
