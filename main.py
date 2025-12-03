from machine import Pin
import time
from motors import right_wall_follow, motor_speed_a, motor_speed_b, motorstop
from vl6180x import (
    setup_sensors,
    read_left,
    read_middle,
    read_right,
)

#LEDs
indicator_led_1 = Pin(12, Pin.OUT)
indicator_led_2 = Pin(13, Pin.OUT)

#Buttons
button_1 = Pin(10, Pin.IN)
button_2 = Pin(11, Pin.IN)

#Startup Test
print("Micromouse Active")
indicator_led_1.on()
indicator_led_2.on()
time.sleep(0.5)
indicator_led_1.off()
indicator_led_2.off()

print("buttons ready for testing")
button_1.value()
button_2.value()



sensor_map = setup_sensors()




while True:
    
    if button_1.value() == 0:
        print("R", read_right(sensor_map), "L", read_left(sensor_map), "M", read_middle(sensor_map))

    

    if button_2.value() == 0:
        right_wall_follow(sensor_map)