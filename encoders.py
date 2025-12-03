from machine import Pin, PWM
import time

#Encoders
ENC1A = Pin(6, Pin.IN, Pin.PULL_UP)
ENC1B = Pin(7, Pin.IN, Pin.PULL_UP)
ENC2A = Pin(8, Pin.IN, Pin.PULL_UP)
ENC2B = Pin(9, Pin.IN, Pin.PULL_UP)

#variables to count total ticks 
left_ticks = 0
right_ticks = 0

#counting functions 
def left_counter(Pin): #count motor movement fwd/bck
    global left_ticks
    # Flip direction so forward counts positive
    if ENC1A.value() == ENC1B.value():
        left_ticks -= 1
    else:
        left_ticks += 1

def right_counter(Pin):
    global right_ticks
    # Flip direction so forward counts positive
    if ENC2A.value() == ENC2B.value():
        right_ticks += 1
    else:
        right_ticks -= 1
    

#interrupts to count ticks
ENC1A.irq(trigger = Pin.IRQ_RISING ,handler = left_counter)
ENC2A.irq(trigger = Pin.IRQ_RISING ,handler = right_counter)
