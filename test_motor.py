# test_motor.py
# Direct motor test - bypasses all tasks/shares
# In REPL: import test_motor
# Motors should spin for 2 seconds then stop

from pyb import Pin, Timer
from utime import sleep_ms

print("test_motor: starting")

pwm_tim = Timer(3, freq=20000)

# Left motor: B1=PWM, B15=DIR, B14=nSLP, Timer3 ch4
nSLP_L = Pin(Pin.cpu.B14, Pin.OUT_PP)
DIR_L  = Pin(Pin.cpu.B15, Pin.OUT_PP)
PWM_L  = pwm_tim.channel(4, pin=Pin.cpu.B1, mode=Timer.PWM, pulse_width_percent=0)

# Right motor: B0=PWM, B5=DIR, B4=nSLP, Timer3 ch3
nSLP_R = Pin(Pin.cpu.B4, Pin.OUT_PP)
DIR_R  = Pin(Pin.cpu.B5, Pin.OUT_PP)
PWM_R  = pwm_tim.channel(3, pin=Pin.cpu.B0, mode=Timer.PWM, pulse_width_percent=0)

print("Enabling motors...")
nSLP_L.high()
nSLP_R.high()
sleep_ms(100)

print("Left motor forward 30% for 2s...")
DIR_L.low()
PWM_L.pulse_width_percent(30)
sleep_ms(2000)
PWM_L.pulse_width_percent(0)
print("Left done")

sleep_ms(500)

print("Right motor forward 30% for 2s...")
DIR_R.low()
PWM_R.pulse_width_percent(30)
sleep_ms(2000)
PWM_R.pulse_width_percent(0)
print("Right done")

nSLP_L.low()
nSLP_R.low()
print("test_motor: done")