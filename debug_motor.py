# debug_motor.py
from pyb import Pin, Timer
from task_share import Share
from task_motor import task_motor
from encoder import encoder
from motor_driver import motor_driver
from utime import sleep_ms

print("Setting up...")

pwm_tim = Timer(3, freq=20000)
leftMotor  = motor_driver(Pin.cpu.B1, Pin.cpu.B15, Pin.cpu.B14, pwm_tim, 4)
tim_enc_left = Timer(1, period=0xFFFF, prescaler=0)
leftEncoder = encoder(tim_enc_left, Pin.cpu.A8, Pin.cpu.A9)

goFlag = Share("b", name="dbg_go")
vRef   = Share("f", name="dbg_vref")
Kp     = Share("f", name="dbg_kp")
Ki     = Share("f", name="dbg_ki")

goFlag.put(False)
vRef.put(200.0)
Kp.put(1.0)
Ki.put(2.0)

mot = task_motor(leftMotor, leftEncoder, goFlag, vRef, Kp, Ki, None, None, None)
gen = mot.run()

next(gen)
print("After init, state=", mot._state)

next(gen)
print("After wait tick (go=False), state=", mot._state)

goFlag.put(True)
next(gen)
print("After wait tick (go=True), state=", mot._state)

print("Running S2_RUN for 30 ticks...")
for i in range(30):
    next(gen)
    v = leftEncoder.get_velocity()
    print("tick={} state={} v={:.1f} vf={:.2f} ei={:.3f}".format(
        i, mot._state, v, mot._v_filt, mot._e_int))
    sleep_ms(5)

leftMotor.set_effort(0)
leftMotor.disable()
print("Done - did motor move?")
