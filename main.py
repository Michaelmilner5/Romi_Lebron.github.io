from pyb import Pin, Timer, I2C
import gc
from task_share import Share, Queue, show_all
gc.collect()
from cotask import Task, task_list
from task_garbage import garbage
gc.collect()
from task_course import task_course   # ← moved up
gc.collect()
from motor_driver import motor_driver
from encoder import encoder
gc.collect()
from task_motor import task_motor
gc.collect()
from line_sensor import LineSensorArray
gc.collect()
from task_line import LineFollowTask
gc.collect()
from driver import BNO055
gc.collect()
from task_imu import task_imu
gc.collect()
from task_bump import task_bump
gc.collect()
from task_start_button import task_start_button
gc.collect()
from task_user_cmds import UserCmds
gc.collect()
from task_user import task_user
gc.collect()
pwm_tim = Timer(3, freq=20000)
leftMotor  = motor_driver(Pin.cpu.B1, Pin.cpu.B15, Pin.cpu.B14, pwm_tim, 4)
rightMotor = motor_driver(Pin.cpu.B0, Pin.cpu.B5,  Pin.cpu.B4,  pwm_tim, 3)
tim_enc_left  = Timer(1, period=0xFFFF, prescaler=0)
tim_enc_right = Timer(2, period=0xFFFF, prescaler=0)
leftEncoder  = encoder(tim_enc_left,  Pin.cpu.A8, Pin.cpu.A9)
rightEncoder = encoder(tim_enc_right, Pin.cpu.A0, Pin.cpu.A1)
adc_pins = [
    Pin.cpu.C0, Pin.cpu.C1, Pin.cpu.A5, Pin.cpu.A6,
    Pin.cpu.A7, Pin.cpu.C2, Pin.cpu.C5, Pin.cpu.C4
]
line_sensors = LineSensorArray(
    adc_pins,
    emitter_pin=Pin.cpu.C10,
    weights=[-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5],
    invert=False
)
try:
    line_sensors.load_calib("line_calib.json")
    print("Line calib loaded")
except Exception as e:
    print("No line calib file yet:", e)
i2c = I2C(1, I2C.MASTER, baudrate=400000)
imu = BNO055(i2c)
leftMotorGo  = Share("b", name="Left Mot Go")
rightMotorGo = Share("b", name="Right Mot Go")
velSetpoint_L = Share("f", name="Vel Set L")
velSetpoint_R = Share("f", name="Vel Set R")
vBaseShare      = Share("f", name="LF Base Speed")
lfEnableShare   = Share("b", name="LF Enable")
estStreamEnable = Share("b", name="EST Stream Enable")
courseEnableShare = Share("b", name="Course Enable")
bumpEventShare    = Share("b", name="Bump Event")
KpShare = Share("f", name="Kp")
KiShare = Share("f", name="Ki")
imuCalReq = Share("b", name="IMU Cal Request")
uL_volts = Share("f", name="uL volts")
uR_volts = Share("f", name="uR volts")
sL_meas     = Share("f", name="sL meas")
sR_meas     = Share("f", name="sR meas")
psi_meas    = Share("f", name="psi meas")
psiDot_meas = Share("f", name="psiDot meas")
s_hat   = Share("f", name="s_hat")
psi_hat = Share("f", name="psi_hat")
wL_hat  = Share("f", name="wL_hat")
wR_hat  = Share("f", name="wR_hat")
KpShare.put(0.06)
KiShare.put(0.6)
vBaseShare.put(220.0)
velSetpoint_L.put(0.0)
velSetpoint_R.put(0.0)
lfEnableShare.put(False)
estStreamEnable.put(False)
courseEnableShare.put(False)
bumpEventShare.put(False)
leftMotorGo.put(False)
rightMotorGo.put(False)
uL_volts.put(0.0)
uR_volts.put(0.0)
sL_meas.put(0.0)
sR_meas.put(0.0)
psi_meas.put(0.0)
psiDot_meas.put(0.0)
s_hat.put(0.0)
psi_hat.put(0.0)
wL_hat.put(0.0)
wR_hat.put(0.0)
imuCalReq.put(False)
leftMotorTask = task_motor(
    leftMotor, leftEncoder, leftMotorGo,
    velSetpoint_L, KpShare, KiShare,
    None, None, None,
    u_share=uL_volts, s_share=sL_meas, Vbatt=7.2
)
rightMotorTask = task_motor(
    rightMotor, rightEncoder, rightMotorGo,
    velSetpoint_R, KpShare, KiShare,
    None, None, None,
    u_share=uR_volts, s_share=sR_meas, Vbatt=7.2
)
imuTask = task_imu(
    imu,
    psi_meas,
    psiDot_meas,
    cal_req=imuCalReq
)









userTask = task_user(
    leftMotorGo, rightMotorGo,
    None, None, None,
    None, None, None,
    KpShare, KiShare,
    vBaseShare, lfEnableShare,
    estStreamEnable,
    line_sensors,
    imu=imu,
    psi_meas=psi_meas, psiDot_meas=psiDot_meas,
    sL_meas=sL_meas, sR_meas=sR_meas,
    s_hat=s_hat, psi_hat=psi_hat, wL_hat=wL_hat, wR_hat=wR_hat,
    imuCalReq=imuCalReq,
    courseEnable=courseEnableShare,
    bumpEvent=bumpEventShare,
    velSetpoint_L=velSetpoint_L,
    velSetpoint_R=velSetpoint_R
)
lineFollowTask = LineFollowTask(
    line_sensors,
    vBaseShare,
    velSetpoint_L,
    velSetpoint_R,
    lfEnableShare,
    Kp=60.0, Ki=0.0, Kd=8.0,
    u_max=220.0,
    max_min=0.08, sum_min=0.20,
    oversample=5,
    den_min=10,
    pos_alpha=0.25,
    pos_deadband=0.03,
    stream_hz=0
)
bumpTask = task_bump(
    bumpEventShare
)
startBtnTask = task_start_button(
    courseEnableShare, bumpEventShare=bumpEventShare
)
courseTask = task_course(
    line_sensors,
    courseEnableShare,
    bumpEventShare,
    leftMotorGo,
    rightMotorGo,
    lfEnableShare,
    estStreamEnable,
    velSetpoint_L,
    velSetpoint_R,
    vBaseShare,
    psi_hat,
    sL_meas,
    sR_meas,
    KpShare,
    KiShare
)
task_list.append(Task(leftMotorTask.run,  name="Left Mot. Task",  priority=3, period=5,  profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task", priority=3, period=5,  profile=True))
task_list.append(Task(lineFollowTask.run, name="Line Follow Task", priority=2, period=10, profile=False))
task_list.append(Task(userTask.run,       name="User Int. Task",   priority=2, period=20, profile=False))
task_list.append(Task(imuTask.run,        name="IMU Task",         priority=1, period=20, profile=True))
task_list.append(Task(courseTask.run,     name="Course Task",      priority=4, period=20, profile=False))
task_list.append(Task(bumpTask.run,       name="Bump Task",        priority=5, period=5,  profile=False))
task_list.append(Task(startBtnTask.run,   name="Start Button",     priority=3, period=20, profile=False))
task_list.append(Task(garbage,            name="Garbage",          priority=0, period=100, profile=False))
while True:
    try:
        task_list.rr_sched()
    except KeyboardInterrupt:
        try:
            leftMotor.disable()
        except Exception:
            pass
        try:
            rightMotor.disable()
        except Exception:
            pass
        break
print(task_list)
print(show_all())