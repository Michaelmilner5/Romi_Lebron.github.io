import micropython

S0_INIT=micropython.const(0)
S1_IDLE=micropython.const(1)
S2_FAST_LINE=micropython.const(2)
S3_SLOW_LINE=micropython.const(3)
S4_SMALL_RIGHT=micropython.const(4)
S5_STRAIGHT=micropython.const(5)
S6_TURN_RIGHT=micropython.const(6)
S7_STOP=micropython.const(7)
S8_DRIVE_TO_WALL=micropython.const(8)
S9_BACK_UP=micropython.const(9)
S10_TURN_LEFT=micropython.const(10)
S11_FORWARD_AFTER_TURN=micropython.const(11)
S12_RESUME_LINE=micropython.const(12)
S13_FINAL_RIGHT=micropython.const(13)
S14_FINAL_FORWARD=micropython.const(14)
S15_FINAL_LINE=micropython.const(15)
S16_PRE_FINAL_FORWARD=micropython.const(16)
S17_FAST_FINAL_LINE_1=micropython.const(17)
S18_FAST_FINAL_LINE_2=micropython.const(18)
S19_CUP_RIGHT_TURN=micropython.const(19)
S20_CUP_STRAIGHT_1=micropython.const(20)
S21_CUP_LEFT_TURN_1=micropython.const(21)
S22_CUP_STRAIGHT_2=micropython.const(22)
S23_CUP_LEFT_TURN_2=micropython.const(23)
S24_CUP_HIT_FORWARD=micropython.const(24)
S25_POST_CUP_BACK=micropython.const(25)
S26_POST_CUP_LEFT=micropython.const(26)
S27_POST_CUP_LINE=micropython.const(27)
S28_POST_CUP_RIGHT=micropython.const(28)
S29_POST_CUP_FINAL_LINE=micropython.const(29)
S30_END_SMALL_RIGHT=micropython.const(30)
S31_END_STRAIGHT=micropython.const(31)
S111_MID_LEFT=micropython.const(111)
S112_MID_FORWARD=micropython.const(112)
S113_MID_BACKWARD=micropython.const(113)
S114_MID_RIGHT=micropython.const(114)

FAST_KP=0.06
FAST_KI=0.35
FAST_SPEED=2500.0

SLOW_KP=0.04
SLOW_KI=0.4
SLOW_SPEED=500.0

FAST_LINE_TRAVEL=1.4
SLOW_LINE_TRAVEL=0.23

SMALL_RIGHT_TRAVEL=0.026
SMALL_TURN_SPEED=500.0

STRAIGHT_TRAVEL=0.181
STRAIGHT_SPEED=700.0

TURN_RIGHT_TRAVEL=.102
TURN_SPEED=700.0

DRIVE_TO_WALL_SPEED=1200.0

BACK_UP_TRAVEL=0.0160
BACK_UP_SPEED=700.0

LEFT_TURN_SPEED=700.0
LEFT_TURN_TRAVEL=0.107

FORWARD_AFTER_TURN_TRAVEL=0.230
FORWARD_AFTER_TURN_SPEED=1000.0

MID_LEFT_TRAVEL=0.104
MID_FORWARD_TRAVEL=0.34
MID_BACKWARD_SPEED=1000.0
MID_RIGHT_TRAVEL=0.110

RESUME_LINE_TRAVEL=0.095

PRE_FINAL_FORWARD_TRAVEL=0.040
PRE_FINAL_FORWARD_SPEED=400.0

FINAL_RIGHT_TRAVEL=0.092
FINAL_FORWARD_TRAVEL=0.07
FINAL_FORWARD_SPEED=400.0

FINAL_LINE_KP=0.12
FINAL_LINE_KI=0.25
FINAL_LINE_SPEED=320.0
FINAL_LINE_TRAVEL=1.0

FAST_FINAL_KP=0.12
FAST_FINAL_KI=0.20

FAST_FINAL_SPEED_1=350.0
FAST_FINAL_TRAVEL_1=0.65

FAST_FINAL_SPEED_2=1000.0
FAST_FINAL_TRAVEL_2=0.5

CUP_TURN_SPEED=700.0
CUP_RIGHT_TRAVEL=0.105

CUP_STRAIGHT1_SPEED=700.0
CUP_STRAIGHT1_TRAVEL=0.1

CUP_LEFT1_TRAVEL=0.105

CUP_STRAIGHT2_SPEED=700.0
CUP_STRAIGHT2_TRAVEL=0.160

CUP_LEFT2_TRAVEL=0.110

CUP_HIT_SPEED=700.0
CUP_HIT_TRAVEL=0.15

POST_CUP_BACK_TRAVEL=0.05
POST_CUP_BACK_SPEED=700.0

POST_CUP_LEFT_TRAVEL=0.110
POST_CUP_LEFT_SPEED=700.0

POST_CUP_LINE_KP=0.06
POST_CUP_LINE_KI=0.35
POST_CUP_LINE_SPEED=1000.0
POST_CUP_LINE_TRAVEL=0.6

POST_CUP_RIGHT_TRAVEL=0.02
POST_CUP_RIGHT_SPEED=500.0

POST_CUP_FINAL_LINE_KP=0.06
POST_CUP_FINAL_LINE_KI=0.35
POST_CUP_FINAL_LINE_SPEED=600.0
POST_CUP_FINAL_LINE_TRAVEL=0.3

END_SMALL_RIGHT_TRAVEL=0.02
END_SMALL_RIGHT_SPEED=500.0

END_STRAIGHT_TRAVEL=0.1
END_STRAIGHT_SPEED=500.0

class task_course:
    def __init__(self,line_sensors,courseEnable,bumpEvent,leftMotorGo,rightMotorGo,lfEnable,estStreamEnable,velSetpoint_L,velSetpoint_R,vBaseShare,psi_meas,sL_meas,sR_meas,KpShare,KiShare):
        self._state=S0_INIT
        self._courseEnable=courseEnable
        self._bumpEvent=bumpEvent
        self._leftMotorGo=leftMotorGo
        self._rightMotorGo=rightMotorGo
        self._lfEnable=lfEnable
        self._velL=velSetpoint_L
        self._velR=velSetpoint_R
        self._vBase=vBaseShare
        self._sL_meas=sL_meas
        self._sR_meas=sR_meas
        self._Kp=KpShare
        self._Ki=KiShare
        self._enable_prev=False
        self._seg_sL0=0.0
        self._seg_sR0=0.0

    def _clear_bump_event(self):
        try:
            self._bumpEvent.put(False)
        except Exception:
            pass

    def _stop(self):
        try:
            self._lfEnable.put(False)
        except Exception:
            pass
        try:
            self._leftMotorGo.put(False)
        except Exception:
            pass
        try:
            self._rightMotorGo.put(False)
        except Exception:
            pass
        try:
            self._velL.put(0.0)
        except Exception:
            pass
        try:
            self._velR.put(0.0)
        except Exception:
            pass
        try:
            self._courseEnable.put(False)
        except Exception:
            pass

    def _apply_fast_settings(self):
        self._Kp.put(FAST_KP)
        self._Ki.put(FAST_KI)
        self._vBase.put(FAST_SPEED)

    def _apply_slow_settings(self):
        self._Kp.put(SLOW_KP)
        self._Ki.put(SLOW_KI)
        self._vBase.put(SLOW_SPEED)

    def _get_sL(self):
        try:
            return float(self._sL_meas.get())
        except Exception:
            return 0.0

    def _get_sR(self):
        try:
            return float(self._sR_meas.get())
        except Exception:
            return 0.0

    def _mark_segment_start(self):
        self._seg_sL0=self._get_sL()
        self._seg_sR0=self._get_sR()

    def _avg_forward_travel(self):
        sL=self._get_sL()
        sR=self._get_sR()
        dL=sL-self._seg_sL0
        dR=sR-self._seg_sR0
        return 0.5*(dL+dR)

    def _avg_backward_travel(self):
        sL=self._get_sL()
        sR=self._get_sR()
        dL=self._seg_sL0-sL
        dR=self._seg_sR0-sR
        return 0.5*(dL+dR)

    def _avg_abs_wheel_travel(self):
        sL=self._get_sL()
        sR=self._get_sR()
        dL=abs(sL-self._seg_sL0)
        dR=abs(sR-self._seg_sR0)
        return 0.5*(dL+dR)

    def _start_fast_line(self):
        self._apply_fast_settings()
        self._lfEnable.put(True)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._mark_segment_start()

    def _start_slow_line(self):
        self._apply_slow_settings()
        self._lfEnable.put(True)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._mark_segment_start()

    def _start_small_right(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._velL.put(SMALL_TURN_SPEED)
        self._velR.put(-SMALL_TURN_SPEED)
        self._mark_segment_start()

    def _start_straight(self,speed):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._velL.put(speed)
        self._velR.put(speed)
        self._mark_segment_start()

    def _start_right_turn(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._velL.put(TURN_SPEED)
        self._velR.put(-TURN_SPEED)
        self._mark_segment_start()

    def _start_drive_to_wall(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._clear_bump_event()
        self._velL.put(DRIVE_TO_WALL_SPEED)
        self._velR.put(DRIVE_TO_WALL_SPEED)

    def _start_back_up(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._velL.put(-BACK_UP_SPEED)
        self._velR.put(-BACK_UP_SPEED)
        self._mark_segment_start()

    def _start_left_turn(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._velL.put(-LEFT_TURN_SPEED)
        self._velR.put(LEFT_TURN_SPEED)
        self._mark_segment_start()

    def _start_forward_after_turn(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._velL.put(FORWARD_AFTER_TURN_SPEED)
        self._velR.put(FORWARD_AFTER_TURN_SPEED)
        self._mark_segment_start()

    def _start_resume_line(self):
        self._apply_slow_settings()
        self._lfEnable.put(True)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._mark_segment_start()

    def _start_cup_right_turn(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._velL.put(CUP_TURN_SPEED)
        self._velR.put(-CUP_TURN_SPEED)
        self._mark_segment_start()

    def _start_cup_left_turn(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._velL.put(-CUP_TURN_SPEED)
        self._velR.put(CUP_TURN_SPEED)
        self._mark_segment_start()

    def _start_post_cup_back(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._velL.put(-POST_CUP_BACK_SPEED)
        self._velR.put(-POST_CUP_BACK_SPEED)
        self._mark_segment_start()

    def _start_post_cup_left(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._velL.put(-POST_CUP_LEFT_SPEED)
        self._velR.put(POST_CUP_LEFT_SPEED)
        self._mark_segment_start()

    def _start_post_cup_line(self):
        self._Kp.put(POST_CUP_LINE_KP)
        self._Ki.put(POST_CUP_LINE_KI)
        self._vBase.put(POST_CUP_LINE_SPEED)
        self._lfEnable.put(True)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._mark_segment_start()

    def _start_post_cup_right(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._velL.put(POST_CUP_RIGHT_SPEED)
        self._velR.put(-POST_CUP_RIGHT_SPEED)
        self._mark_segment_start()

    def _start_post_cup_final_line(self):
        self._Kp.put(POST_CUP_FINAL_LINE_KP)
        self._Ki.put(POST_CUP_FINAL_LINE_KI)
        self._vBase.put(POST_CUP_FINAL_LINE_SPEED)
        self._lfEnable.put(True)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._mark_segment_start()
    
    def _start_fast_final_line_2(self):
        self._Kp.put(FAST_FINAL_KP)
        self._Ki.put(FAST_FINAL_KI)
        self._vBase.put(FAST_FINAL_SPEED_2)
        self._lfEnable.put(True)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._mark_segment_start()

    def _start_end_small_right(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._velL.put(END_SMALL_RIGHT_SPEED)
        self._velR.put(-END_SMALL_RIGHT_SPEED)
        self._mark_segment_start()

    def _start_end_straight(self):
        self._lfEnable.put(False)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._velL.put(END_STRAIGHT_SPEED)
        self._velR.put(END_STRAIGHT_SPEED)
        self._mark_segment_start()

    def run(self):
        while True:
            if self._state==S0_INIT:
                self._stop()
                self._clear_bump_event()
                self._enable_prev=False
                self._state=S1_IDLE

            elif self._state==S1_IDLE:
                en=bool(self._courseEnable.get())
                if en and not self._enable_prev:
                    self._clear_bump_event()
                    self._start_fast_line()
                    self._state=S2_FAST_LINE
                elif (not en) and self._enable_prev:
                    self._stop()
                    self._clear_bump_event()
                self._enable_prev=en

            elif self._state==S2_FAST_LINE:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=FAST_LINE_TRAVEL:
                    self._start_slow_line()
                    self._state=S3_SLOW_LINE

            elif self._state==S3_SLOW_LINE:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=SLOW_LINE_TRAVEL:
                    self._start_small_right()
                    self._state=S4_SMALL_RIGHT

            elif self._state==S4_SMALL_RIGHT:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_abs_wheel_travel()>=SMALL_RIGHT_TRAVEL:
                    self._start_straight(STRAIGHT_SPEED)
                    self._state=S5_STRAIGHT

            elif self._state==S5_STRAIGHT:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=STRAIGHT_TRAVEL:
                    self._start_right_turn()
                    self._state=S6_TURN_RIGHT

            elif self._state==S6_TURN_RIGHT:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_abs_wheel_travel()>=TURN_RIGHT_TRAVEL:
                    self._start_drive_to_wall()
                    self._state=S8_DRIVE_TO_WALL

            elif self._state==S8_DRIVE_TO_WALL:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._bumpEvent.get():
                    self._clear_bump_event()
                    self._start_back_up()
                    self._state=S9_BACK_UP

            elif self._state==S9_BACK_UP:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_backward_travel()>=BACK_UP_TRAVEL:
                    self._start_left_turn()
                    self._state=S10_TURN_LEFT

            elif self._state==S10_TURN_LEFT:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_abs_wheel_travel()>=LEFT_TURN_TRAVEL:
                    self._start_forward_after_turn()
                    self._state=S11_FORWARD_AFTER_TURN

            elif self._state==S11_FORWARD_AFTER_TURN:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=FORWARD_AFTER_TURN_TRAVEL:
                    self._start_left_turn()
                    self._state=S111_MID_LEFT

            elif self._state==S111_MID_LEFT:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_abs_wheel_travel()>=MID_LEFT_TRAVEL:
                    self._start_straight(FORWARD_AFTER_TURN_SPEED)
                    self._state=S112_MID_FORWARD

            elif self._state==S112_MID_FORWARD:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=MID_FORWARD_TRAVEL:
                    self._lfEnable.put(False)
                    self._leftMotorGo.put(True)
                    self._rightMotorGo.put(True)
                    self._velL.put(-MID_BACKWARD_SPEED)
                    self._velR.put(-MID_BACKWARD_SPEED)
                    self._mark_segment_start()
                    self._state=S113_MID_BACKWARD

            elif self._state==S113_MID_BACKWARD:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_backward_travel()>=MID_FORWARD_TRAVEL:
                    self._start_right_turn()
                    self._state=S114_MID_RIGHT

            elif self._state==S114_MID_RIGHT:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_abs_wheel_travel()>=MID_RIGHT_TRAVEL:
                    self._start_resume_line()
                    self._state=S12_RESUME_LINE

            elif self._state==S12_RESUME_LINE:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=RESUME_LINE_TRAVEL:
                    self._lfEnable.put(False)
                    self._leftMotorGo.put(True)
                    self._rightMotorGo.put(True)
                    self._velL.put(PRE_FINAL_FORWARD_SPEED)
                    self._velR.put(PRE_FINAL_FORWARD_SPEED)
                    self._mark_segment_start()
                    self._state=S16_PRE_FINAL_FORWARD

            elif self._state==S16_PRE_FINAL_FORWARD:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=PRE_FINAL_FORWARD_TRAVEL:
                    self._start_right_turn()
                    self._state=S13_FINAL_RIGHT

            elif self._state==S13_FINAL_RIGHT:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_abs_wheel_travel()>=FINAL_RIGHT_TRAVEL:
                    self._lfEnable.put(False)
                    self._leftMotorGo.put(True)
                    self._rightMotorGo.put(True)
                    self._velL.put(FINAL_FORWARD_SPEED)
                    self._velR.put(FINAL_FORWARD_SPEED)
                    self._mark_segment_start()
                    self._state=S14_FINAL_FORWARD

            elif self._state==S14_FINAL_FORWARD:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=FINAL_FORWARD_TRAVEL:
                    self._Kp.put(FINAL_LINE_KP)
                    self._Ki.put(FINAL_LINE_KI)
                    self._vBase.put(FINAL_LINE_SPEED)
                    self._lfEnable.put(True)
                    self._leftMotorGo.put(True)
                    self._rightMotorGo.put(True)
                    self._mark_segment_start()
                    self._state=S15_FINAL_LINE

            elif self._state==S15_FINAL_LINE:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=FINAL_LINE_TRAVEL:
                    self._Kp.put(FAST_FINAL_KP)
                    self._Ki.put(FAST_FINAL_KI)
                    self._vBase.put(FAST_FINAL_SPEED_1)
                    self._lfEnable.put(True)
                    self._leftMotorGo.put(True)
                    self._rightMotorGo.put(True)
                    self._mark_segment_start()
                    self._state=S17_FAST_FINAL_LINE_1

            elif self._state==S17_FAST_FINAL_LINE_1:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=FAST_FINAL_TRAVEL_1:
                    self._start_fast_final_line_2()
                    self._state=S18_FAST_FINAL_LINE_2

            elif self._state==S18_FAST_FINAL_LINE_2:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=FAST_FINAL_TRAVEL_2:
                    self._start_cup_right_turn()
                    self._state=S19_CUP_RIGHT_TURN

            elif self._state==S19_CUP_RIGHT_TURN:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_abs_wheel_travel()>=CUP_RIGHT_TRAVEL:
                    self._start_straight(CUP_STRAIGHT1_SPEED)
                    self._state=S20_CUP_STRAIGHT_1

            elif self._state==S20_CUP_STRAIGHT_1:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=CUP_STRAIGHT1_TRAVEL:
                    self._start_cup_left_turn()
                    self._state=S21_CUP_LEFT_TURN_1

            elif self._state==S21_CUP_LEFT_TURN_1:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_abs_wheel_travel()>=CUP_LEFT1_TRAVEL:
                    self._start_straight(CUP_STRAIGHT2_SPEED)
                    self._state=S22_CUP_STRAIGHT_2

            elif self._state==S22_CUP_STRAIGHT_2:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=CUP_STRAIGHT2_TRAVEL:
                    self._start_cup_left_turn()
                    self._state=S23_CUP_LEFT_TURN_2

            elif self._state==S23_CUP_LEFT_TURN_2:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_abs_wheel_travel()>=CUP_LEFT2_TRAVEL:
                    self._start_straight(CUP_HIT_SPEED)
                    self._state=S24_CUP_HIT_FORWARD

            elif self._state==S24_CUP_HIT_FORWARD:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=CUP_HIT_TRAVEL:
                    self._start_post_cup_back()
                    self._state=S25_POST_CUP_BACK

            elif self._state==S25_POST_CUP_BACK:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_backward_travel()>=POST_CUP_BACK_TRAVEL:
                    self._start_post_cup_left()
                    self._state=S26_POST_CUP_LEFT

            elif self._state==S26_POST_CUP_LEFT:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_abs_wheel_travel()>=POST_CUP_LEFT_TRAVEL:
                    self._start_post_cup_line()
                    self._state=S27_POST_CUP_LINE

            elif self._state==S27_POST_CUP_LINE:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=POST_CUP_LINE_TRAVEL:
                    self._start_post_cup_right()
                    self._state=S28_POST_CUP_RIGHT

            elif self._state==S28_POST_CUP_RIGHT:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_abs_wheel_travel()>=POST_CUP_RIGHT_TRAVEL:
                    self._start_post_cup_final_line()
                    self._state=S29_POST_CUP_FINAL_LINE

            elif self._state==S29_POST_CUP_FINAL_LINE:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=POST_CUP_FINAL_LINE_TRAVEL:
                    self._start_end_small_right()
                    self._state=S30_END_SMALL_RIGHT

            elif self._state==S30_END_SMALL_RIGHT:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_abs_wheel_travel()>=END_SMALL_RIGHT_TRAVEL:
                    self._start_end_straight()
                    self._state=S31_END_STRAIGHT

            elif self._state==S31_END_STRAIGHT:
                if not self._courseEnable.get():
                    self._stop()
                    self._clear_bump_event()
                    self._state=S1_IDLE
                    yield self._state
                    continue
                if self._avg_forward_travel()>=END_STRAIGHT_TRAVEL:
                    self._state=S7_STOP

            elif self._state==S7_STOP:
                self._stop()
                self._clear_bump_event()
                self._state=S1_IDLE

            yield self._state