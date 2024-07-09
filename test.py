import time
import Adafruit_PCA9685
from mpu6050 import mpu6050
import Kalman_filter
import PID
import threading

set_direction = 1

if set_direction:
	leftSide_direction  = 1
	rightSide_direction = 0
else:
	leftSide_direction  = 0
	rightSide_direction = 1

if set_direction:
	leftSide_height  = 0
	rightSide_height = 1
else:
	leftSide_height  = 1
	rightSide_height = 0


height_change = 30
P = 5
I = 0.01
D = 0


pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

pwm0 = 300
pwm1 = 300
pwm2 = 300
pwm3 = 300

pwm4 = 300
pwm5 = 300
pwm6 = 300
pwm7 = 300

pwm8 = 300
pwm9 = 300
pwm10 = 300
pwm11 = 300

pwm12 = 300
pwm13 = 300
pwm14 = 300
pwm15 = 300

X_pid = PID.PID()
X_pid.SetKp(P)
X_pid.SetKd(I)
X_pid.SetKi(D)
Y_pid = PID.PID()
Y_pid.SetKp(P)
Y_pid.SetKd(I)
Y_pid.SetKi(D)
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)
kalman_filter_X =  Kalman_filter.Kalman_filter(0.001,0.1)
kalman_filter_Y =  Kalman_filter.Kalman_filter(0.001,0.1)

def init_all():
	pwm.set_pwm(0, 0, pwm0)
	pwm.set_pwm(1, 0, pwm1)
	pwm.set_pwm(2, 0, pwm2)
	pwm.set_pwm(3, 0, pwm3)

	pwm.set_pwm(4, 0, pwm4)
	pwm.set_pwm(5, 0, pwm5)
	pwm.set_pwm(6, 0, pwm6)
	pwm.set_pwm(7, 0, pwm7)

	pwm.set_pwm(8, 0, pwm8)
	pwm.set_pwm(9, 0, pwm9)
	pwm.set_pwm(10, 0, pwm10)
	pwm.set_pwm(11, 0, pwm11)

	pwm.set_pwm(12, 0, pwm12)
	pwm.set_pwm(13, 0, pwm13)
	pwm.set_pwm(14, 0, pwm14)
	pwm.set_pwm(15, 0, pwm15)

init_all()

def left_I(pos,wiggle,heightAdjust=0):
	if pos == 0:
		#pwm.set_pwm(0,0,pwm0)
		if leftSide_height:
			pwm.set_pwm(1,0,pwm1+heightAdjust)
		else:
			pwm.set_pwm(1,0,pwm1-heightAdjust)
	else:
		if leftSide_direction:
			if pos == 1:
				pwm.set_pwm(0,0,pwm0)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1+3*height_change)
				else:
					pwm.set_pwm(1,0,pwm1-3*height_change)
			elif pos == 2:
				pwm.set_pwm(0,0,pwm0+wiggle)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1-height_change)
				else:
					pwm.set_pwm(1,0,pwm1+height_change)
			elif pos == 3:
				pwm.set_pwm(0,0,pwm0)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1-height_change)
				else:
					pwm.set_pwm(1,0,pwm1+height_change)
			elif pos == 4:
				pwm.set_pwm(0,0,pwm0-wiggle)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1-height_change)
				else:
					pwm.set_pwm(1,0,pwm1+height_change)
		else:
			if pos == 1:
				pwm.set_pwm(0,0,pwm0)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1+3*wiggle)
				else:
					pwm.set_pwm(1,0,pwm1-3*wiggle)
			elif pos == 2:
				pwm.set_pwm(0,0,pwm0-wiggle)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1-wiggle)
				else:
					pwm.set_pwm(1,0,pwm1+wiggle)
			elif pos == 3:
				pwm.set_pwm(0,0,pwm0)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1-wiggle)
				else:
					pwm.set_pwm(1,0,pwm1+wiggle)
			elif pos == 4:
				pwm.set_pwm(0,0,pwm0+wiggle)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1-wiggle)
				else:
					pwm.set_pwm(1,0,pwm1+wiggle)


def left_II(pos,wiggle,heightAdjust=0):
	if pos == 0:
		#pwm.set_pwm(2,0,pwm2)
		if leftSide_height:
			pwm.set_pwm(3,0,pwm3+heightAdjust)
		else:
			pwm.set_pwm(3,0,pwm3-heightAdjust)
	else:
		if leftSide_direction:
			if pos == 1:
				pwm.set_pwm(2,0,pwm2)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3+3*height_change)
				else:
					pwm.set_pwm(3,0,pwm3-3*height_change)
			elif pos == 2:
				pwm.set_pwm(2,0,pwm2+wiggle)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3-height_change)
				else:
					pwm.set_pwm(3,0,pwm3+height_change)
			elif pos == 3:
				pwm.set_pwm(2,0,pwm2)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3-height_change)
				else:
					pwm.set_pwm(3,0,pwm3+height_change)
			elif pos == 4:
				pwm.set_pwm(2,0,pwm2-wiggle)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3-height_change)
				else:
					pwm.set_pwm(3,0,pwm3+height_change)
		else:
			if pos == 1:
				pwm.set_pwm(2,0,pwm2)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3+3*wiggle)
				else:
					pwm.set_pwm(3,0,pwm3-3*wiggle)
			elif pos == 2:
				pwm.set_pwm(2,0,pwm2-wiggle)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3-wiggle)
				else:
					pwm.set_pwm(3,0,pwm3+wiggle)
			elif pos == 3:
				pwm.set_pwm(2,0,pwm2)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3-wiggle)
				else:
					pwm.set_pwm(3,0,pwm3+wiggle)
			elif pos == 4:
				pwm.set_pwm(2,0,pwm2+wiggle)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3-wiggle)
				else:
					pwm.set_pwm(3,0,pwm3+wiggle)


def left_III(pos,wiggle,heightAdjust=0):
	if pos == 0:
		#pwm.set_pwm(4,0,pwm4)
		if leftSide_height:
			pwm.set_pwm(5,0,pwm5+heightAdjust)
		else:
			pwm.set_pwm(5,0,pwm5-heightAdjust)
	else:
		if leftSide_direction:
			if pos == 1:
				pwm.set_pwm(4,0,pwm4)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5+3*height_change)
				else:
					pwm.set_pwm(5,0,pwm5-3*height_change)
			elif pos == 2:
				pwm.set_pwm(4,0,pwm4+wiggle)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5-height_change)
				else:
					pwm.set_pwm(5,0,pwm5+height_change)
			elif pos == 3:
				pwm.set_pwm(4,0,pwm4)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5-height_change)
				else:
					pwm.set_pwm(5,0,pwm5+height_change)
			elif pos == 4:
				pwm.set_pwm(4,0,pwm4-wiggle)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5-height_change)
				else:
					pwm.set_pwm(5,0,pwm5+height_change)
		else:
			if pos == 1:
				pwm.set_pwm(4,0,pwm4)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5+3*wiggle)
				else:
					pwm.set_pwm(5,0,pwm5-3*wiggle)
			elif pos == 2:
				pwm.set_pwm(4,0,pwm4-wiggle)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5-wiggle)
				else:
					pwm.set_pwm(5,0,pwm5+wiggle)
			elif pos == 3:
				pwm.set_pwm(4,0,pwm4)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5-wiggle)
				else:
					pwm.set_pwm(5,0,pwm5+wiggle)
			elif pos == 4:
				pwm.set_pwm(4,0,pwm4+wiggle)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5-wiggle)
				else:
					pwm.set_pwm(5,0,pwm5+wiggle)


def right_I(pos,wiggle,heightAdjust=0):
	#wiggle = -wiggle
	if pos == 0:
		#pwm.set_pwm(6,0,pwm6)
		if rightSide_height:
			pwm.set_pwm(7,0,pwm7+heightAdjust)
		else:
			pwm.set_pwm(7,0,pwm7-heightAdjust)
	else:
		if rightSide_direction:
			if pos == 1:
				pwm.set_pwm(6,0,pwm6)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7+3*height_change)
				else:
					pwm.set_pwm(7,0,pwm7-3*height_change)
			elif pos == 2:
				pwm.set_pwm(6,0,pwm6+wiggle)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7-height_change)
				else:
					pwm.set_pwm(7,0,pwm7+height_change)
			elif pos == 3:
				pwm.set_pwm(6,0,pwm6)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7-height_change)
				else:
					pwm.set_pwm(7,0,pwm7+height_change)
			elif pos == 4:
				pwm.set_pwm(6,0,pwm6-wiggle)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7-height_change)
				else:
					pwm.set_pwm(7,0,pwm7+height_change)
		else:
			if pos == 1:
				pwm.set_pwm(6,0,pwm6)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7+3*height_change)
				else:
					pwm.set_pwm(7,0,pwm7-3*height_change)
			elif pos == 2:
				pwm.set_pwm(6,0,pwm6-wiggle)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7-height_change)
				else:
					pwm.set_pwm(7,0,pwm7+height_change)
			elif pos == 3:
				pwm.set_pwm(6,0,pwm6)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7-height_change)
				else:
					pwm.set_pwm(7,0,pwm7+height_change)
			elif pos == 4:
				pwm.set_pwm(6,0,pwm6+wiggle)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7-height_change)
				else:
					pwm.set_pwm(7,0,pwm7+height_change)


def right_II(pos,wiggle,heightAdjust=0):
	#wiggle = -wiggle
	if pos == 0:
		#pwm.set_pwm(8,0,pwm8)
		if rightSide_height:
			pwm.set_pwm(9,0,pwm9+heightAdjust)
		else:
			pwm.set_pwm(9,0,pwm9-heightAdjust)
	else:
		if rightSide_direction:
			if pos == 1:
				pwm.set_pwm(8,0,pwm8)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9+3*height_change)
				else:
					pwm.set_pwm(9,0,pwm9-3*height_change)
			elif pos == 2:
				pwm.set_pwm(8,0,pwm8+wiggle)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9-height_change)
				else:
					pwm.set_pwm(9,0,pwm9+height_change)
			elif pos == 3:
				pwm.set_pwm(8,0,pwm8)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9-height_change)
				else:
					pwm.set_pwm(9,0,pwm9+height_change)
			elif pos == 4:
				pwm.set_pwm(8,0,pwm8-wiggle)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9-height_change)
				else:
					pwm.set_pwm(9,0,pwm9+height_change)
		else:
			if pos == 1:
				pwm.set_pwm(8,0,pwm8)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9+3*height_change)
				else:
					pwm.set_pwm(9,0,pwm9-3*height_change)
			elif pos == 2:
				pwm.set_pwm(8,0,pwm8-wiggle)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9-height_change)
				else:
					pwm.set_pwm(9,0,pwm9+height_change)
			elif pos == 3:
				pwm.set_pwm(8,0,pwm8)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9-height_change)
				else:
					pwm.set_pwm(9,0,pwm9+height_change)
			elif pos == 4:
				pwm.set_pwm(8,0,pwm8+wiggle)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9-height_change)
				else:
					pwm.set_pwm(9,0,pwm9+height_change)


def right_III(pos,wiggle,heightAdjust=0):
	#wiggle = -wiggle
	if pos == 0:
		#pwm.set_pwm(10,0,pwm10)
		if rightSide_height:
			pwm.set_pwm(11,0,pwm11+heightAdjust)
		else:
			pwm.set_pwm(11,0,pwm11-heightAdjust)
	else:
		if rightSide_direction:
			if pos == 1:
				pwm.set_pwm(10,0,pwm10)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11+3*height_change)
				else:
					pwm.set_pwm(11,0,pwm11-3*height_change)
			elif pos == 2:
				pwm.set_pwm(10,0,pwm10+wiggle)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11-height_change)
				else:
					pwm.set_pwm(11,0,pwm11+height_change)
			elif pos == 3:
				pwm.set_pwm(10,0,pwm10)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11-height_change)
				else:
					pwm.set_pwm(11,0,pwm11+height_change)
			elif pos == 4:
				pwm.set_pwm(10,0,pwm10-wiggle)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11-height_change)
				else:
					pwm.set_pwm(11,0,pwm11+height_change)
		else:
			if pos == 1:
				pwm.set_pwm(10,0,pwm10)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11+3*height_change)
				else:
					pwm.set_pwm(11,0,pwm11-3*height_change)
			elif pos == 2:
				pwm.set_pwm(10,0,pwm10-wiggle)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11-height_change)
				else:
					pwm.set_pwm(11,0,pwm11+height_change)
			elif pos == 3:
				pwm.set_pwm(10,0,pwm10)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11-height_change)
				else:
					pwm.set_pwm(11,0,pwm11+height_change)
			elif pos == 4:
				pwm.set_pwm(10,0,pwm10+wiggle)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11-height_change)
				else:
					pwm.set_pwm(11,0,pwm11+height_change)


def move(step_input, speed, command):
	step_I  = step_input
	step_II = step_input + 2

	if step_II > 4:
		step_II = step_II - 4
	if speed == 0:
		return

	if command == 'no':
		right_I(step_I, speed, 0)
		left_II(step_I, speed, 0)
		right_III(step_I, speed, 0)

		left_I(step_II, speed, 0)
		right_II(step_II, speed, 0)
		left_III(step_II, speed, 0)
	elif command == 'left':
		right_I(step_I, speed, 0)
		left_II(step_I, -speed, 0)
		right_III(step_I, speed, 0)

		left_I(step_II, -speed, 0)
		right_II(step_II, speed, 0)
		left_III(step_II, -speed, 0)
	elif command == 'right':
		right_I(step_I, -speed, 0)
		left_II(step_I, speed, 0)
		right_III(step_I, -speed, 0)

		left_I(step_II, speed, 0)
		right_II(step_II, -speed, 0)
		left_III(step_II, speed, 0)

def stand():
	pwm.set_pwm(0,0,300)
	pwm.set_pwm(1,0,300)
	pwm.set_pwm(2,0,300)
	pwm.set_pwm(3,0,300)
	pwm.set_pwm(4,0,300)
	pwm.set_pwm(5,0,300)
	pwm.set_pwm(6,0,300)
	pwm.set_pwm(7,0,300)
	pwm.set_pwm(8,0,300)
	pwm.set_pwm(9,0,300)
	pwm.set_pwm(10,0,300)
	pwm.set_pwm(11,0,300)

direction_command = 'no'
turn_command = 'no'

def move_thread():
    global step_set
    step_set = 1
    while True:
        if direction_command == 'forward' and turn_command == 'no':
            move(step_set, 35, 'no')
            time.sleep(0.1)
            step_set += 1
            if step_set == 5:
                step_set = 1
        elif direction_command == 'backward' and turn_command == 'no':
            move(step_set, -35, 'no')
            time.sleep(0.1)
            step_set += 1
            if step_set == 5:
                step_set = 1
        elif turn_command == 'left':
            move(step_set, 35, 'left')
            time.sleep(0.1)
            step_set += 1
            if step_set == 5:
                step_set = 1
        elif turn_command == 'right':
            move(step_set, 35, 'right')
            time.sleep(0.1)
            step_set += 1
            if step_set == 5:
                step_set = 1
        else:
            stand()
            time.sleep(0.1)

def command_input():
    global direction_command, turn_command
    while True:
        command = input("Enter command (forward, backward, left, right, stop): ")
        if command == 'forward':
            direction_command = 'forward'
            turn_command = 'no'
        elif command == 'backward':
            direction_command = 'backward'
            turn_command = 'no'
        elif command == 'left':
            direction_command = 'no'
            turn_command = 'left'
        elif command == 'right':
            direction_command = 'no'
            turn_command = 'right'
        elif command == 'stop':
            direction_command = 'no'
            turn_command = 'no'
            stand()

if __name__ == "__main__":
    thread1 = threading.Thread(target=move_thread)
    thread2 = threading.Thread(target=command_input)
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()