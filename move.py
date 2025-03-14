#!/usr/bin/env python3
# File name   : move.py
# Description : Control Motor
# Product     : GWR
# Website     : www.gewbot.com
# Author      : William
# Date        : 2019/07/24

from __future__ import division
import time
import RPi.GPIO as GPIO
import sys
import Adafruit_PCA9685
import threading
import random
# motor_EN_A: Pin7  |  motor_EN_B: Pin11
# motor_A:  Pin8,Pin10    |  motor_B: Pin13,Pin12

#!/usr/bin/env python3
# File name   : servo.py
# Description : Control Servos
# Author	  : William
# Date		: 2019/02/23



'''
change this form 1 to -1 to reverse servos
'''
pwm = Adafruit_PCA9685.PCA9685(busnum=1)
time.sleep(1)
pwm.set_pwm_freq(50)

init_pwm0 = 351
init_pwm1 = 300
init_pwm2 = 300
init_pwm3 = 289

init_pwm4 = 300
init_pwm5 = 300
init_pwm6 = 300
init_pwm7 = 300

init_pwm8 = 300
init_pwm9 = 300
init_pwm10 = 300
init_pwm11 = 300

init_pwm12 = 300
init_pwm13 = 300
init_pwm14 = 300
init_pwm15 = 300

class ServoCtrl(threading.Thread):

	def __init__(self, *args, **kwargs):
		self.sc_direction = [1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1]
		self.initPos = [init_pwm0,init_pwm1,init_pwm2,init_pwm3,
						init_pwm4,init_pwm5,init_pwm6,init_pwm7,
						init_pwm8,init_pwm9,init_pwm10,init_pwm11,
						init_pwm12,init_pwm13,init_pwm14,init_pwm15]
		self.goalPos = [300,300,300,300, 300,300,300,300 ,300,300,300,300 ,300,300,300,300]
		self.nowPos  = [300,300,300,300, 300,300,300,300 ,300,300,300,300 ,300,300,300,300]
		self.bufferPos  = [300.0,300.0,300.0,300.0, 300.0,300.0,300.0,300.0 ,300.0,300.0,300.0,300.0 ,300.0,300.0,300.0,300.0]
		self.lastPos = [300,300,300,300, 300,300,300,300 ,300,300,300,300 ,300,300,300,300]
		self.ingGoal = [300,300,300,300, 300,300,300,300 ,300,300,300,300 ,300,300,300,300]
		self.maxPos  = [520,520,520,520, 520,520,520,520 ,520,520,520,520 ,520,520,520,520]
		self.minPos  = [100,100,100,100, 100,100,100,100 ,100,100,100,100 ,100,100,100,100]
		self.scSpeed = [0,0,0,0, 0,0,0,0 ,0,0,0,0 ,0,0,0,0]

		self.ctrlRangeMax = 560
		self.ctrlRangeMin = 100
		self.angleRange = 180

		'''
		scMode: 'init' 'auto' 'certain' 'quick' 'wiggle'
		'''
		self.scMode = 'auto'
		self.scTime = 2.0
		self.scSteps = 30
		
		self.scDelay = 0.037
		self.scMoveTime = 0.037

		self.goalUpdate = 0
		self.wiggleID = 0
		self.wiggleDirection = 1

		super(ServoCtrl, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()
		self.__flag.clear()


	def pause(self):
		print('......................pause..........................')
		self.__flag.clear()


	def resume(self):
		print('resume')
		self.__flag.set()


	def moveInit(self):
		self.scMode = 'init'
		for i in range(0,16):
			pwm.set_pwm(i,0,self.initPos[i])
			self.lastPos[i] = self.initPos[i]
			self.nowPos[i] = self.initPos[i]
			self.bufferPos[i] = float(self.initPos[i])
			self.goalPos[i] = self.initPos[i]
		self.pause()


	def initConfig(self, ID, initInput, moveTo):
		if initInput > self.minPos[ID] and initInput < self.maxPos[ID]:
			self.initPos[ID] = initInput
			if moveTo:
				pwm.set_pwm(ID,0,self.initPos[ID])
		else:
			print('initPos Value Error.')


	def moveServoInit(self, ID):
		self.scMode = 'init'
		for i in range(0,len(ID)):
			pwm.set_pwm(ID[i], 0, self.initPos[ID[i]])
			self.lastPos[ID[i]] = self.initPos[ID[i]]
			self.nowPos[ID[i]] = self.initPos[ID[i]]
			self.bufferPos[ID[i]] = float(self.initPos[ID[i]])
			self.goalPos[ID[i]] = self.initPos[ID[i]]
		self.pause()


	def posUpdate(self):
		self.goalUpdate = 1
		for i in range(0,16):
			self.lastPos[i] = self.nowPos[i]
		self.goalUpdate = 0


	def speedUpdate(self, IDinput, speedInput):
		for i in range(0,len(IDinput)):
			self.scSpeed[IDinput[i]] = speedInput[i]


	def moveAuto(self):
		for i in range(0,16):
			self.ingGoal[i] = self.goalPos[i]

		for i in range(0, self.scSteps):
			for dc in range(0,16):
				if not self.goalUpdate:
					self.nowPos[dc] = int(round((self.lastPos[dc] + (((self.goalPos[dc] - self.lastPos[dc])/self.scSteps)*(i+1))),0))
					pwm.set_pwm(dc, 0, self.nowPos[dc])

				if self.ingGoal != self.goalPos:
					self.posUpdate()
					time.sleep(self.scTime/self.scSteps)
					return 1
			time.sleep((self.scTime/self.scSteps - self.scMoveTime))

		self.posUpdate()
		self.pause()
		return 0


	def moveCert(self):
		for i in range(0,16):
			self.ingGoal[i] = self.goalPos[i]
			self.bufferPos[i] = self.lastPos[i]

		while self.nowPos != self.goalPos:
			for i in range(0,16):
				if self.lastPos[i] < self.goalPos[i]:
					self.bufferPos[i] += self.pwmGenOut(self.scSpeed[i])/(1/self.scDelay)
					newNow = int(round(self.bufferPos[i], 0))
					if newNow > self.goalPos[i]:newNow = self.goalPos[i]
					self.nowPos[i] = newNow
				elif self.lastPos[i] > self.goalPos[i]:
					self.bufferPos[i] -= self.pwmGenOut(self.scSpeed[i])/(1/self.scDelay)
					newNow = int(round(self.bufferPos[i], 0))
					if newNow < self.goalPos[i]:newNow = self.goalPos[i]
					self.nowPos[i] = newNow

				if not self.goalUpdate:
					pwm.set_pwm(i, 0, self.nowPos[i])

				if self.ingGoal != self.goalPos:
					self.posUpdate()
					return 1
			self.posUpdate()
			time.sleep(self.scDelay-self.scMoveTime)

		else:
			self.pause()
			return 0


	def pwmGenOut(self, angleInput):
		return int(round(((self.ctrlRangeMax-self.ctrlRangeMin)/self.angleRange*angleInput),0))


	def setAutoTime(self, autoSpeedSet):
		self.scTime = autoSpeedSet


	def setDelay(self, delaySet):
		self.scDelay = delaySet


	def autoSpeed(self, ID, angleInput):
		self.scMode = 'auto'
		self.goalUpdate = 1
		for i in range(0,len(ID)):
			newGoal = self.initPos[ID[i]] + self.pwmGenOut(angleInput[i])*self.sc_direction[ID[i]]
			if newGoal>self.maxPos[ID[i]]:newGoal=self.maxPos[ID[i]]
			elif newGoal<self.minPos[ID[i]]:newGoal=self.minPos[ID[i]]
			self.goalPos[ID[i]] = newGoal
		self.goalUpdate = 0
		self.resume()


	def certSpeed(self, ID, angleInput, speedSet):
		self.scMode = 'certain'
		self.goalUpdate = 1
		for i in range(0,len(ID)):
			newGoal = self.initPos[ID[i]] + self.pwmGenOut(angleInput[i])*self.sc_direction[ID[i]]
			if newGoal>self.maxPos[ID[i]]:newGoal=self.maxPos[ID[i]]
			elif newGoal<self.minPos[ID[i]]:newGoal=self.minPos[ID[i]]
			self.goalPos[ID[i]] = newGoal
		self.speedUpdate(ID, speedSet)
		self.goalUpdate = 0
		self.resume()


	def moveWiggle(self):
		self.bufferPos[self.wiggleID] += self.wiggleDirection*self.sc_direction[self.wiggleID]*self.pwmGenOut(self.scSpeed[self.wiggleID])/(1/self.scDelay)
		newNow = int(round(self.bufferPos[self.wiggleID], 0))
		if self.bufferPos[self.wiggleID] > self.maxPos[self.wiggleID]:self.bufferPos[self.wiggleID] = self.maxPos[self.wiggleID]
		elif self.bufferPos[self.wiggleID] < self.minPos[self.wiggleID]:self.bufferPos[self.wiggleID] = self.minPos[self.wiggleID]
		self.nowPos[self.wiggleID] = newNow
		self.lastPos[self.wiggleID] = newNow
		if self.bufferPos[self.wiggleID] < self.maxPos[self.wiggleID] and self.bufferPos[self.wiggleID] > self.minPos[self.wiggleID]:
			pwm.set_pwm(self.wiggleID, 0, self.nowPos[self.wiggleID])
		else:
			self.stopWiggle()
		time.sleep(self.scDelay-self.scMoveTime)


	def stopWiggle(self):
		self.pause()
		self.posUpdate()


	def singleServo(self, ID, direcInput, speedSet):
		self.wiggleID = ID
		self.wiggleDirection = direcInput
		self.scSpeed[ID] = speedSet
		self.scMode = 'wiggle'
		self.posUpdate()
		self.resume()


	def moveAngle(self, ID, angleInput):
		self.nowPos[ID] = int(self.initPos[ID] + self.sc_direction[ID]*self.pwmGenOut(angleInput))
		if self.nowPos[ID] > self.maxPos[ID]:self.nowPos[ID] = self.maxPos[ID]
		elif self.nowPos[ID] < self.minPos[ID]:self.nowPos[ID] = self.minPos[ID]
		self.lastPos[ID] = self.nowPos[ID]
		pwm.set_pwm(ID, 0, self.nowPos[ID])


	def scMove(self):
		if self.scMode == 'init':
			self.moveInit()
		elif self.scMode == 'auto':
			self.moveAuto()
		elif self.scMode == 'certain':
			self.moveCert()
		elif self.scMode == 'wiggle':
			self.moveWiggle()


	def setPWM(self, ID, PWM_input):
		self.lastPos[ID] = PWM_input
		self.nowPos[ID] = PWM_input
		self.bufferPos[ID] = float(PWM_input)
		self.goalPos[ID] = PWM_input
		pwm.set_pwm(ID, 0, PWM_input)
		self.pause()


	def run(self):
		while 1:
			self.__flag.wait()
			self.scMove()
			pass



Motor_A_EN    = 4
Motor_B_EN    = 17

Motor_A_Pin1  = 26
Motor_A_Pin2  = 21
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

Dir_forward   = 0
Dir_backward  = 1

left_forward  = 1
left_backward = 0

right_forward = 0
right_backward= 1

pwn_A = 0
pwm_B = 0

def motorStop():#Motor stops
	GPIO.output(Motor_A_Pin1, GPIO.LOW)
	GPIO.output(Motor_A_Pin2, GPIO.LOW)
	GPIO.output(Motor_B_Pin1, GPIO.LOW)
	GPIO.output(Motor_B_Pin2, GPIO.LOW)
	GPIO.output(Motor_A_EN, GPIO.LOW)
	GPIO.output(Motor_B_EN, GPIO.LOW)


def setup():#Motor initialization
	global pwm_A, pwm_B
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(Motor_A_EN, GPIO.OUT)
	GPIO.setup(Motor_B_EN, GPIO.OUT)
	GPIO.setup(Motor_A_Pin1, GPIO.OUT)
	GPIO.setup(Motor_A_Pin2, GPIO.OUT)
	GPIO.setup(Motor_B_Pin1, GPIO.OUT)
	GPIO.setup(Motor_B_Pin2, GPIO.OUT)

	motorStop()
	try:
		pwm_A = GPIO.PWM(Motor_A_EN, 1000)
		pwm_B = GPIO.PWM(Motor_B_EN, 1000)
	except:
		pass


def motor_left(status, direction, speed):#Motor 2 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_B_Pin1, GPIO.LOW)
		GPIO.output(Motor_B_Pin2, GPIO.LOW)
		GPIO.output(Motor_B_EN, GPIO.LOW)
	else:
		if direction == Dir_backward:
			GPIO.output(Motor_B_Pin1, GPIO.HIGH)
			GPIO.output(Motor_B_Pin2, GPIO.LOW)
			pwm_B.start(100)
			pwm_B.ChangeDutyCycle(speed)
		elif direction == Dir_forward:
			GPIO.output(Motor_B_Pin1, GPIO.LOW)
			GPIO.output(Motor_B_Pin2, GPIO.HIGH)
			pwm_B.start(0)
			pwm_B.ChangeDutyCycle(speed)


def motor_right(status, direction, speed):#Motor 1 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_A_Pin1, GPIO.LOW)
		GPIO.output(Motor_A_Pin2, GPIO.LOW)
		GPIO.output(Motor_A_EN, GPIO.LOW)
	else:
		if direction == Dir_forward:#
			GPIO.output(Motor_A_Pin1, GPIO.HIGH)
			GPIO.output(Motor_A_Pin2, GPIO.LOW)
			pwm_A.start(100)
			pwm_A.ChangeDutyCycle(speed)
		elif direction == Dir_backward:
			GPIO.output(Motor_A_Pin1, GPIO.LOW)
			GPIO.output(Motor_A_Pin2, GPIO.HIGH)
			pwm_A.start(0)
			pwm_A.ChangeDutyCycle(speed)
	return direction


def move(speed, direction, turn, radius=0.6):   # 0 < radius <= 1  
	#speed = 100
	if direction == 'forward':
		if turn == 'right':
			motor_left(0, left_backward, int(speed*radius))
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(0, right_backward, int(speed*radius))
		else:
			motor_left(1, left_forward, speed)
			motor_right(1, right_forward, speed)
	elif direction == 'backward':
		if turn == 'right':
			motor_left(0, left_forward, int(speed*radius))
			motor_right(1, right_backward, speed)
		elif turn == 'left':
			motor_left(1, left_backward, speed)
			motor_right(0, right_forward, int(speed*radius))
		else:
			motor_left(1, left_backward, speed)
			motor_right(1, right_backward, speed)
	elif direction == 'no':
		if turn == 'right':
			motor_left(1, left_backward, speed)
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(1, right_backward, speed)
		else:move




def destroy():
	motorStop()
	GPIO.cleanup()             # Release resource


def DriveStraight():
	pwm.set_pwm(0, 0, int(round((520+100)/2)))
	time.sleep(0.3)
	move(60, 'forward', 'no', 0.8)

def DriveLeft():
	pwm.set_pwm(0, 0, 520)
	time.sleep(0.3)
	move(60, 'forward', 'no', 0.8)

def DriveRight():
	#sc.moveAngle(0,-60)
	pwm.set_pwm(0, 0, 100)
	time.sleep(0.3)
	move(60, 'forward', 'no', 0.8)

setup()
sc = ServoCtrl()
sc.start()

FORWARD_LONG_TIME = 1.3  # Avanzar 1 metro
FORWARD_SHORT_TIME = 0.26  # Avanzar 20 cm
SPIN_TIME = 1.5  # Giro de 180° (ajustar según sea necesario)

try:
	speed_set = 60
	setup()
	running = True
	while running:

		# Secuencia de movimientos
		DriveStraight()
		time.sleep(FORWARD_LONG_TIME)
		motorStop()

		DriveRight()
		time.sleep(SPIN_TIME)
		motorStop()

		DriveStraight()
		time.sleep(FORWARD_SHORT_TIME)
		motorStop()

		DriveRight()
		time.sleep(SPIN_TIME)
		motorStop()

		DriveStraight()
		time.sleep(FORWARD_LONG_TIME)
		motorStop()

		DriveLeft()
		time.sleep(SPIN_TIME)
		motorStop()

		DriveStraight()
		time.sleep(FORWARD_SHORT_TIME)
		motorStop()

		DriveLeft()
		time.sleep(SPIN_TIME)
		motorStop()

except KeyboardInterrupt:
	destroy()


time.sleep(1)
motorStop()
destroy()