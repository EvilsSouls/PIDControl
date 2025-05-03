#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, GyroSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pid import PIDControl

class Robot:
    def __init__(self):
        # Defines the starting tuning values
        self.BLACK = 4
        self.WHITE = 55

        self.MOTOR_SPEED = 100
        self.WHEEL_DIAMETER = 82
        self.AXLE_TRACK = 95

        self.LINE_PROPORTIONAL_GAIN = 0.9
        self.LINE_INTEGRAL_GAIN = 0
        self.LINE_DERIVATIVE_GAIN = 0

        # Initializes the EV3Brick
        self.ev3 = EV3Brick()
        # Initializes and configures the motors
        self.frontGripper = Motor(Port.B)
        self.motorA = Motor(Port.A)
        self.motorB = Motor(Port.D)
        self.driveBase = DriveBase(self.motorA, self.motorB, self.WHEEL_DIAMETER, self.AXLE_TRACK)
        # Initializes and configures the sensors
        self.lightSensor = ColorSensor(Port.S4)
        self.gyroSensor = GyroSensor(Port.S1)
        self.ultrasonicSensor = UltrasonicSensor(Port.S2)

        self.linePID = PIDControl((self.BLACK + self.WHITE) / 2, self.LINE_PROPORTIONAL_GAIN, self.LINE_INTEGRAL_GAIN, self.LINE_DERIVATIVE_GAIN)
        self.degreePID = PIDControl(None, 1.5, 0, 0)

    def grasp(self):
        self.frontGripper.run_target(60, -190)

    def release(self):
        self.frontGripper.run_target(60, 0)

    def followLine(self, speed, condition):
        while(condition()):
            self.ev3.screen.print(str(self.driveBase.distance()))
            turnRate = self.linePID.calcPID(self.lightSensor.reflection())

            # Drives the Robot
            self.driveBase.drive(speed, turnRate)

    def followDegree(self, speed, degree, condition):
        while(condition()):
            self.degreePID.setTargetVal(degree)
            self.ev3.screen.print(str(self.gyroSensor.angle()) + ", " + str(degree))
            turnRate = self.degreePID.calcPID(self.gyroSensor.angle())

            # Drives the Robot
            self.driveBase.drive(speed, turnRate)
        
        self.driveBase.stop()

    def reset(self):
        self.driveBase.stop()
        wait(500)
        self.driveBase.reset()
        self.gyroSensor.reset_angle(0)
        self.frontGripper.reset_angle(0)