#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Defines the starting values
TARGET_VALUE = int(50)
MOTOR_SPEED = 20
WHEEL_DIAMETER = 20
AXLE_TRACK = 100
PROPORTIONAL_COEFFICIENT = float(0.5)
INTEGRAL_COEFFICIENT = float(1)
DERIVATIVE_COEFFICIENT = float(4)

prevError = int(0)
prevErrors = list()
integralValue = int(0)

# Initialize the EV3Brick
ev3 = EV3Brick()
# Initialize and configure the motors
motorA = Motor("A")
motorB = Motor("B")

driveBase = DriveBase(motorA, motorB, )
# Initialize and configure the color / light intensity sensor
lightSensor = ColorSensor("S1")

def proportionalController(error, coefficient):
    return error * coefficient

def integralController(errorValues, cachedSum, newError, coefficient, maxLength = 1000):
    errorValues.append(newError)
    cachedSum += newError

    # When the maximum Length of the array is reached, removes the oldest value and subtracts it from the cached sum
    if len(errorValues) > maxLength:
        reversedErrorValues = errorValues.reverse()
        oldVal = reversedErrorValues.pop()
        errorValues = reversedErrorValues.reverse()
        cachedSum -= oldVal

    return {integralValue: cachedSum * coefficient, errorValues: errorValues}

def derivativeController(prevError, newError, coefficient):
    return (prevError - newError) * coefficient

lightIntensity = lightSensor.reflection()
error = TARGET_VALUE - lightIntensity

proportionalValue = proportionalController(error, PROPORTIONAL_COEFFICIENT)

integralDict = integralController(prevErrors, integralValue, error, INTEGRAL_COEFFICIENT)
integralValue = integralDict.integralValue
prevErrors = integralDict.errorValues

derivativeValue = derivativeController(prevError, error, DERIVATIVE_COEFFICIENT)

turnRate = proportionalValue + integralValue + derivativeValue

# Drives the Robot
driveBase.drive(MOTOR_SPEED, turnRate)