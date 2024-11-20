#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
#from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
#from pybricks.media.ev3dev import SoundFile, ImageFile

# Defines the starting values
BLACK = 10
WHITE = 50
TARGET_VALUE = (BLACK + WHITE) / 2 # Gets the target value by calculating what percent of light reflected would equal 50% white and 50% black (by getting the mean value between the two values) 
MOTOR_SPEED = 300
WHEEL_DIAMETER = 55.5
AXLE_TRACK = 114
PROPORTIONAL_COEFFICIENT = 2.5
INTEGRAL_COEFFICIENT = 0
DERIVATIVE_COEFFICIENT = 0

prevError = 0
prevErrors = []
integralCachedSum = 0

# Initialize the EV3Brick
ev3 = EV3Brick()
# Initialize and configure the motors
motorA = Motor(Port.A)
motorB = Motor(Port.D)
driveBase = DriveBase(motorA, motorB, WHEEL_DIAMETER, AXLE_TRACK)

# Initialize and configure the color / light intensity sensor
lightSensor = ColorSensor(Port.S4)

def proportionalController(error: int, coefficient: float) -> float:
    return error * coefficient

def integralController(errorValues, cachedSum: int, newError: int, maxLength: int = 1000) -> list:
    errorValues.append(newError)
    cachedSum += newError # The cached sum is just the cached sum of the list, as adding all values from the list together is very cpu-intensive

    # When the maximum Length of the array is reached, removes the oldest value and subtracts it from the cached sum
    if len(errorValues) > maxLength:
        # Removes the oldest value from the list and subtracts it from the cached sum, to keep it always up-to-date with the list
        oldVal = errorValues.pop(0)
        cachedSum -= oldVal

    return [cachedSum, errorValues]

def derivativeController(prevError: int, newError: int, coefficient: float) -> float:
    return (prevError - newError) * coefficient

while True:
    lightIntensity = lightSensor.reflection()
    error = TARGET_VALUE - lightIntensity

    proportionalValue = proportionalController(error, PROPORTIONAL_COEFFICIENT)

    integralList = integralController(prevErrors, integralCachedSum, error)
    integralCachedSum = integralList[0]
    integralValue = integralCachedSum * INTEGRAL_COEFFICIENT
    prevErrors = integralList[1]

    derivativeValue = derivativeController(prevError, error, DERIVATIVE_COEFFICIENT)

    turnRate = proportionalValue + integralValue + derivativeValue

    # Drives the Robot
    driveBase.drive(MOTOR_SPEED, turnRate)