#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
#from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
#from pybricks.media.ev3dev import SoundFile, ImageFile

# Defines the starting tuning values
BLACK = 7
WHITE = 61
TARGET_VALUE = (BLACK + WHITE) / 2 # Gets the target value by calculating what percent of light reflected would equal 50% white and 50% black (by getting the mean value between the two values)

MOTOR_SPEED = 300
WHEEL_DIAMETER = 82
AXLE_TRACK = 120

PROPORTIONAL_COEFFICIENT = 1
INTEGRAL_COEFFICIENT = 0.5
DERIVATIVE_COEFFICIENT = 0

debug = True

# Initializes starting values
prevError = 0
prevErrors = []
integralCachedSum = 0

# Initializes the EV3Brick
ev3 = EV3Brick()
# Initializes and configures the motors
motorA = Motor(Port.A)
motorB = Motor(Port.D)
driveBase = DriveBase(motorA, motorB, WHEEL_DIAMETER, AXLE_TRACK)

# Initializes and configures the color / light intensity sensor
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

if debug:
    i = 0

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

    if(debug):
        i += 1
        ev3.screen.print("Iteration " + i)
        ev3.screen.print("Prop Value: " + proportionalValue)
        ev3.screen.print("Inte Value: " + integralValue)
        ev3.screen.print("Deri Value: " + derivativeValue)
        ev3.screen.print("Turn Rate: " + turnRate)

    # Drives the Robot
    if not(debug):
        driveBase.drive(MOTOR_SPEED, turnRate)
    else:
        driveBase.drive(MOTOR_SPEED, turnRate)
        wait(100)
        driveBase.stop()
        
        while not(ev3.buttons.pressed):
            wait(1)