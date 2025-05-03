#!/usr/bin/env pybricks-micropython
from robot import Robot
from pybricks.tools import wait
from pybricks.media.ev3dev import Font

robot = Robot()

robot.ev3.screen.set_font(Font('Lucida', 8))

robot.reset()

robot.followDegree(robot.MOTOR_SPEED, 0, condition=lambda: robot.lightSensor.reflection() > 25)
robot.ev3.speaker.beep()

robot.followLine(robot.MOTOR_SPEED * 0.75, condition=lambda: robot.ultrasonicSensor.distance() < 335)
robot.ev3.speaker.beep()

robot.followDegree(robot.MOTOR_SPEED * 1.75, robot.gyroSensor.angle(), condition=lambda: robot.lightSensor.reflection() > 9)
robot.ev3.speaker.beep()

robot.driveBase.reset()
robot.followLine(robot.MOTOR_SPEED * 0.6, condition=lambda: robot.lightSensor.reflection() > 10 or robot.driveBase.distance() < 210)
robot.ev3.speaker.beep()

robot.driveBase.turn(25)
robot.driveBase.straight(40)
robot.grasp()
robot.ev3.speaker.beep()

robot.driveBase.turn(-130)

robot.ev3.speaker.beep()

robot.driveBase.straight(50)
robot.driveBase.reset()
robot.followDegree(robot.MOTOR_SPEED, robot.gyroSensor.angle(), condition=lambda: robot.lightSensor.reflection() > 10 or robot.driveBase.distance() < 9000)
robot.release()

robot.ev3.speaker.beep()
robot.turn(-165)
#robot.followDegree

while(True):
    wait(10)