#!/usr/bin/env pybricks-micropython

class PIDControl:
    def __init__(self, targetVal: float, propGain: float, integGain: float, derivGain: float):
        self.targetVal = targetVal
        self.propGain = propGain
        self.integGain = integGain
        self.derivGain = derivGain

        self.integSum = 0
        self.prevError = 0

        self.outOfControl = False

        self.debug = False
    
    def setTargetVal(self, newVal: int):
        self.targetVal = newVal

    def calcProp(self, error: int) -> float:
        return error * self.propGain

    def calcInteg(self, error: int) -> float:
        self.integSum += error

        if(self.integSum >= 1000):
            self.outOfControl = True
        elif(self.outOfControl):
            self.outOfControl = False

        return self.integSum * self.integGain

    def calcDeriv(self, error: int) -> float:
        return (error - self.prevError) * self.derivGain

    def calcPID(self, measuredVal: int):
        error = self.targetVal - measuredVal

        prop = self.calcProp(error)
        integ = self.calcInteg(error)
        deriv = self.calcDeriv(error)

        pidOut = prop + integ + deriv

        self.prevError = error

        return pidOut