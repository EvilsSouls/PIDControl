#!/usr/bin/env pybricks-micropython

class PIDControl:
    def __init__(self, targetVal: float, propGain: float, integGain: float, derivGain: float):
        self.targetVal = targetVal
        self.propGain = propGain
        self.integGain = integGain
        self.derivGain = derivGain
        
        self.integSum = 0
        self.prevError = 0

        self.debug = debug
    
    def calcProp(error: int) -> float:
        return error * self.propGain

    def calcInteg(error: int) -> float:
        self.integSum += error
        return self.integSum * self.integGain

    def calcDeriv(error: int) -> float:
        return (error - prevError) * self.derivGain

    def calcPID(measuredVal: int):
        error = self.targetVal - measuredVal

        prop = calcProp(error)
        integ = calcInteg(error)
        deriv = calcDeriv(error)

        pidOut = prop + integ + deriv

        prevError = error

        return pidOut