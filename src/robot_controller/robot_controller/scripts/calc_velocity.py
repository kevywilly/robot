#!/usr/bin/python3

import time
from rosmaster import Rosmaster
import numpy as np
import math
import atexit

bot = Rosmaster(car_type=2)
bot.create_receive_threading()


class VelocityCalculatorResult:
    def __init__(self, requested_velocity: float, actual_velocity: float):
        self.requested_velocity = requested_velocity
        self.actual_velocity = actual_velocity
        self.ratio = self.requested_velocity/self.actual_velocity

    def __str__(self):
        return f"requested: {self.requested_velocity}, actual: {self.actual_velocity}, ratio: {self.ratio}"

class VelocityCalculator:
    def __init__(self, ticks_per_rev: float = 2000, wheel_radius_meters = 97.0/2000.0):

        self.ticks_per_rev: float = ticks_per_rev
        self.sleep_seconds: float = 1.0
        self.wheel_radius_meters = wheel_radius_meters
        self.results = []

        atexit.register(self.stop)

    def stop(self):
        bot.set_car_motion(0,0,0)
    
    def start(self):
        speeds = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]

        for s in speeds:
            ar = []
            bot.set_car_motion(s,0,0)
            time.sleep(self.sleep_seconds)
            for i in range(5):
                ticks0 = np.array(bot.get_motor_encoder())
                time.sleep(self.sleep_seconds)
                ticks1 = np.array(bot.get_motor_encoder())
                diff = ticks1-ticks0
                ar.append(diff)
            
            avg_ticks = np.average(np.array(ar))
            rps = (avg_ticks/self.ticks_per_rev)/self.sleep_seconds
            rpm = rps * 60.0
            mps = rps * 2.0*math.pi*self.wheel_radius_meters

            result = VelocityCalculatorResult(requested_velocity=s, actual_velocity=mps)
            print(result)
            self.results.append(result)

        bot.set_car_motion(0,0,0)

calculator = VelocityCalculator(ticks_per_rev=2000, wheel_radius_meters=97.0/2000)
calculator.start()