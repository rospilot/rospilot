'''
Copyright 2012 Christopher Berner

This file is part of Rospilot.

Rospilot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Rospilot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Rospilot.  If not, see <http://www.gnu.org/licenses/>.
'''

class PidController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()

    def reset(self):
        self.i = 0
        self.last_val = None
        self.val = None
        self.dt = 0

    def update(self, val, dt):
        self.i += val*dt
        self.last_val = self.val
        self.val = val
        self.dt = dt

    def get_control(self):
        p = self.kp * self.val
        i = self.ki * self.i
        if self.last_val:
            d = self.kd * (self.val - self.last_val) / self.dt
        else:
            d = 0
        return  p + i + d


