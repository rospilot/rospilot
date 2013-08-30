'''
Copyright 2012 the original author or authors.
See the NOTICE file distributed with this work for additional
information regarding copyright ownership.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
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
        self.i += val * dt
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
        return p + i + d
