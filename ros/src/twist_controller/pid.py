
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time):
        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time
        derivative = (error - self.last_error) / sample_time

        y = self.kp * error + self.ki * self.int_val + self.kd * derivative
        ##val = max(self.min, min(y, self.max)) 

        if y > self.max:
            val = self.max 
            ## self.int_val = (self.max - self.kp * error - self.kd * derivative) / self.ki #(wrong implementation)       
        elif y < self.min:
            val = self.min 
            ## self.int_val = (self.min - self.kp * error - self.kd * derivative) / self.ki #(wrong implementation)          
        else:
            val = y
            ## only update the int_val when the total value is not saturated 
            self.int_val = integral
        self.last_error = error

        return val
