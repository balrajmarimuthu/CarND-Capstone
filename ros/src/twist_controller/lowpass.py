
class LowPassFilter(object):
    def __init__(self, tau, ts):
        self.a = 1. / (tau / ts + 1.)
        self.b = tau / ts / (tau / ts + 1.)
        # Variable to store prrevious value for all motion commands
        self.last_val = 0.
        self.ready = False
    
    # Get the last_val for that motion command
    def get(self, flag):
        if flag == 0:
            self.last_val = self.last_val_throttle
        elif flag == 1:
            self.last_val = self.last_val_brake
        else:
            self.last_val = self.last_val_steer
        return self.last_val
    
    
    # Set the last_val for that motion command
    def set_val(self, last_val, flag):
        self.last_val = last_val
        if flag == 0:
            self.last_val_throttle = self.last_val
        elif flag == 1:
            self.last_val_brake = self.last_val
        else:
            self.last_val_steer = self.last_val

    # Filter to give a resulting value based on the current and previous value
    def filt(self, val):
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val
