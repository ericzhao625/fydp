from h_bridge import HBridge


class Aim(HBridge):
    def __init__(self, in1, in2, enable, pwm_freq, pwm_dc):
        super().__init__(in1, in2, enable, pwm_freq, pwm_dc)
    
    def track_player(self, pose_estimation):
        if pose_estimation == 'move left':
            self.forward()
        elif pose_estimation == 'move right':
            self.backward()
        else:
            self.stop()