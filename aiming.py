from h_bridge import HBridge
import constants

class Aim(HBridge):
    def __init__(
        self,
        in1=constants.IN7,
        in2=constants.IN8,
        enable=constants.ENABLE_D,
        pwm_freq=constants.AIMING_MOTOR_FREQ,
        pwm_dc=constants.AIMING_MOTOR_DC
        ):
        super().__init__(in1, in2, enable, pwm_freq, pwm_dc)
    
    def track_player(self, pose_estimation):
        if pose_estimation == 'move left':
            self.forward()
        elif pose_estimation == 'move right':
            self.backward()
        else:
            self.stop()