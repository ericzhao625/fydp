import time

import constants


class PIDController:
    """
    PID Controller for motor aiming control.

    Attributes:
        Kp (float): Proportional gain.
        Ki (float): Integral gain.
        Kd (float): Derivative gain.
        min_pwm (float): Minimum PWM value to turn throwing mechanism.
        max_pwm (float): Maximum PWM value to prevent overshooting.
        prev_error (float): Previous error value between expected and actual.
        last_time (time): Previous time value to determine time step.
    """

    def __init__(self):
        """
        Initializes the PID controller for motor aiming control.

        Args:
            Kp (float): Proportional gain.
            Ki (float): Integral gain.
            Kd (float): Derivative gain.
            min_pwm (float): Minimum PWM value to turn throwing mechanism.
            max_pwm (float): Maximum PWM value to prevent overshooting.
        """
        self.Kp = constants.KP
        self.Ki = constants.KI
        self.Kd = constants.KD
        self.min_pwm = constants.PID_MAX_PWM
        self.max_pwm = constants.PID_MIN_PWM

        self.prev_error = 0
        self.last_time = time.time()

    def compute(self, error):
        """
        Compute the PWM output based on the PID equation.

        Args:
            error (float): The angle deviation from the target.

        Returns:
            pwm (int): The calculated PWM value.

        Raises:
            Exception: If error is None
        """
        try:
            current_time = time.time()
            dt = current_time - self.last_time

            if dt == 0:
                return 0  # Prevent division by zero

            # Proportional term
            P = self.Kp * error

            # Integral term (accumulates error over time)
            integral += error * dt
            I = self.Ki * integral

            # Derivative term (rate of change of error)
            derivative = (error - self.prev_error) / dt
            D = self.Kd * derivative

            # Compute total output
            pwm = P + I + D

            # Constrain PWM output
            pwm = max(self.min_pwm, min(self.max_pwm, abs(pwm)))

            # Update previous values
            self.prev_error = error
            self.last_time = current_time

            return pwm

        except Exception as e:
            print(f'Unexpected error in PID controller: {e}')
        return None
