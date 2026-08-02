from dataclasses import dataclass


@dataclass
class StepResult:
    command: float
    integral: float
    state: str


class ControllerModel:
    def __init__(self):
        self.kp = 100.0
        self.ki = 1.0
        self.kd = 5.0
        self.setpoint = 2.0
        self.integral_limit = 100.0
        self.command_limit = 255.0
        self.minimum_pwm = 40.0
        self.deadband = 0.30
        self.fault_angle = 15.0
        self.integral = 0.0
        self.previous_angle = self.setpoint
        self.state = "BALANCING"

    def step(self, angle, dt):
        if not 0.001 <= dt <= 0.05:
            self.state = "FAULT"
            return StepResult(0.0, self.integral, self.state)

        error = self.setpoint - angle
        if abs(error) > self.fault_angle:
            self.state = "FAULT"
            return StepResult(0.0, self.integral, self.state)
        if self.state == "FAULT":
            return StepResult(0.0, self.integral, self.state)

        derivative = -(angle - self.previous_angle) / dt
        candidate_integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral + error * dt),
        )
        unsaturated = self.kp * error + self.ki * candidate_integral + self.kd * derivative
        saturated = max(-self.command_limit, min(self.command_limit, unsaturated))

        if unsaturated == saturated or error * unsaturated < 0:
            self.integral = candidate_integral

        command = self.kp * error + self.ki * self.integral + self.kd * derivative
        command = max(-self.command_limit, min(self.command_limit, command))
        if abs(error) <= self.deadband:
            command = 0.0
        elif abs(command) < self.minimum_pwm:
            command = self.minimum_pwm if command > 0 else -self.minimum_pwm

        self.previous_angle = angle
        return StepResult(command, self.integral, self.state)
