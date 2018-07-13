import math
import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(
            self,
            vehicle_mass,
            fuel_capacity,
            brake_deadband,
            decel_limit,
            accel_limit,
            wheel_radius,
            wheel_base,
            steer_ratio,
            max_lat_accel,
            max_steer_angle,
            max_throttle):
        # Yaw controller to controller the turning
        self.vehicle_min_velocity = .1
        self.yaw_controller = YawController(
            wheel_base=wheel_base,
            steer_ratio=steer_ratio,
            min_speed=self.vehicle_min_velocity,
            max_lat_accel=max_lat_accel,
            max_steer_angle=max_steer_angle)

        self.steering_controller = PID(
            kp=.0, ki=.001, kd=.1, min=-max_steer_angle, max=max_steer_angle)

        # Throttle controller
        self.throttle_controller = PID(
            kp=.8, ki=.001, kd=.1, min=0, max=max_throttle)

        # Remove high frequency noise on the velocity
        self.low_pass = LowPassFilter(tau=.5, ts=.02)

        # Assume the tank is full
        fuel_mass = fuel_capacity * GAS_DENSITY
        self.total_mass = vehicle_mass + fuel_mass
        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(
            self,
            current_velocity,
            dbw_enabled,
            linear_velocity,
            angular_velocity,
            cte):
        if not dbw_enabled:
            self.throttle_controller.reset()
            self.steering_controller.reset()
            return .0, .0, .0

        current_velocity = self.low_pass.filter(current_velocity)

        # Calculate the velocity error
        dv = linear_velocity - current_velocity

        # Update time
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Calculate the predictive path and correct for the cross track error
        steering = self.yaw_controller.get_steering(
            linear_velocity,
            angular_velocity,
            current_velocity)
        steering -= self.steering_controller.step(cte, dt)

        # Calculate the throttle and brake
        throttle = self.throttle_controller.step(dv, dt)
        brake = 0.

        if linear_velocity < 0. and current_velocity < self.vehicle_min_velocity:
            # Keep car in place when it is stopped
            throttle = 0.
            brake = 400.
        elif throttle < .1 and dv < 0.:
            throttle = 0.
            decel = max(dv, self.decel_limit)
            brake = abs(decel) * self.total_mass * self.wheel_radius

        return throttle, brake, steering
