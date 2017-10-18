from yaw_controller import YawController 
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, 
                       kp, ki, kd, mn, mx, max_brake_cmd ):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, 
                                            max_lat_accel, max_steer_angle )

        self.long_pid_controller = PID(kp, ki, kd, mn, mx)
        tau = 9
        ts  = 1 
        self.low_pass_fltr = LowPassFilter(tau, ts)
        self.max_brake_cmd = max_brake_cmd
        self.last_vel_time = None  

    def control(self, lin_tgt_vel, ang_tgt_vel, current_vel, current_vel_time, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yaw_controller.get_steering(lin_tgt_vel, ang_tgt_vel, current_vel)

        throttle, brake = self.longitudinal_control(lin_tgt_vel, current_vel, current_vel_time)

        if not dbw_enabled:
            throttle = 0
            brake    = 0
            steer    = 0
            self.long_pid_controller.reset()

        return throttle, brake, steer


    def longitudinal_control(self, lin_tgt_vel, current_vel, current_vel_time):
 
        if self.last_vel_time is None :
            sample_time    = 0.1
        else:
            sample_time    = current_vel_time - self.last_vel_time
        self.last_vel_time = current_vel_time

        sample_time        = max(0.001, sample_time)
        error              = lin_tgt_vel - current_vel 
        # pid
        pid_value          = self.long_pid_controller.step(error, sample_time)
        # filter pid output
        pid_value          = self.low_pass_fltr.filt(pid_value)

        return self.convert_pid_to_cmd(pid_value)


    def convert_pid_to_cmd(self, pid_value):

        if pid_value > 0 :
            throttle = pid_value
            brake    = 0
        else:
            throttle = 0
            brake    = - pid_value * self.max_brake_cmd

        return throttle, brake
