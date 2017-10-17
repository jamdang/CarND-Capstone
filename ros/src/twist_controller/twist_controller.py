from yaw_controller import YawController 

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, 
                                            max_lat_accel, max_steer_angle)

    def control(self, lin_tgt_vel, ang_tgt_vel, current_vel,dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yaw_controller.get_steering(lin_tgt_vel, ang_tgt_vel, current_vel)
        return 1., 0., steer
