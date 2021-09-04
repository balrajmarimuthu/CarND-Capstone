import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass,fuel_capacity,brake_deadband,decel_limit,accel_limit,wheel_radius,\
                 wheel_base,steer_ratio,max_lat_accel,max_steer_angle):        
        #Get all the parameters relevant to calculate throttle, brake and steering
        self.vehicle_mass=vehicle_mass
        self.wheel_radius=wheel_radius
        self.steer_ratio = steer_ratio
        min_velocity     = 0.1
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.deadband    = brake_deadband
        
        # Yaw controller for steering
        self.yawController = YawController(wheel_base, self.steer_ratio, min_velocity, max_lat_accel, max_steer_angle)
        # PID controller for throttle; Kp = 0.85, Ki = 0.0001, Kd = 0.01
        self.throttle_controller = PID(0.85, 0.0001, 0.01, self.decel_limit, self.accel_limit)
        # Low pass filters for avoiding peak changes
        self.velocity_lowpassFilt = LowPassFilter(0.5, 0.02)
        self.throttle_lowpassFilt = LowPassFilter(0.07, 0.02)
        self.brake_lowpassFilt = LowPassFilter(0.08, 0.02)
        self.steer_lowpassFilt = LowPassFilter(0.005, 0.02)
        
        self.last_time = rospy.get_time()

    def control(self, linear_velocity_current, dbw_enabled, linear_velocity_target, angular_velocity_target, angular_velocity_current):
        
        #Check whether dbw is enabled. In simulator it is always enabled
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0,0.0,0.0

        #Identify brake and throttle
        brake = 0.
        # Use PID controller to identify throttle based on the difference in velocity        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(linear_velocity_target-linear_velocity_current, sample_time)
        # Avoid peak changes
        throttle = self.throttle_lowpassFilt.filt(throttle)
        # If the target velocity is less than the current, then brake and make the throttle as zero
        if (throttle < 0.1) and (linear_velocity_target < linear_velocity_current):
            throttle = 0
            brake = min((linear_velocity_current - linear_velocity_target),abs(self.decel_limit)) * self.vehicle_mass * self.wheel_radius
            # Avoid peak changes
            brake = self.brake_lowpassFilt.filt(brake)

        # Come to a complete stop and hold the brake
        if (linear_velocity_target == 0.0) and (linear_velocity_current <= 0.1):
            # Brake Nm to hold the car
            brake = 700
            throttle = 0.

        # Use yawController for steering. Avoid wheel turns when stopping. 
        if linear_velocity_current > 0.05:
            #Avoid peak changes in angular target velocity
            angular_velocity_target = self.steer_lowpassFilt.filt(angular_velocity_target)

            # Identify steering 
            steering = self.yawController.get_steering(linear_velocity_target, angular_velocity_target, linear_velocity_current)
            #Let the vehicle coast when sharp turns
            if (max(angular_velocity_target, angular_velocity_current) - min(angular_velocity_target, angular_velocity_current)) > 0.5:
                throttle = 0
                #brake = 25
            
        else:
            steering = 0.
        return throttle, brake, steering
