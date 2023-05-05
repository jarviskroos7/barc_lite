#!/usr/bin python3

# Your import statements here
import numpy as np
import os
from scipy import interpolate
from mpclab_common.pytypes import VehicleState, VehiclePrediction
from mpclab_common.track import get_track
from mpclab_controllers.abstract_controller import AbstractController

class PIController:
    
    # Simple PI controller

    def __init__(self, kp, ki, dt):
        self.kp = kp
        self.ki = ki
        self.integral = 0.0
        self.error_prev = 0.0
        self.dt = dt # 10hz
        
    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.error_prev) / self.dt # for PID
        output = self.kp * error + self.ki * self.integral
        self.error_prev = error
        return output

# The ProjectController class will be instantiated when creating the ROS node.
class ProjectController(AbstractController):
    def __init__(self, dt: float, print_method=print):
        # The control interval is set at 10 Hz
        self.dt = dt

        # If printing to terminal, use self.print_method('some string').
        # The ROS print method will be passed in when instantiating the class
        if print_method is None:
            self.print_method = lambda s: None
        else:
            self.print_method = print_method

        # The state and input prediction object
        self.state_input_prediction = VehiclePrediction()

        # The state and input reference object
        self.state_input_reference = VehiclePrediction()

        # Load the track used in the MPC Lab. The functions for transforming between
        # Global (x, y, psi) and Frenet (s, e_y, e_psi) frames is contained in the returned
        # object. i.e. global_to_local and local_to_global
        self.track = get_track('L_track_barc')
        self.L = self.track.track_length
        self.W = self.track.track_width

        # Example for obtaining the x-y points of the track boundaries
        track_xy = self.track.get_track_xy()
        bound_in_xy = track_xy['bound_in']
        bound_out_xy = track_xy['bound_out'] 

        # Convert x-y points to frenet frame
        bound_in_sey = []
        for _x, _y in zip(bound_in_xy['x'], bound_in_xy['y']):
            _s, _ey, _, = self.track.global_to_local((_x, _y, 0))
            bound_in_sey.append([_s, _ey])

        bound_out_sey = []
        for _x, _y in zip(bound_out_xy['x'], bound_out_xy['y']):
            _s, _ey, _, = self.track.global_to_local((_x, _y, 0))
            bound_out_sey.append([_s, _ey])

        # Loading raceline
        self.raceline = np.load('project_code/raceline_tt=0.1.npz', allow_pickle=True)
        self.s_ref = self.raceline['s']
        self.e_y_ref = self.raceline['e_y']
        self.e_psi_ref = self.raceline['e_psi']
        self.func_e_y = interpolate.interp1d(self.s_ref, self.e_y_ref)
        self.func_e_psi = interpolate.interp1d(self.s_ref, self.e_psi_ref)

        self.v_long_ref = self.raceline['v_long']
        self.func_v_long = interpolate.interp1d(self.s_ref, self.v_long_ref)

        # Initialize controller
        self.accel_controller = PIController(kp=1, ki=0.01, dt=self.dt)
        self.steer_controller = PIController(kp=0.8, ki=0.05, dt=self.dt)

    # This method will be called upon starting the control loop
    def initialize(self, vehicle_state: VehicleState):
        self.t0 = vehicle_state.t

    # This method will be called once every time step, make sure to modify the vehicle_state
    # object in place with your computed control actions for acceleration (m/s^2) and steering (rad)
    def step(self, vehicle_state: VehicleState):
        
        # Example transformation from global to Frenet frame coordinates
        s, e_y, e_psi = self.track.global_to_local((vehicle_state.x.x, vehicle_state.x.y, vehicle_state.e.psi))

        # interpolate by s to get the corresponding ey and epsi values on the track
        interp_e_y = self.func_e_y(s)
        interp_e_psi = self.func_e_psi(s)
        interp_v_long = self.func_v_long(s)

        t = vehicle_state.t - self.t0

        # Modify the vehicle state object in place to pass control inputs to the ROS node
        
        # simple P controller
        # definition of vehicle_state
        # https://github.com/MPC-Berkeley/barc_lite/blob/8260d93c1922d0b01537ada339514e1fee795b6d/workspace/src/mpclab_common/mpclab_common/lib/mpclab_common/pytypes.py#L300

        # accel = -0.1 * (vehicle_state.v.v_long - 4)
        # accel = -1 * (vehicle_state.v.v_long - 3)
        # accel = -1 * (vehicle_state.v.v_long - 0.9 * interp_v_long)
        # steer = -1 * ((e_y - interp_e_y) + (e_psi - interp_e_psi))

        accel = self.accel_controller.update(0.7 * interp_v_long - vehicle_state.v.v_long)
        steer = self.steer_controller.update((interp_e_y - e_y) + 1.1 * (interp_e_psi - e_psi))

        # acceleration and steering angle bound
        accel = max(min(accel, 1), -1)
        steer = max(min(steer, 0.43), -0.43)

        vehicle_state.u.u_a = accel
        vehicle_state.u.u_steer = steer

        # Example of printing
        self.print_method(f's: {round(s, 3)} | e_y: {round(e_y, 3)} | e_psi: {round(e_psi, 3)}')
        self.print_method(f'Accel: {round(accel, 3)} | Steering: {round(steer, 3)}')

        return

    # This method will be called once every time step. If you would like to visualize
    # the predictions made by your controller, make sure to populate the state_input_prediction
    # object
    def get_prediction(self):
        return self.state_input_prediction

    # This method will be called once every time step. If you would like to visualize
    # some user defined reference, make sure to populate the state_input_reference
    # object
    def get_reference(self):
        return self.state_input_reference