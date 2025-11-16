import math
import numpy as np
from sdc_course.utils.utility import *


class PurePursuitLateralController:
    """
    PurePursuitLateralController implements lateral control using the pure pursuit controller.
    :L: Length of the vehicle
    :min_lookahead: Minimum lookahead distance
    :K_pp: Tuning parameter
    """

    def __init__(self, vehicle, L, min_lookahead, K_pp):
        self._vehicle = vehicle
        self._L = L
        self._min_lookahead = min_lookahead
        self._k_pp = K_pp

    def run_step(self, waypoints):
        return self._pure_pursuit_control(waypoints, self._vehicle.get_transform())

    def _get_goal_waypoint_index(self, vehicle, waypoints, lookahead_dist):
        for i in range(len(waypoints)):
            dist = compute_distance_to_waypoint(vehicle, waypoints[i])
            if dist >= lookahead_dist:
                return max(0, i)
        return len(waypoints) - 1

    def _get_steering_direction(self, v1, v2):
        """
        Note that Carla uses a left hand coordinate system, this is why a positive
        cross product requires a negative steering direction.
        :param v1: vector between vehicle and waypoint
        :param v2: vector in direction of vehicle
        :return: steering direction
        """
        cross_prod = v1[0] * v2[1] - v1[1] * v2[0]
        if cross_prod >= 0:
            return -1
        return 1

    def _pure_pursuit_control(self, waypoints, vehicle_transform):
        """
        :param waypoint: list of waypoints
        :param vehicle_transform: current transform of the vehicle
        :return: steering control
        """
        steering = 0.0
        #######################################################################
        ################## TODO: IMPLEMENT PURE-PURSUIT CONTROL HERE ##########
        #######################################################################

        # Get the vehicle states
        vehicle_location = vehicle_transform.location
        vehicle_velocity = get_velocity_ms(self._vehicle)

        # Calculate dynamic lookahead-distance
        lookahead_dist = max(self._min_lookahead, self._k_pp * vehicle_velocity)

        # Find the goal waypoint at the lookahead distance
        goal_wp_index = self._get_goal_waypoint_index(self._vehicle, waypoints, lookahead_dist)
        goal_wp = waypoints[goal_wp_index]

        # Get vehicle heading and goal waypoint vector with norms
        forward_vec = vehicle_transform.get_forward_vector()  # carla's 3d vector
        vehicle_heading_vec = np.array([forward_vec.x, forward_vec.y])  # 2d heading
        to_waypoint_vec = np.array([goal_wp[0] - vehicle_location.x,
                                   goal_wp[1] - vehicle_location.y])  # vector to waypoint
        
        norm_heading = np.linalg.norm(vehicle_heading_vec)
        norm_to_wp = np.linalg.norm(to_waypoint_vec) 

        # Calculate the angle alpha between vehicle heading and waypoint vector
        if norm_heading == 0 or norm_to_wp == 0:
            alpha = 0.0
        else:
            cos_alpha = np.dot(vehicle_heading_vec, to_waypoint_vec) / (norm_heading * norm_to_wp)
            cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
            alpha = np.arccos(cos_alpha)

        # Pure Pursuit steering control law
        # steering = atan2(2 * L * sin(alpha), lookahead_dist)
        steering = np.arctan2(2 * self._L * np.sin(alpha), lookahead_dist)

        # Apply steering direction
        steering_direction = self._get_steering_direction(to_waypoint_vec, vehicle_heading_vec) 
        steering *= steering_direction

        return steering
