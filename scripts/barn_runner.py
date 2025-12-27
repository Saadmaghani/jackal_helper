#!/usr/bin/env python3

# this file will be used to run the experiments and control the simulation via gazebo_simulation

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
# from rclpy.executors import SingleThreadedExecutor --> cant make executors inside nodes. meant to be outside nodes.
from ament_index_python.packages import get_package_share_directory

from gazebo_simulation import GazeboSimulation
import time
import os
from os.path import dirname
import threading
import numpy as np


INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 10]  # relative to the initial position
TIMEOUT = 100

def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def path_coord_to_gazebo_coord(x, y):
        RADIUS = 0.075
        r_shift = -RADIUS - (30 * RADIUS * 2)
        c_shift = RADIUS + 5

        gazebo_x = x * (RADIUS * 2) + r_shift
        gazebo_y = y * (RADIUS * 2) + c_shift

        return (gazebo_x, gazebo_y)


class BARNRunner(Node):
    def __init__(self):
        super().__init__('BARN_Runner')
        self.get_logger().info("BARN RUNNER started")

        # declare parameters
        self._declare_parameters()

        self.gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)

        self.init_coor = (INIT_POSITION[0], INIT_POSITION[1])
        self.goal_coor = (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1])
        self.timeout = TIMEOUT

        self.file_lock = threading.Lock()

    def reset_jackal_to_start(self):
        self.get_logger().info("reset_jackal_to_start started")
        
        pos = self.gazebo_sim.get_model_state().position
        curr_coor = (pos.x, pos.y)
        collided = True

        self.get_logger().info(f"initial position: ({curr_coor[0]}, {curr_coor[1]})")
        self.get_logger().info(f"goal position: ({self.goal_coor[0]}, {self.goal_coor[1]})")

        while compute_distance(self.init_coor, curr_coor) > 0.1 or collided:
            rclpy.spin_until_future_complete(self, self.gazebo_sim.reset_model())
            pos = self.gazebo_sim.get_model_state().position
            curr_coor = (pos.x, pos.y)
            collided = self.gazebo_sim.get_hard_collision()

    def run_trial(self):
        self.get_logger().info(f"Trial running...")
        curr_time = self.get_clock().now()
        pos = self.gazebo_sim.get_model_state().position
        curr_coor = (pos.x, pos.y)


        self.get_logger().info(f"Checking if robot moved...")
        # check whether the robot started to move
        while compute_distance(self.init_coor, curr_coor) < 0.1:
            # self.get_logger().info(f"x: {curr_coor[0]:.2f} (m), y: {curr_coor[1]:.2f} (m)")
            curr_time = self.get_clock().now()
            pos = self.gazebo_sim.get_model_state().position
            curr_coor = (pos.x, pos.y)
            rclpy.spin_once(self.gazebo_sim, timeout_sec=0.01) # spin gazebo_sim node instead of self so gazebo_sim gets updated. or TODO use executor to spin both.
        self.get_logger().info(f"Robot moved!")

        # start navigation, check position, time and collision
        start_time = curr_time
        start_time_cpu = time.time()
        collided = False
        elapsed = (curr_time - start_time).nanoseconds/1e9

        while compute_distance(self.goal_coor, curr_coor) > 1 and not collided and elapsed < self.timeout:
            curr_time = self.get_clock().now()
            pos = self.gazebo_sim.get_model_state().position
            curr_coor = (pos.x, pos.y)
            elapsed = (curr_time - start_time).nanoseconds/1e9
            print(f"Time: {elapsed:.2f} (s), x: {curr_coor[0]:.2f} (m), y: {curr_coor[1]:.2f} (m)", end='\r')
            collided = self.gazebo_sim.get_hard_collision()
            rclpy.spin_once(self.gazebo_sim, timeout_sec=0.01)
        print() # so that we can keep the last print statement. 

        self.get_logger().info(f"Trial end! Pausing gazebo.")
        self.gazebo_sim.pause()

        self.trial_elapsed_time = elapsed
        self.trial_success = False
        if collided:
            self.trial_status = "collided"
        elif elapsed >= self.timeout:
            self.trial_status = "timeout"
        else:
            self.trial_status = "succeeded"
            self.trial_success = True
        
        self.get_logger().info(f"Navigation {self.trial_status} with time {elapsed:.4f} (s).")
    
    def trial_cleanup(self):
        # report metric and generate log
        
        world_idx = self.get_parameter('world_idx').value
        out_file = self.get_parameter('out_file').value

        if world_idx >= 300:
            path_length = self.goal_coor[0] - self.init_coor[0]
        else:
            path_file_name = os.path.join(get_package_share_directory("jackal_helper"), "worlds/BARN/path_files", f"path_{world_idx}.npy")
            path_array = np.load(path_file_name)
            path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
            path_array = np.insert(path_array, 0, (self.init_coor[0], self.init_coor[1]), axis=0)
            path_array = np.insert(path_array, len(path_array), (self.init_coor[0] + self.goal_coor[0], self.init_coor[1] + self.goal_coor[1]), axis=0)
            path_length = 0
            for p1, p2 in zip(path_array[:-1], path_array[1:]):
                path_length += compute_distance(p1, p2)
        
        # Navigation metric: 1_success *  optimal_time / clip(actual_time, 2 * optimal_time, 8 * optimal_time)
        optimal_time = path_length / 2
        nav_metric = int(self.trial_success) * optimal_time / np.clip(self.trial_elapsed_time, 2 * optimal_time, 8 * optimal_time)
        self.get_logger().info(f"Navigation metric: {nav_metric:.4f}")
        
        out_path = os.path.join(self._get_pkg_src_path(), "res")
        os.makedirs(out_path, exist_ok=True)
        out_path = os.path.join(out_path, out_file) 
        
        collided = self.trial_status == "collided"
        timeout = self.trial_status == "timeout"

        with self.file_lock:
            with open(out_path, "a") as f:
                f.write(
                    f"{world_idx} {self.trial_success} {collided} {timeout} {self.trial_elapsed_time:.4f} {nav_metric:.4f}\n"
                )
    
    def _declare_parameters(self):
        self.declare_parameter('world_idx')
        self.declare_parameter('out_file', 'out.txt')

        pass

    # hack to get src/jackal_helper
    # TODO: alternatively we can set this as a parameter
    def _get_pkg_src_path(self):
        workspace_path = dirname(dirname(dirname(get_package_share_directory("jackal_helper"))))
        jackal_helper_src_path = os.path.join(workspace_path, "src", "jackal_helper")
        return jackal_helper_src_path


def main():
    rclpy.init()
    node = BARNRunner()
    node.reset_jackal_to_start()
    node.run_trial()
    node.trial_cleanup()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
