#!/usr/bin/env python3

# this file will be used to run the experiments and control the simulation via gazebo_simulation

import rclpy
from rclpy.node import Node

from gazebo_simulation import GazeboSimulation

class BARNRunner(Node):
    def __init__(self):
        super().__init__('BARN_Runner')
        


def main():
    rclpy.init()
    node = GazeboResetClient()
    node.reset()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
