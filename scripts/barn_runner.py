#!/usr/bin/env python3

# this file will be used to run the experiments and control the simulation via gazebo_simulation

import rclpy
from rclpy.node import Node

from gazebo_simulation import GazeboSimulation

class BARNRunner(Node):
    def __init__(self, init_position = [0, 0, 0]):
        super().__init__('BARN Runner')
        
        # setup service clients
        self.set_entity_state_client = self.create_client(
            SetEntityPose,
            '/world/default/set_entity_pose'   # change world name if needed
        )
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_entity_pose service...')


    #     self.get_entity_state_client = self.create_client(
    #         GetEntityState,
    #         '/gzserver/get_entity_state'
    #     )

    #     self.set_entity_state_client = self.create_client(
    #         SetEntityState,
    #         '/gzserver/set_entity_state'
    #     )

    #     while not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('/gzserver/get_entity_state service not available, waiting again...')

    #     while not self.set_simulation_state_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('/gzserver/set_simulation_state service not available, waiting again...')

    #     # initialize EntityState 
        self._init_model_state = create_model_entity_pose(init_position[0], init_position[1], 0, init_position[2])

    #     # initialize collision counter and relevant code
    #     self.collision_count = 0
    #     self.create_subscription(Bool, '/collision', self._collision_cb, 10)

    #     # initialize subscriber for laser_scan
    #     self.create_subscription(LaserScan, '/front/scan', self._scan_cb, 10)
    #     self.latest_scan = None

    # def _collision_cb(self, msg):
    #     if msg.data:
    #         self.collision_count += 1 

    # def _scan_cb(self, msg):
    #     self.latest_scan = msg

    # def get_hard_collision(self):
    #     # hard collision count since last call
    #     collided = self.collision_count > 0
    #     self.collision_count = 0
    #     return collided

    # def get_laser_scan(self):
    #     while self.latest_scan is None:
    #         rclpy.spin_once(self, timeout_sec=0.1)
    #     return self.latest_scan

    # def pause(self):
    #     req = self.set_simulation_state_client.Request()
    #     req.state = SimulationState()
    #     req.state.state = SimulationState.STATE_PAUSED 
    #     future = self.set_simulation_state_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result()
        
    # def unpause(self):
    #     req = self.set_simulation_state_client.Request()
    #     req.state = SimulationState()
    #     req.state.state = SimulationState.STATE_PLAYING 
    #     future = self.set_simulation_state_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result()


    def reset_model(self):
        """
        This resets the model. In ROS1 BARN, this was reset() which is ill-named as reset() means resetting the world and not just the model's pose. 
        """
        req = SetEntityPose.Request()
        req.entity = self._init_model_state[0]
        req.pose = self._init_model_state[1]
        future = self.set_entity_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    # def get_model_state(self):
    #     req = self.get_entity_state_client.Request()
    #     req.entity = "jackal"
    #     future = self.get_entity_state_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result()
    
    # def reset_init_model_state(self, init_position = [0, 0, 0]):
    #     """
    #     Overwrite the initial model state
    #     Args:
    #         init_position (list, optional): initial model state in x, y, z. Defaults to [0, 0, 0].
    #     """
    #     self._init_model_state = create_model_state(init_position[0],init_position[1],0,init_position[2])




def main():
    rclpy.init()
    node = GazeboResetClient()
    node.reset()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
