#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cslam_storage.map_recovery import MapRecovery

class MapRecoveryNode(Node):
    """
        This class allows us to interrogate the robots 
        if they have map updates to share. 
        
        This node is typically launched in the base station.
    """    
    def __init__(self):
        super().__init__('cslam_map_recovery')
        
        # Set parameters
        self.declare_parameters(
            namespace='',
            parameters=[('max_nb_robots', 2)],
        )
        self.max_nb_robots = self.get_parameter(
        'max_nb_robots').value
        
        self.map_recuperators = {}
        for robot_id in range(self.max_nb_robots):
                self.map_recuperators[robot_id] = MapRecovery(self, robot_id) 
        
def main(args=None):
    rclpy.init(args=args)
    map_recovery_node = MapRecoveryNode()
    rclpy.spin(map_recovery_node)

    map_recovery_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()