#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
# from cslam_storage import CslamStorage
# from cslam_storage.map_recovery import MapRecovery

from cslam_common_interfaces.msg import PoseGraph
from cslam_common_interfaces.msg import PoseGraphValue
from cslam_common_interfaces.msg import PoseGraphEdge
from cslam_common_interfaces.msg import MultiRobotKey
from cslam_common_interfaces.srv._map_request import MapRequest
from geometry_msgs.msg import Pose

from std_msgs.msg import UInt32

class MapRecovery():
    def __init__(self, node, robot_id):
        self.node = node
        self.robot_id = robot_id
        self.robot_heartbeat_subscriber = self.node.create_subscription(
            UInt32, 
            '/r' + str(robot_id) + '/cslam/heartbeat', 
            self.heartbeat_received_callback, 
            10
        )
        self.pose_graph_publisher = self.node.create_publisher(
            PoseGraph, "/cslam/viz/pose_graph", 10)

        self.map_recovery_client = self.node.create_client(MapRequest, '/r' + str(robot_id) + '/publish_previous_map')       # CHANGE
        # while not self.map_recovery_client.wait_for_service(timeout_sec = 1.0):
        #     self.node.get_logger().info('service not available, waiting again...')
        self.map_recovery_req = MapRequest.Request()                                   # CHANGE
        
    def heartbeat_received_callback(self, msg):
        if (msg.data == 1):
            self.node.get_logger().info("Getting data from robot " + str(self.robot_id)) 
            self.future = self.map_recovery_client.call_async(self.map_recovery_req)

    def retrieve_pose_graph(self):
        """ Read pose graph from json file 
            Path is passed as parameter in the yaml file """
        pose_graph_path = self.params['map_path'] + "/" + self.params['pose_graph_file_name']

        # TODO: bug is path doesnt exist
        with open(pose_graph_path, 'r') as file:
            pose_graph_msg = PoseGraph()
            global_pose_graph = json.load(file)
            robot_pose_graph = global_pose_graph[str(self.robot_id)]
            values = []
            edges = []

            # Retrieve each cslam_common_interfaces/msg/PoseGraphValue
            for keyframe_id, pose_dict in robot_pose_graph["values"].items():
                keyframe_id_int = int(keyframe_id)
                values.append(self.dict_to_pose_graph_value(pose_dict, self.robot_id, keyframe_id_int))
            
            # Retrieve each cslam_common_interfaces/msg/PoseGraphEdge
            for edge_dict in robot_pose_graph["edges"]: 
                edges.append(self.dict_to_pose_graph_edge(edge_dict))

            pose_graph_msg.robot_id = self.robot_id
            pose_graph_msg.origin_robot_id = self.robot_id
            # TODO: affect this attribut !!!
            # pose_graph_msg.connected_robots 
            pose_graph_msg.values = values
            pose_graph_msg.edges = edges    
            self.pose_graph_publisher.publish(pose_graph_msg)

            # for robot_id, robot_pose_graph in global_pose_graph.items():
            #     robot_id_int = int(robot_id)
            #     self.origin_robot_ids[robot_id_int] = robot_id_int
                
            #     if robot_id_int not in self.robot_pose_graphs:
            #         self.robot_pose_graphs[robot_id_int] = {}

            #     # Retrieve each cslam_common_interfaces/msg/PoseGraphValue
            #     for keyframe_id, pose_dict in robot_pose_graph["values"].items():
            #         keyframe_id_int = int(keyframe_id)
            #         self.robot_pose_graphs[robot_id_int][keyframe_id_int] = self.dict_to_pose_graph_value(pose_dict, robot_id_int, keyframe_id_int)


            #     if robot_id_int not in self.robot_pose_graphs_edges:
            #         self.robot_pose_graphs_edges[robot_id_int] = []
                
    def dict_to_pose(self, dict):
        """Convert dict to geometry_msgs/msg/Pose""" 
        pose = Pose()
        pose.position.x = dict['position']['x']
        pose.position.y = dict['position']['y']
        pose.position.z = dict['position']['z']
        pose.orientation.x = dict['orientation']['x']
        pose.orientation.y = dict['orientation']['y']
        pose.orientation.z = dict['orientation']['z']
        pose.orientation.w = dict['orientation']['w']
        return pose
    
    def dict_to_pose_graph_value(self, dict, robot_id, keyframe_id):
        """ Convert dict to cslam_common_interfaces/msg/PoseGraphValue
            Attention: the "key" property is not converted
        """
        pose_graph_value = PoseGraphValue()
        pose_graph_value.key = MultiRobotKey()
        pose_graph_value.key.robot_id = robot_id
        pose_graph_value.key.keyframe_id = keyframe_id
        pose_graph_value.pose = self.dict_to_pose(dict)
        return pose_graph_value

    def dict_to_pose_graph_edge(self, dict):
        """ Convert dict to cslam_common_interfaces/msg/PoseGraphEdge """
        pose_graph_edge = PoseGraphEdge()
        pose_graph_edge.key_from = MultiRobotKey()
        pose_graph_edge.key_from.robot_id = int(dict["key_from"]["robot_id"])
        pose_graph_edge.key_from.keyframe_id = int(dict["key_from"]["keyframe_id"])
        pose_graph_edge.key_to = MultiRobotKey()
        pose_graph_edge.key_to.robot_id = int(dict["key_to"]["robot_id"])
        pose_graph_edge.key_to.keyframe_id = int(dict["key_to"]["keyframe_id"])
        pose_graph_edge.measurement = self.dict_to_pose(dict["measurement"])
        pose_graph_edge.noise_std = dict["noise_std"]                    
        return pose_graph_edge
            

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
            # pass
        
def main(args=None):
    rclpy.init(args=args)
    map_recovery_node = MapRecoveryNode()
    rclpy.spin(map_recovery_node)

    map_recovery_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()