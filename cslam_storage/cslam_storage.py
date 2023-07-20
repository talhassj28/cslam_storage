#!/usr/bin/env python3
import rclpy
import os.path
import json
import cslam.lidar_pr.icp_utils as icp_utils
import open3d
import threading
import numpy as np

from rclpy.node import Node
from cslam_common_interfaces.msg import PoseGraph
from cslam_common_interfaces.msg import PoseGraphValue
from cslam_common_interfaces.msg import PoseGraphEdge
from cslam_common_interfaces.msg import MultiRobotKey
from cslam_common_interfaces.msg import VizPointCloud
# TODO: find why this file takes this name and change it (_map_request) 
from cslam_common_interfaces.srv._map_request import MapRequest
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import UInt32
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from tf2_ros import TransformBroadcaster

from cslam_visualization.utils.transform import Transform

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

        self.map_recovery_client = self.node.create_client(MapRequest, '/r' + str(robot_id) + '/publish_previous_map')
        self.map_recovery_req = MapRequest.Request()
        
    def heartbeat_received_callback(self, msg):
        if (msg.data == 1):
            self.future = self.map_recovery_client.call_async(self.map_recovery_req)

class CslamStorage(Node):

    def __init__(self):
        super().__init__('cslam_storage')
        
        # Publisher and subscribers
        self.pose_graph_storage_subscriber = self.create_subscription(
                PoseGraph, '/cslam/viz/pose_graph', self.pose_graph_storage_callback, 10)
        self.pointclouds_storage_subscriber = self.create_subscription(
                VizPointCloud, '/cslam/viz/keyframe_pointcloud', self.point_clouds_storage_callback, 10)
        self.pose_graph_publisher = self.create_publisher(
            PoseGraph, "/cslam/viz/pose_graph", 10)
        self.pointclouds_publisher = self.create_publisher(
            VizPointCloud, '/cslam/viz/keyframe_pointcloud', 10)
        self.pointclouds2_publisher = self.create_publisher(
            PointCloud2, 'pointcloud', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)

        # Services
        self.publish_previous_map = self.create_service(MapRequest, 'publish_previous_map', self.publish_previous_map_callback)
        self.pose_graph_to_store = {}

        # Set parameters
        self.declare_parameters(
            namespace='',
            parameters=[# TODO: test if this default value works
                        # TODO: raise exception if map_path is not given when enable_map_storage or 
                        # enable_map_reading are true, or use a default path 
                        ('map_path', ''),
                        ('pose_graph_file_name', 'pose_graph.json'), 
                        ('is_robot', False),
                        ('max_nb_robots', 2),
                        ('enable_own_storage', False),])
        
        self.map_path = self.get_parameter(
        'map_path').value
        self.pose_graph_file_name = self.get_parameter(
            'pose_graph_file_name').value
        self.is_robot = self.get_parameter(
            'is_robot').value
        self.max_nb_robots = self.get_parameter(
        'max_nb_robots').value
        self.enable_own_storage = self.get_parameter(
        'enable_own_storage').value
        
        # Define map recovery clients (we create a service client for each robot) 
        self.map_recovery_clients = {} 
        if not self.is_robot:
            for robot_id in range(self.max_nb_robots):
                self.map_recovery_clients[robot_id] = MapRecovery(self, robot_id) 

        if self.enable_own_storage and os.path.exists(self.map_path + '/' + self.pose_graph_file_name):
            delay = 2  # in seconds
            # pose_graph_timer = threading.Timer(delay, self.retrieve_pose_graph)
            # point_clouds_timer = threading.Timer(delay, self.retrieve_point_cloud_keyframes)
            # pose_graph_timer.start()
            # point_clouds_timer.start()
            # point_clouds2_timer = threading.Timer(delay, self.publish_pcd)
            # point_clouds2_timer.start()
            self.get_logger().info("*************** Displaying map from current computer ***************") 

    # Conversion methods
    def pose_graph_value_to_dict(self, pose_graph_value):
        """ Convert cslam_common_interfaces/msg/PoseGraphValue to dict
            Attention: the "key" property is not converted 
            TODO: maybe think about add key property
        """
        return {
            "position": {
                "x": pose_graph_value.pose.position.x,
                "y": pose_graph_value.pose.position.y,
                "z": pose_graph_value.pose.position.z
            },
            "orientation": {
                "x": pose_graph_value.pose.orientation.x,
                "y": pose_graph_value.pose.orientation.y,
                "z": pose_graph_value.pose.orientation.z,
                "w": pose_graph_value.pose.orientation.w
            }
        }
    
    def pose_graph_edge_to_dict(self, edge):
        """Convert cslam_common_interfaces/msg/PoseGraphEdge to dict""" 
        return {
            "key_from": {
                "robot_id": edge.key_from.robot_id,
                "keyframe_id": edge.key_from.keyframe_id
            },
            "key_to": {
                "robot_id": edge.key_to.robot_id,
                "keyframe_id": edge.key_to.keyframe_id
            },
            "measurement": {
                "position": {
                    "x": edge.measurement.position.x,
                    "y": edge.measurement.position.y,
                    "z": edge.measurement.position.z
                },
                "orientation": {
                    "x": edge.measurement.orientation.x,
                    "y": edge.measurement.orientation.y,
                    "z": edge.measurement.orientation.z,
                    "w": edge.measurement.orientation.w,
                },
            },
            "noise_std": edge.noise_std.tolist()
        }

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
    
    # Methods to retrieve and store the map in files 
    def retrieve_pose_graph(self):
        """ Read pose graph from json file 
            Path is passed as parameter in the yaml file """
        pose_graph_path = self.map_path + "/" + self.pose_graph_file_name
        
        if not os.path.exists(pose_graph_path):
            return 
        
        with open(pose_graph_path, 'r') as file:
            pose_graph_msg = PoseGraph()
            global_pose_graph = json.load(file)
            
            for robot_id, robot_pose_graph in global_pose_graph.items():
                robot_id_int = int(robot_id)
                values = []
                edges = []

                # Retrieve each cslam_common_interfaces/msg/PoseGraphValue
                for keyframe_id, pose_dict in robot_pose_graph["values"].items():
                    keyframe_id_int = int(keyframe_id)
                    values.append(self.dict_to_pose_graph_value(pose_dict, robot_id_int, keyframe_id_int))
                
                # Retrieve each cslam_common_interfaces/msg/PoseGraphEdge
                for edge_dict in robot_pose_graph["edges"]: 
                    edges.append(self.dict_to_pose_graph_edge(edge_dict))

                pose_graph_msg.robot_id = robot_id_int
                pose_graph_msg.origin_robot_id = robot_id_int
                # TODO: affect this attribut !!!
                # pose_graph_msg.connected_robots 
                pose_graph_msg.values = values
                pose_graph_msg.edges = edges    
                self.pose_graph_publisher.publish(pose_graph_msg)

    def retrieve_point_cloud_keyframes(self):
        pose_graph_path = self.map_path + "/" + self.pose_graph_file_name

        if not os.path.exists(pose_graph_path):
            return 
        
        with open(pose_graph_path, 'r') as file:
            global_pose_graph = json.load(file)

            for robot_id, robot_pose_graph in global_pose_graph.items():
                point_cloud_keyframes_folder = self.map_path + '/robot' + robot_id
                for keyframe_id, pose_graph_value in robot_pose_graph["values"].items():
                    point_cloud_keyframe_path = point_cloud_keyframes_folder + '/keyframe_' + keyframe_id + '.pcd'
                    
                    if (os.path.exists(point_cloud_keyframe_path)):
                        pcd = open3d.io.read_point_cloud(point_cloud_keyframe_path)
                        point_cloud = icp_utils.open3d_to_ros(pcd)
                        viz_point_cloud = VizPointCloud()
                        viz_point_cloud.robot_id = int(robot_id)
                        viz_point_cloud.keyframe_id = int(keyframe_id)
                        viz_point_cloud.pointcloud = point_cloud
                        self.pointclouds_publisher.publish(viz_point_cloud)

    def store_pose_graph(self):
        # Make sure that intermediate directories exist
        os.makedirs(self.map_path, exist_ok=True)

        pose_graph_path = self.map_path + "/" + self.pose_graph_file_name
        with open(pose_graph_path, "w+") as json_file:
            json.dump(self.pose_graph_to_store, json_file)
    
    # Subscriber callbacks  
    def pose_graph_storage_callback(self, msg):    
        # Initialize robot pose graph if it doesn't exist yet
        if msg.robot_id not in self.pose_graph_to_store:
            self.pose_graph_to_store[msg.robot_id] = {
                "edges": {},
                "values": {}
            }

        # Convert PoseGraphValue and PoseGraphEdge to dict (json) to be stored 
        for pose in msg.values:
            self.pose_graph_to_store[msg.robot_id]["values"][pose.key.keyframe_id] = self.pose_graph_value_to_dict(pose)
        self.pose_graph_to_store[msg.robot_id]["edges"] = list(map(self.pose_graph_edge_to_dict, msg.edges))

        # Make sure to not store a empty json pose graph
        if (len(msg.values) != 0) and (len(msg.edges) != 0): 
            self.store_pose_graph()

    def point_clouds_storage_callback(self, pc_msg):
        """Store point cloud data into a given .pcd file 

        Args:
            pc_msg (VizPointCloud): point cloud message 
        """       
        robot_folder = self.map_path + "/robot" + str(pc_msg.robot_id)
        os.makedirs(robot_folder, exist_ok=True)        
        pcd_file_path = robot_folder + "/keyframe_" + str(pc_msg.keyframe_id) + ".pcd"
        
        point_cloud = icp_utils.ros_to_open3d(pc_msg.pointcloud)
        open3d.io.write_point_cloud(pcd_file_path, point_cloud)

    def publish_previous_map_callback(self, request, response):
        # self.get_logger().info('PUBLISH PREVIOOUS MAP WITH SERVICE')
            
        self.retrieve_pose_graph()
        self.retrieve_point_cloud_keyframes()
        response.publishing_map = True
        return response
    
    def pose_to_transform(self, pose):
        quat = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        pos = [pose.position.x, pose.position.y, pose.position.z]
        return Transform(quat=quat, pos=pos)
    
    def publish_pcd(self):
        pose_graph_path = self.map_path + "/" + self.pose_graph_file_name

        tfs_to_publish = []
        point_cloud_msgs = []
        
        if not os.path.exists(pose_graph_path):
            return 
        
        with open(pose_graph_path, 'r') as file:
            # pose_graph_msg = PoseGraph()
            global_pose_graph = json.load(file)
            
            for robot_id, robot_pose_graph in global_pose_graph.items():
                robot_id_int = int(robot_id)
                # values = []
                # edges = []

                # Retrieve each cslam_common_interfaces/msg/PoseGraphValue
                for keyframe_id, pose_dict in robot_pose_graph["values"].items():
                    pose_value = self.dict_to_pose_graph_value(pose_dict, robot_id_int, int(keyframe_id))

                    tf_to_publish = TransformStamped()
                    tf_to_publish.header.frame_id = "base_link"
                    tf_to_publish.header.stamp = rclpy.time.Time().to_msg()
                    tf_to_publish.child_frame_id = "robot1_" + 'keyframe_' + keyframe_id
                    t = self.pose_to_transform(pose_value.pose)
                    rotation_to_sensor_frame = Transform(quat=[1, 0, 0, 0], pos=[0, 0, 0])
                    t = t * rotation_to_sensor_frame
                    tf_to_publish.transform = t.to_msg()
                    tfs_to_publish.append(tf_to_publish)

                    pcd = open3d.io.read_point_cloud(os.path.join(self.map_path, 'robot1', 'keyframe_' + keyframe_id + '.pcd'))
                    point_cloud_msg = icp_utils.open3d_to_ros(pcd)
                    point_cloud_msg.header = Header()
                    point_cloud_msg.header.frame_id = 'base_link'
                    point_cloud_msgs.append(point_cloud_msg)
            
            # for tf in tfs_to_publish:
            #     self.tf_broadcaster.sendTransform(tf)
            
            for point_cloud_msg in point_cloud_msgs:
                self.pointclouds2_publisher.publish(point_cloud_msg)
        
        
        # keyframes_path = os.path.join(self.map_path, 'robot1') 


        # for pcd_file_name in os.listdir(keyframes_path):
        #     pcd = open3d.io.read_point_cloud(os.path.join(keyframes_path, pcd_file_name))

        #     # points = np.asarray(pcd.points)
        #     # points = []
        #     # for point in pcd.points:
        #     #     points.append([point[0], point[1], point[2]])

        #     # self.get_logger().info(os.path.join(self.map_path, pcd_file_name))
        #     # self.get_logger().info(str(pcd.points))
        #     # rclcpp::Time now = node_->get_clock()->now();
        #     point_cloud_msg = icp_utils.open3d_to_ros(pcd)
        #     point_cloud_msg.header = Header()
        #     # point_cloud_msg.header.frame_id = pcd_file_name.replace('.pcd', '')
        #     point_cloud_msg.header.frame_id = 'robot1_map'
        #     # point_cloud_msg.fields = [
        #     #     PointField(name='x', offset=0,
        #     #              datatype=PointField.FLOAT32, count=1),
        #     #     PointField(name='y', offset=4,
        #     #                 datatype=PointField.FLOAT32, count=1),
        #     #     PointField(name='z', offset=8,
        #     #                 datatype=PointField.FLOAT32, count=1)
        #     # ]
        #     # point_cloud_msg.data = points

        #     # point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        #     if pcd_file_name == 'keyframe_0.pcd':
        #         self.get_logger().info('ICIIIII :')
        #         self.get_logger().info(str(point_cloud_msg.header))

        #     self.pointclouds2_publisher.publish(point_cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    cslam_storage = CslamStorage()
    rclpy.spin(cslam_storage)

    cslam_storage.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()