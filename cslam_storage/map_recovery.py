from std_msgs.msg import UInt32

class MapRecovery():
    def __init__(self, node, robot_id):
        self.node = node
        self.robot_heartbeat_subscriber = self.node.create_subscription(
            UInt32, 
            '/r' + str(robot_id) '/cslam/heartbeat', 
            self.heartbeat_received_callback, 
            10)
        
    def heartbeat_received_callback(self, msg):
        if (msg.data == 1):
            self.node.get_logger().info("Getting data from robot " + str(self.robot_id)) 

