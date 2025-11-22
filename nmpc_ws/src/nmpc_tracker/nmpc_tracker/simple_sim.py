import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class SimpleSim(Node):
    def __init__(self):
        super().__init__('simple_sim')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/true_path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.05, self.update_physics) # 20Hz
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = self.get_clock().now()
        
        # Path history for Viz
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_physics(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Kinematics Update
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        self.th += self.w * dt

        # Quaternion
        qx = 0.0
        qy = 0.0
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        # Publish Odom
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)
        
        # Publish History Path
        pose = PoseStamped()
        pose.header.stamp = current_time.to_msg()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        self.path_msg.poses.append(pose)
        
        # Keep path length reasonable
        if len(self.path_msg.poses) > 500:
            self.path_msg.poses.pop(0)
            
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
