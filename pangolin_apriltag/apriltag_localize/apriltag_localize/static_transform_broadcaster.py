import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class StaticTagBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tag_broadcaster')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.broadcast_timer_callback)
        
        # Load tag poses from YAML
        self.declare_parameter('tag_poses_file', '/home/pangolin//pangolin_ws/pangolin_robot/pangolin_visual_slam/apriltag_localize/config/tag_poses.yaml')
        tag_poses_file = self.get_parameter('tag_poses_file').value
        with open(tag_poses_file, 'r') as file:
            self.tag_info = yaml.safe_load(file)['tag_info']

    def broadcast_timer_callback(self):
        for tag_id, pose in self.tag_info.items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = f'tag_{tag_id}'
            t.transform.translation.x = pose[0]
            t.transform.translation.y = pose[1]
            t.transform.translation.z = pose[2]
            q = quaternion_from_euler(pose[3], pose[4], pose[5])
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = StaticTagBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()