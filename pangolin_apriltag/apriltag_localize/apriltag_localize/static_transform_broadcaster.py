import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import yaml
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from transforms3d import quaternions, euler

class AprilTagPattern:
    def __init__(self, tag_id):
        self.tag_id = tag_id
        self.grid_size = 6 
        self.pattern = self._generate_36h11_pattern(tag_id)
    
    def _generate_36h11_pattern(self, tag_id):

        base_patterns = {
            0: [
                [1,1,1,1,1,1],
                [1,0,0,0,0,1],
                [1,0,1,1,0,1],
                [1,0,1,1,0,1],
                [1,0,0,0,0,1],
                [1,1,1,1,1,1]
            ],
            1: [
                [1,1,1,1,1,1],
                [1,0,0,0,0,1],
                [1,0,1,1,0,1],
                [1,0,0,1,0,1],
                [1,0,0,0,0,1],
                [1,1,1,1,1,1]
            ]
        }
        return base_patterns.get(tag_id % 2, base_patterns[0])

    def get_vertices(self, size):
        vertices = []
        colors = []
        
        cell_size = size / self.grid_size
        offset = -size / 2
        
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                x = offset + j * cell_size
                y = offset + i * cell_size
                
                if self.pattern[i][j] == 1:
                    color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
                else:
                    color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                
                vertices.extend([
                    Point(x=x, y=y, z=0.0),
                    Point(x=x+cell_size, y=y, z=0.0),
                    Point(x=x, y=y+cell_size, z=0.0)
                ])
                colors.extend([color, color, color])
                
                vertices.extend([
                    Point(x=x+cell_size, y=y, z=0.0),
                    Point(x=x+cell_size, y=y+cell_size, z=0.0),
                    Point(x=x, y=y+cell_size, z=0.0)
                ])
                colors.extend([color, color, color])
                
        return vertices, colors

class StaticTagBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tag_broadcaster')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, 'apriltag_markers', 10)
        self.timer = self.create_timer(1.0, self.broadcast_timer_callback)
        
        self.declare_parameter('tag_poses_file', '/home/bruce/CSL/Pangolin/pangolin_ws/src/quadruped_robot_4_DOF/pangolin_apriltag/apriltag_localize/config/tag_poses.yaml')
        tag_poses_file = self.get_parameter('tag_poses_file').value
        with open(tag_poses_file, 'r') as file:
            self.tag_info = yaml.safe_load(file)['tag_info']
    
    def create_tag_marker(self, tag_id, pose):
        tag_size = 0.162*3  # 16.2cm
        
        tag_pattern = AprilTagPattern(int(tag_id))
        vertices, colors = tag_pattern.get_vertices(tag_size)
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "apriltag_patterns"
        marker.id = int(tag_id)
        marker.type = Marker.TRIANGLE_LIST
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = pose[2]
        q = euler.euler2quat(pose[3], pose[4], pose[5], 'sxyz')
        # q = quaternion_from_euler(pose[3], pose[4], pose[5])
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        
        marker.points = vertices
        marker.colors = colors
        
        return marker

    def create_coordinate_axis(self, tag_id, pose):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "apriltag_axes"
        marker.id = int(tag_id) + 1000
        marker.type = Marker.LINE_LIST
        
        axis_length = 0.08  # 8cm
        points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=float(axis_length), y=0.0, z=0.0),  # X軸
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=0.0, y=float(axis_length), z=0.0),  # Y軸
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=0.0, y=0.0, z=float(axis_length))   # Z軸
        ]
        
        marker.points = points
        marker.scale.x = 0.005  # 線寬
        
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        ]
        
        marker.colors = colors
        
        marker.pose.position.x = float(pose[0])
        marker.pose.position.y = float(pose[1])
        marker.pose.position.z = float(pose[2])
        
        q = quaternion_from_euler(pose[3], pose[4], pose[5])
        marker.pose.orientation.x = float(q[0])
        marker.pose.orientation.y = float(q[1])
        marker.pose.orientation.z = float(q[2])
        marker.pose.orientation.w = float(q[3])
        
        return marker

    # def broadcast_tag_transforms(self):
    #     for tag_id, pose in self.map_tag_info.items():
    #         transform = TransformStamped()
    #         transform.header.stamp = self.get_clock().now().to_msg()
    #         transform.header.frame_id = 'map'
    #         transform.child_frame_id = f'tag_{tag_id}'
    #         transform.transform.translation = Vector3(x=pose[0], y=pose[1], z=pose[2])
    #         quat = euler.euler2quat(pose[3], pose[4], pose[5], 'sxyz')
    #         transform.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    #         self.static_broadcaster.sendTransform(transform)

    def broadcast_timer_callback(self):
        marker_array = MarkerArray()
        
        for tag_id, pose in self.tag_info.items():
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = f'tag_{tag_id}'
            transform.transform.translation = Vector3(x=pose[0], y=pose[1], z=pose[2])
            quat = euler.euler2quat(pose[3], pose[4], pose[5], 'sxyz')
            transform.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            self.broadcaster.sendTransform(transform)
            
            tag_marker = self.create_tag_marker(tag_id, pose)
            marker_array.markers.append(tag_marker)
            
            axis_marker = self.create_coordinate_axis(tag_id, pose)
            marker_array.markers.append(axis_marker)
        
        self.marker_pub.publish(marker_array)



def main(args=None):
    rclpy.init(args=args)
    node = StaticTagBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()