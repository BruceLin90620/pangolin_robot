import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Pose, Point
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from isaac_ros_visual_slam_interfaces.srv import SetSlamPose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
from transforms3d import quaternions, euler
import yaml
# from tf_transformations import quaternion_from_euler

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

class AprilTagLocalization(Node):
    def __init__(self):
        super().__init__('apriltag_localization')
        # Subscribe to AprilTag detections
        self.tag_sub = self.create_subscription(AprilTagDetectionArray, '/tag_detections', self.tag_callback, 10)
        # Create a client to update the SLAM pose
        self.vslam_client = self.create_client(SetSlamPose, 'visual_slam/set_slam_pose')
        # Create publisher for marker visualization
        self.marker_pub = self.create_publisher(MarkerArray, 'apriltag_markers', 10)
        
        self.load_tag_poses()
        self.setup_tf()
        # Create timer for visualization
        # self.vis_timer = self.create_timer(1.0, self.publish_tag_markers)

    def load_tag_poses(self):
        tag_poses_file = self.declare_parameter('tag_poses_file', 
            '/home/pangolin/pangolin_ws/src/quadruped_robot_4_DOF/pangolin_apriltag/apriltag_localize/config/tag_poses.yaml').value
        with open(tag_poses_file, 'r') as file:
            self.map_tag_info = yaml.safe_load(file)['tag_info']

    def setup_tf(self):
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self.broadcast_tag_transforms)

    def create_tag_marker(self, tag_id, pose):
        tag_size = 0.162*3  
        
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
        
        axis_length = 0.08  
        points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=float(axis_length), y=0.0, z=0.0),  
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=0.0, y=float(axis_length), z=0.0),  
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=0.0, y=0.0, z=float(axis_length))   
        ]
        
        marker.points = points
        marker.scale.x = 0.005 
        
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
        
        q = euler.euler2quat(pose[3], pose[4], pose[5], 'sxyz')
        marker.pose.orientation.x = float(q[0])
        marker.pose.orientation.y = float(q[1])
        marker.pose.orientation.z = float(q[2])
        marker.pose.orientation.w = float(q[3])
        
        return marker

    def publish_tag_markers(self):
        marker_array = MarkerArray()
        
        for tag_id, pose in self.map_tag_info.items():
            tag_marker = self.create_tag_marker(tag_id, pose)
            marker_array.markers.append(tag_marker)
            
            axis_marker = self.create_coordinate_axis(tag_id, pose)
            marker_array.markers.append(axis_marker)
        
        self.marker_pub.publish(marker_array)

    def broadcast_tag_transforms(self):
        marker_array = MarkerArray()

        for tag_id, pose in self.map_tag_info.items():
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = f'tag_{tag_id}'
            transform.transform.translation = Vector3(x=pose[0], y=pose[1], z=pose[2])
            quat = euler.euler2quat(pose[3], pose[4], pose[5], 'sxyz')
            transform.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            self.static_broadcaster.sendTransform(transform)

            tag_marker = self.create_tag_marker(tag_id, pose)
            marker_array.markers.append(tag_marker)
            
            axis_marker = self.create_coordinate_axis(tag_id, pose)
            marker_array.markers.append(axis_marker)

        self.marker_pub.publish(marker_array)

    def tag_callback(self, msg):
        if not msg.detections:
            return
        
        detection = msg.detections[0]
        detected_tag_pose = detection.pose.pose.pose
        detected_tag_id = detection.id
        detection_distance = np.hypot(detected_tag_pose.position.z, detected_tag_pose.position.x)
        
        if detection_distance < 1.0:
            self.process_detection(detected_tag_id)

    def process_detection(self, detected_tag_id):
        try:
            time_now = rclpy.time.Time()
            base_to_tag = self.tf_buffer.lookup_transform("base_link", f"tag36h11:{detected_tag_id}", time_now)
            map_to_tag = self.tf_buffer.lookup_transform("map", f"tag_{detected_tag_id}", time_now)
            robot_pose_map = self.multiply_transforms(map_to_tag.transform, self.invert_transform(base_to_tag.transform))
            
            pose = Pose()
            pose.position.x = robot_pose_map.translation.x
            pose.position.y = robot_pose_map.translation.y
            pose.position.z = robot_pose_map.translation.z
            pose.orientation = robot_pose_map.rotation

            self.get_logger().info(f"Robot pose in map frame: {pose}")
            self.send_pose_update(pose)
        except Exception as e:
            self.get_logger().error(f'Failed to compute transform: {str(e)}')

    def send_pose_update(self, pose):
        request = SetSlamPose.Request()
        request.pose = pose
        self.vslam_client.call_async(request)

    def multiply_transforms(self, t1, t2):
        m1 = self.transform_to_matrix(t1)
        m2 = self.transform_to_matrix(t2)
        result_matrix = np.dot(m1, m2)
        return self.matrix_to_transform(result_matrix)

    def transform_to_matrix(self, transform):
        translation = [transform.translation.x, transform.translation.y, transform.translation.z]
        rotation = [transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z]
        mat = np.eye(4)
        mat[:3, :3] = quaternions.quat2mat(rotation)
        mat[:3, 3] = translation
        return mat

    def matrix_to_transform(self, matrix):
        translation = matrix[:3, 3]
        rotation_matrix = matrix[:3, :3]
        quat = quaternions.mat2quat(rotation_matrix)
        transform = TransformStamped().transform
        transform.translation = Vector3(x=translation[0], y=translation[1], z=translation[2])
        transform.rotation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])
        return transform

    def invert_transform(self, transform):
        mat = self.transform_to_matrix(transform)
        inv_mat = np.linalg.inv(mat)
        return self.matrix_to_transform(inv_mat)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()