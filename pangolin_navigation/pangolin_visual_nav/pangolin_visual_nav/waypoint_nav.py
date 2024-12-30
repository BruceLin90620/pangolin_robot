#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from routing_agent_interfaces.srv import NavServiceMsg
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time

class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        
        # Initialize class variables
        self.path_sequence_ = []
        self.current_position_ = ""
        self.current_graph_ = {}
        self.current_map_id_ = 0
        
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Create service client for navigation
        self.nav_service_client_ = self.create_client(
            NavServiceMsg,
            '/NavService'
        )

    def has_active_task(self) -> bool:
        """Check if there are any active tasks in the path sequence"""
        return len(self.path_sequence_) > 0

    def request_and_process_path(self, current_position: str) -> str:
        """
        Request and process navigation path from current position
        
        Returns:
            str: Updated current position after processing
        """
        # Wait for service to be available
        while not self.nav_service_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('NavService not available, waiting...')
            
        # Create request
        request = NavServiceMsg.Request()
        request.can_arrive = True
        request.i_am_at = current_position
        self.current_position_ = current_position
        
        self.get_logger().info(f'Sending request to NavService from position: {current_position}')
        
        try:
            # Send request and wait for response
            future = self.nav_service_client_.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            response = future.result()
            if response is not None:
                return self.process_navigation_response(response, current_position)
            else:
                self.get_logger().error('Failed to call NavService')
                self.path_sequence_ = []
        except Exception as e:
            self.get_logger().error(f'Error in request_and_process_path: {str(e)}')
            self.path_sequence_ = []
            
        return current_position

    def process_navigation_response(self, result, current_position: str) -> str:
        """Process the navigation service response"""
        try:
            json_response = json.loads(result.path_to_next_task)
            self.path_sequence_ = json_response["node_sequence:"]
            
            if not self.path_sequence_:
                self.get_logger().info('No tasks available at the moment.')
                return current_position
            
            # Update graph information
            self.update_graph_information(json_response["graph"])
            
            # Log path sequence
            self.log_path_sequence()
            
            # Process first point in sequence and get new position
            if self.path_sequence_:
                return self.process_next_point(current_position)
                
        except Exception as e:
            self.get_logger().error(f'Error parsing response: {str(e)}')
            self.path_sequence_ = []
        
        return current_position

    def update_graph_information(self, new_graph):
        """Update the current graph with new information"""
        self.current_graph_.update(new_graph)

    def log_path_sequence(self):
        """Log the complete path sequence"""
        self.get_logger().info(f'Received path sequence with {len(self.path_sequence_)} points:')
        
        for i, node_id in enumerate(self.path_sequence_):
            location = self.current_graph_[node_id]["local_location"]
            self.get_logger().info(
                f'Point {i + 1}: {node_id} ({location[0]:.2f}, {location[1]:.2f})'
            )

    def process_next_point(self, current_position: str) -> str:
        """
        Process the next point in the sequence
        
        Returns:
            str: The new current position after processing
        """
        self.current_sequence_index_ = 0
        next_node_id = self.path_sequence_[0]
        
        # Extract map ID from node ID
        map_id = int(next_node_id[:3])
        
        if map_id != self.current_map_id_:
            # self.switch_map(map_id, next_node_id)
            self.get_logger().info(f'switch_map')
            time.sleep(3)
        else:
            # self.send_goal(next_node_id, self.current_graph_[next_node_id], current_position)
            self.get_logger().info(f'next_node_id: {next_node_id}, current_graph_ {self.current_graph_[next_node_id]} current_position {current_position}')
            time.sleep(3)
        
        # Remove the processed point from the sequence
        if self.path_sequence_:
            self.path_sequence_.pop(0)
        
        return next_node_id

def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    
    # Set initial position
    current_position = "000_019"
    
    try:
        while rclpy.ok():
            print(f"Requesting task from position: {current_position}")
            
            # Run the request and update current position
            current_position = node.request_and_process_path(current_position)
            
            # Process ROS callbacks
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # If no active task is present, wait for 5 seconds before retrying
            if not node.has_active_task():
                node.get_logger().info("No active task. Waiting 5 seconds before retrying...")
                time.sleep(5)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()