#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

def send_waypoints():
    rclpy.init()
    node = Node('waypoint_sender')
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    
    waypoints = [
        (2.0, 0.0, 0.0),      # 2 meters forward
        (2.0, 2.0, 1.57),     # 2 meters right, facing 90 degrees
        (0.0, 2.0, 3.14),     # 2 meters back, facing 180 degrees
        (0.0, 0.0, 0.0),      # Return to start
    ]
    
    for i, (x, y, yaw) in enumerate(waypoints):
        print(f"Sending waypoint {i+1}: x={x}, y={y}, yaw={yaw}")
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        import math
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)
        
        goal_pub.publish(goal)
        
        if i < len(waypoints) - 1:
            print("Waiting 10 seconds before next waypoint...")
            time.sleep(10)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    send_waypoints()
