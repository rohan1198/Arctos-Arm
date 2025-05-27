#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
import copy


class MoveGroupPythonInterface(Node):
    """MoveIt2 Python Interface"""
    
    def __init__(self):
        super().__init__('moveit_interface_node')
        
        # Create subscription for UI commands
        self.subscription = self.create_subscription(
            String,
            'ui_command',
            self.ui_command_callback,
            10
        )
        
        # Create publisher for joint states (for simple demonstration)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        self.get_logger().info('MoveIt2 Python Interface initialized')

    def ui_command_callback(self, msg):
        command = msg.data.split(',')
        
        if command[0] == "go_to_joint_state":
            joint_state_values = [float(value) for value in command[1:7]]
            self.go_to_joint_state(joint_state_values)
            
        elif command[0] == "plan_cartesian_path":
            cartesian_path_values = [float(value) for value in command[1:]]
            self.plan_cartesian_path(cartesian_path_values)
            
        elif command[0] == "open_gripper":
            self.open_gripper()
            
        elif command[0] == "close_gripper":
            self.close_gripper()
            
        else:
            self.get_logger().warning(f"Unknown command received: {command[0]}")

    def go_to_joint_state(self, joint_state_values):
        """Move to joint state"""
        self.get_logger().info(f"Moving to joint state: {joint_state_values}")
        
        # Create and publish joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"] 
        joint_state.position = joint_state_values
        
        self.joint_pub.publish(joint_state)
        self.get_logger().info("Joint state command published")

    def plan_cartesian_path(self, cartesian_path_values):
        """Plan and execute cartesian path"""
        self.get_logger().info(f"Cartesian path with values: {cartesian_path_values}")
        
        # This is a simplified implementation
        # In a full implementation, you would use MoveIt2's cartesian path planning
        self.get_logger().info("Cartesian path planning not fully implemented in this basic version")

    def open_gripper(self):
        """Open gripper"""
        self.get_logger().info("Opening gripper")
        
        # Create and publish gripper joint state
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["jaw1", "jaw2"]
        joint_state.position = [0.0, 0.0]
        
        self.joint_pub.publish(joint_state)

    def close_gripper(self):
        """Close gripper"""
        self.get_logger().info("Closing gripper")
        
        # Create and publish gripper joint state
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["jaw1", "jaw2"]
        joint_state.position = [0.015, 0.015]
        
        self.joint_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    interface = MoveGroupPythonInterface()
    
    try:
        rclpy.spin(interface)
    except KeyboardInterrupt:
        pass
    finally:
        interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
