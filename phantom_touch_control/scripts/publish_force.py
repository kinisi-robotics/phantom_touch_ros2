#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped
from phantom_touch_msgs.msg import ButtonEvent
from typing import Optional



class ForcePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('force_publisher')
        
        # Publishers
        self.publisher_ = self.create_publisher(Vector3, '/TouchA/force_command', 10)
        
        # Subscribers
        self.button_subscription = self.create_subscription(
            ButtonEvent,
            '/TouchA/button_event',
            self.button_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/TouchA/pose_stylus_current',
            self.pose_callback,
            10
        )
        
        # Timer for force computation and publishing
        self.timer = self.create_timer(0.1, self.timer_callback)  # 100 Hz for smoother force control
        
        # State variables
        self.current_pose: Optional[PoseStamped] = None
        self.set_position: Optional[tuple] = None  # (x, y, z) set point when button 2 is pressed
        self.force_active: bool = False
        
        # Spring constant (you can adjust this value as needed)
        self.spring_constant: float = 40.0  # N/m
        
        self.get_logger().info('ForcePublisherNode started. Listening for button events and pose updates.')
    
    def button_callback(self, msg: ButtonEvent) -> None:
        """Handle button events from the phantom device."""
        if msg.button == ButtonEvent.BUTTON_WHITE and msg.event == ButtonEvent.EVENT_PRESSED:
            if self.current_pose is not None:
                # Save current position as set point
                self.set_position = (
                    self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y,
                    self.current_pose.pose.position.z
                )
                self.force_active = True
                self.get_logger().info(
                    f'Button 2 pressed! Set point saved: x={self.set_position[0]:.3f}, '
                    f'y={self.set_position[1]:.3f}, z={self.set_position[2]:.3f}'
                )
            else:
                self.get_logger().warn('Button 2 pressed but no current pose available')
        elif msg.button == ButtonEvent.BUTTON_WHITE and msg.event == ButtonEvent.EVENT_RELEASED:
            self.force_active = False
            self.set_position = None
            self.get_logger().info('Button 2 released! Force feedback disabled.')
    
    def pose_callback(self, msg: PoseStamped) -> None:
        """Update current pose from the phantom device."""
        self.current_pose = msg
    
    def timer_callback(self) -> None:
        """Compute and publish force based on Hooke's law."""
        force_msg = Vector3()
        
        if self.force_active and self.set_position is not None and self.current_pose is not None:
            # Get current position
            current_x = self.current_pose.pose.position.x
            current_y = self.current_pose.pose.position.y
            current_z = self.current_pose.pose.position.z
            
            # Compute displacement from set point
            dx = current_x - self.set_position[0]
            dy = current_y - self.set_position[1]
            dz = current_z - self.set_position[2]
            
            # Apply Hooke's law: F = -K * displacement (negative for restoring force)
            force_msg.x = -self.spring_constant * dx
            force_msg.y = -self.spring_constant * dy
            force_msg.z = -self.spring_constant * dz
            
            # Optional: Log force values (uncomment for debugging)
            # self.get_logger().info(
            #     f'Force: x={force_msg.x:.2f}, y={force_msg.y:.2f}, z={force_msg.z:.2f}'
            # )
        else:
            # No force when not active
            force_msg.x = 0.0
            force_msg.y = 0.0
            force_msg.z = 0.0

        
        self.get_logger().info(
                f'Force: x={force_msg.x:.2f}, y={force_msg.y:.2f}, z={force_msg.z:.2f}'
            )
        # Always publish (even zero force)
        self.publisher_.publish(force_msg)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ForcePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()