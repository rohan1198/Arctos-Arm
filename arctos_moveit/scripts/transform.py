#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TfEchoPublisher(Node):
    
    def __init__(self):
        super().__init__('tf_echo_publisher')
        
        # Initialize the tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publisher
        self.publisher = self.create_publisher(TransformStamped, '/transformed_tf', 10)
        
        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        self.get_logger().info('TF Echo Publisher started')

    def timer_callback(self):
        try:
            # Look up transform from base_link to Gripper_1
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                'Gripper_1', 
                rclpy.time.Time()
            )
            
            # Publish the transform
            self.publisher.publish(transform)
            self.get_logger().debug('Transform published successfully')
            
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'Failed to lookup transform: {str(e)}')
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f'Connectivity error: {str(e)}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'Extrapolation error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    tf_echo_publisher = TfEchoPublisher()
    
    try:
        rclpy.spin(tf_echo_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        tf_echo_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
