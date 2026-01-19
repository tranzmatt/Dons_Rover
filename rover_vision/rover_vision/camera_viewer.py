#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Parameters
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('show_window', True)
        self.declare_parameter('window_name', 'Rover Camera')
        self.declare_parameter('display_fps', True)
        
        # Get parameters
        image_topic = self.get_parameter('image_topic').value
        self.show_window = self.get_parameter('show_window').value
        self.window_name = self.get_parameter('window_name').value
        self.display_fps = self.get_parameter('display_fps').value
        
        # CV Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)
        
        # Stats
        self.frame_count = 0
        self.last_time = self.get_clock().now()
        self.fps = 0.0
        
        self.get_logger().info(f'Rover camera viewer started')
        self.get_logger().info(f'Subscribing to: {image_topic}')
        self.get_logger().info(f'Show window: {self.show_window}')
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            self.frame_count += 1
            
            # Calculate FPS
            if self.display_fps and self.frame_count % 10 == 0:
                current_time = self.get_clock().now()
                time_diff = (current_time - self.last_time).nanoseconds / 1e9
                self.fps = 10.0 / time_diff if time_diff > 0 else 0.0
                self.last_time = current_time
            
            # Add overlay info to image
            height, width = cv_image.shape[:2]
            
            # Frame counter
            cv2.putText(cv_image, f'Frame: {self.frame_count}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.6, (0, 255, 0), 2)
            
            # Resolution
            cv2.putText(cv_image, f'Size: {width}x{height}', 
                       (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.6, (0, 255, 0), 2)
            
            # FPS
            if self.display_fps:
                cv2.putText(cv_image, f'FPS: {self.fps:.1f}', 
                           (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (0, 255, 0), 2)
            
            # Display the image if enabled
            if self.show_window:
                cv2.imshow(self.window_name, cv_image)
                cv2.waitKey(1)
            
            # Log every 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Processed {self.frame_count} frames @ {self.fps:.1f} FPS')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def __del__(self):
        if self.show_window:
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        viewer = CameraViewer()
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()