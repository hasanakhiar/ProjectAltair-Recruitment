import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber_node')
        self.subscription_ = self.create_subscription(
            Image,
            'video_feed',         
            self.listener_callback,
            10)

        self.bridge_ = CvBridge()
        self.get_logger().info('Video subscriber started. Displaying /video_feed')

        # Create an OpenCV window
        cv2.namedWindow('ROS 2 Video Feed', cv2.WINDOW_AUTOSIZE)

    def listener_callback(self, msg):
        try:
            cv_frame = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Display the image in the window
        cv2.imshow('ROS 2 Video Feed', cv_frame)
        cv2.waitKey(1) 

def main(args=None):
    rclpy.init(args=args)
    node = VideoSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()