import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # ROS 2 Image message
import cv2  # OpenCV
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher_node')
        self.publisher_ = self.create_publisher(Image, 'video_feed', 10)

        # We'll publish at ~30 FPS (1.0 / 30.0 = 0.033 seconds)
        self.timer_ = self.create_timer(0.033, self.timer_callback)

        # Try to open the default webcam (device 0)
        self.cap_ = cv2.VideoCapture(0)
        if not self.cap_.isOpened():
            self.get_logger().error('Could not open video capture device 0')

        # Create a CvBridge instance
        self.bridge_ = CvBridge()
        self.get_logger().info('Video publisher started. Publishing to /video_feed')

    def timer_callback(self):
        # Read a frame from the webcam
        ret, frame = self.cap_.read()

        if ret:
            # Convert the OpenCV image (a BGR ndarray) to a ROS 2 Image message
            ros_image_msg = self.bridge_.cv2_to_imgmsg(frame, encoding="bgr8")

            # Publish the message
            self.publisher_.publish(ros_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.cap_.release() # Release the webcam
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()