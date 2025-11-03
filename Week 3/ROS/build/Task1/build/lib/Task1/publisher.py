import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher_node')
        # Create a publisher on the 'hello_topic'
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        
        # Create a timer to call the timer_callback function every 1 second
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Publisher node started, sending "Hello Rover" every second...')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello Rover'
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log to the console
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = HelloPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()