import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
import time
import math
import sys

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller_node')
        # Create a publisher to the topic the turtle listens to
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Turtle Controller Node Started. Ready to move!')

    def move_circle(self):

        self.get_logger().info('Moving in a CIRCLE...')
        msg = Twist()
        msg.linear.x = 2.0  # Forward speed
        msg.angular.z = 1.0 # Turning speed
        
        self.publisher_.publish(msg)

    def move_square(self):
        """Moves the turtle in a square."""
        self.get_logger().info('Moving in a SQUARE...')
        
        stop_msg = Twist()
        forward_msg = Twist()
        forward_msg.linear.x = 2.0
        
        # 90 degrees = pi / 2 radians (approx 1.57)
        turn_msg = Twist()
        turn_msg.angular.z = math.pi / 2.0

        for i in range(4):
            self.get_logger().info(f'Square side {i+1}/4')
            
            # 1. Move forward for 2 seconds
            self.publisher_.publish(forward_msg)
            time.sleep(2.0)

            # 2. Stop and turn for 1 second
            self.publisher_.publish(turn_msg)
            time.sleep(1.0)
        
        # Stop at the end
        self.publisher_.publish(stop_msg)
        self.get_logger().info('Square complete.')

    def move_spiral(self):
        """Moves the turtle in an outward spiral."""
        self.get_logger().info('Moving in a SPIRAL... Press Ctrl+C in this terminal to stop.')
        
        msg = Twist()
        msg.angular.z = 1.5  # Constant turn rate
        linear_speed = 0.5   # Starting linear speed
        
        try:
            while rclpy.ok():
                msg.linear.x = linear_speed
                self.publisher_.publish(msg)
                
                
                linear_speed += 0.05
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            # Stop the turtle when Ctrl+C is pressed
            self.get_logger().info('Spiral stopped.')
            self.publisher_.publish(Twist()) 

    def stop_moving(self):
        self.get_logger().info('Stopping...')
        self.publisher_.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    controller_node = TurtleController()

    try:
        while rclpy.ok():
            print("\n--- Turtle Control Menu ---")
            print("  A: Move in a Circle")
            print("  B: Move in a Square")
            print("  C: Move in a Spiral")
            print("  Q: Quit")
            
            choice = input("Enter your choice (A, B, C, or Q): ").strip().upper()

            if choice == 'A':
                controller_node.move_circle()
                input("  -> Circle is active. Press ENTER to stop and return to menu.")
                controller_node.stop_moving()
                
            elif choice == 'B':
                controller_node.move_square()
                
            elif choice == 'C':
                controller_node.move_spiral()

            elif choice == 'Q':
                controller_node.get_logger().info('Quitting...')
                controller_node.stop_moving()
                break
            
            else:
                print("Invalid choice. Please try again.")
                
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()