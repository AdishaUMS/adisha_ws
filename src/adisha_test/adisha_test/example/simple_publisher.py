import rclpy
import std_msgs.msg as std_msgs
from rclpy.node import Node



class SimplePublisherNode(Node):


    def __init__(self) -> None:
        # Attributes
        self.elapsed_t = 0

        # Inherit the Node class from rclpy
        super().__init__('simple_publisher')

        # Create publisher
        self.publisher = self.create_publisher(std_msgs.String, 'simple_publisher/msg', 10)

        # Create a timer to publish a message for every 1 second
        self.timer = self.create_timer(1.0, self.publishCallback)


    def publishCallback(self) -> None:
        # Create a message
        msg         = std_msgs.String()
        msg.data    = f'{self.elapsed_t}s since the simple_publisher started'

        # Publish the message
        self.publisher.publish(msg)

        # Increment time
        self.elapsed_t += 1



def main(args=None):
    # Initiate rclpy
    rclpy.init(args=args)

    # Create the Node
    simple_publisher_node = SimplePublisherNode()

    # Spin the node until stopped
    rclpy.spin(simple_publisher_node)

    # Shutdown and close everything
    simple_publisher_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()