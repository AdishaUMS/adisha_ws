import rclpy
import std_msgs.msg as std_msgs
from rclpy.node import Node



class SimpleSubscriberNode(Node):


    def __init__(self) -> None:
        # Inherit the Node class from rclpy
        super().__init__('simple_subscriber')

        # Create subscription
        self.subscriber = self.create_subscription(std_msgs.String, 'simple_publisher/msg', self.subscriptionCallback, 1)


    def subscriptionCallback(self, msg) -> None:
        # Print incoming message
        self.get_logger().info(f'{msg.data}')



def main(args=None):
    # Initiate rclpy
    rclpy.init(args=args)

    # Create a node
    simple_subscriber_node = SimpleSubscriberNode()

    # Spin the node until stopped
    rclpy.spin(simple_subscriber_node)

    # Shutdown and close everything
    simple_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()