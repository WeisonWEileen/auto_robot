import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(Int64, 'robot_mode', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int64()
        msg.data = 3  # 直接将2赋值给msg.data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    my_publisher = MyPublisher()

    rclpy.spin(my_publisher)

    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()