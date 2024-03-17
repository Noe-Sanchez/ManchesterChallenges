import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32

class ExampleSubscriber(Node):
    def __init__(self):
        super().__init__('process')
        self.sub = self.create_subscription(Float32, 'signal', self.listener_callback, 10)
        self.sub = self.create_subscription(Float32, 'time', self.time_callback, 10)
        self.get_logger().info('Process node init')
        self.publisher = self.create_publisher(Float32,'proc_signal', 10)
        self.msg = Float32()

    def listener_callback(self, msg):
        pass

    def time_callback(self, msg):
        self.msg.data = 0.5*math.sin((msg.data+2)/2)+0.5
        self.publisher.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    e_s = ExampleSubscriber()
    rclpy.spin(e_s)
    e_s.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
