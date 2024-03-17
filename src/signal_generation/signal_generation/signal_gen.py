import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import math

class ExamplePublisher(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.pubisher = self.create_publisher(Float32, 'signal', 10)
        self.pubisher_counter = self.create_publisher(Float32, 'time', 10)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

        timer_period = 0.05
        self.timer_2 = self.create_timer(timer_period, self.timer_callback_counter)
        self.get_logger().info('Signal node init')
        self.msg = Float32()
        self.msg_counter = Float32()
        self.i = 0.05
    
    def timer_callback_counter(self):
        self.msg_counter.data = self.i
        self.pubisher_counter.publish(self.msg_counter)
        self.i += 0.05

    def timer_callback(self):
        self.msg.data = math.sin(self.i/2)
        self.pubisher.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    e_p = ExamplePublisher()
    rclpy.spin(e_p)
    e_p.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
