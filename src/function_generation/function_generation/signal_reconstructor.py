import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from signal_msgs.msg import SignalSpec

class FunctionReconstructor(Node):
    def __init__(self):
        super().__init__('signal_reconstructor')
        self.sub = self.create_subscription(SignalSpec, 'signal_params', self.specs_callback, 10)

        self.get_logger().info('Function reconstruction online')

        self.timer_period = 0.002
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.signal_publisher = self.create_publisher(Float32,'signal_reconstructed', 10)

        self.signal_msg = Float32()
        self.i = 0
        self.amplitude = 0
        self.offset = 0
        self.frequency = 0
        self.ftype = 0

        self.rflag = True

    def specs_callback(self, msg):
        self.amplitude = msg.amplitude
        self.offset = msg.offset
        self.frequency = msg.frequency
        self.ftype = msg.type

    def timer_callback(self):
        if self.ftype == 1:
            self.signal_msg.data = self.amplitude * math.sin(2 * math.pi * self.frequency * self.i) + self.offset
        elif self.ftype == 2:
            for i in range(0, 10):
                if self.i % (1/self.frequency) < (1/self.frequency)/2:
                    self.signal_msg.data = self.amplitude + self.offset
                else:
                    self.signal_msg.data = -self.amplitude + self.offset
        elif self.ftype == 3:
            self.signal_msg.data = (2 * self.amplitude / (1/self.frequency)) * (self.i % (1/self.frequency)) - self.amplitude + self.offset

        self.signal_publisher.publish(self.signal_msg)
        self.i += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    e_s = FunctionReconstructor()
    rclpy.spin(e_s)
    e_s.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
