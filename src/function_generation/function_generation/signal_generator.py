import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String
from signal_msgs.msg import SignalSpec
import math

class FunctionGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.declare_parameters(
            namespace  = '',
            parameters = [
                ('type', rclpy.Parameter.Type.INTEGER),
                ('frequency', rclpy.Parameter.Type.DOUBLE),
                ('offset'   , rclpy.Parameter.Type.DOUBLE),
                ('amplitude', rclpy.Parameter.Type.DOUBLE),
            ]
        )

        self.signal_publisher = self.create_publisher(Float32, 'signal', 10)
        self.params_publisher = self.create_publisher(SignalSpec, 'signal_params', 10)

        self.signal_timer_period = 0.002
        self.signal_timer = self.create_timer(self.signal_timer_period, self.signal_timer_callback)

        self.params_timer_period = 0.1
        self.params_timer = self.create_timer(self.params_timer_period, self.params_timer_callback)

        self.get_logger().info('Function generation online')

        self.signal_msg = Float32()
        self.params_msg = SignalSpec()
        self.params_msg.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        self.params_msg.offset =    self.get_parameter('offset').get_parameter_value().double_value
        self.params_msg.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.params_msg.type =      self.get_parameter('type').get_parameter_value().integer_value

        self.i = 0
    
    def signal_timer_callback(self):
        amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        offset =    self.get_parameter('offset').get_parameter_value().double_value
        frequency = self.get_parameter('frequency').get_parameter_value().double_value
        ftype =      self.get_parameter('type').get_parameter_value().integer_value

        if ftype == 1:
            self.signal_msg.data = amplitude * math.sin(2 * math.pi * frequency * self.i) + offset
        elif ftype == 2: # square wave
            for i in range(0, 10):
                if self.i % (1/frequency) < (1/frequency)/2:
                    self.signal_msg.data = amplitude + offset
                else:
                    self.signal_msg.data = -amplitude + offset
        elif ftype == 3: # sawtooth wave
            self.signal_msg.data = (2 * amplitude / (1/frequency)) * (self.i % (1/frequency)) - amplitude + offset
        else:
            self.signal_msg.data = 0

        self.signal_publisher.publish(self.signal_msg)
        self.i += self.signal_timer_period

    def params_timer_callback(self):
        self.params_msg.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        self.params_msg.offset =    self.get_parameter('offset').get_parameter_value().double_value
        self.params_msg.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.params_msg.type =      self.get_parameter('type').get_parameter_value().integer_value
        self.params_publisher.publish(self.params_msg)

def main(args=None):
    rclpy.init(args=args)
    e_p = FunctionGenerator()
    rclpy.spin(e_p)
    e_p.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
