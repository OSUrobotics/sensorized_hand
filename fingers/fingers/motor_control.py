import rclpy
from rclpy.node import Node
from dynamixel_control import Dynamixel
from std_msgs.msg import String


class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')

        # Create an instance of the Dynamixel class
        self.dc = Dynamixel(port = '/dev/ttyUSB0')
        self.setup_motors()

        # General idea
        """
        1) read motor position and torque and publish it
        2) subscribe to motor commands and send them (just position for now???)
        """

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def setup_motors(self):
        # Connect each of the motors and set their settings
        self.dc.add_dynamixel(type="XL-330", ID_number=0, calibration=[1023,2048,3073])
        self.dc.add_dynamixel(type="XL-330", ID_number=1, calibration=[1023,2048,3073])
        self.dc.add_dynamixel(type="XL-330", ID_number=2, calibration=[1023,2048,3073]) 
        self.dc.add_dynamixel(type="XL-330", ID_number=3, calibration=[1023,2048,3073])

        self.dc.set_speed(80)
        self.dc.setup_all()
        self.dc.update_PID(1000,400,2000)

        # Give it a small break 
        rclpy.rate(2).sleep()
    
    def go_to_start_position(self):
        # Semi-blocking command to move to the start position
        start_position = [.2, 0, -.2, 0]
        self.dc.go_to_start_position(start_position)
        rclpy.sleep(2)


    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()

    rclpy.spin(motor_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()