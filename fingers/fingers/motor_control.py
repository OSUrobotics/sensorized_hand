import rclpy
from rclpy.node import Node
from dynamixel_control import Dynamixel
from std_msgs.msg import String
from time import sleep
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion


class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')



        # Create an instance of the Dynamixel class
        self.dc = Dynamixel(port = '/dev/ttyUSB0')

        self.setup_motors()
        self.go_to_start_position()

        # General idea
        """
        1) read motor position and torque and publish it
        2) subscribe to motor commands and send them (just position for now???)
        """
        # Create a joint state publisher and transform broadcaster
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        timer_period = 0.1  # seconds (10 hz)
        self.timer = self.create_timer(timer_period, self.motor_timer_callback)

        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'world'
        self.odom_trans.child_frame_id = 'base_link'
        self.odom_trans.transform.translation.x = 0.0
        self.odom_trans.transform.translation.y = 0.0
        self.odom_trans.transform.translation.z = 0.0
        self.odom_trans.transform.rotation = \
            euler_to_quaternion(0, 0, 0)


        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

        self.motor_pos_sub = self.create_subscription(String, 'topic', self.listener_callback, 10)


    def motor_timer_callback(self):
        # Here we publish motor position and effort
        pos, current = self.dc.read_pos_torque() 

        # Fixing "negative" currents from max of 2 byte int to be actual negative values
        current = [(lambda i: i-65536.0 if i > 32768 else i)(i) for i in current]

        joint_state = JointState()
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['prox_right_joint', 'dist_right_joint', 'prox_left_joint', 'dist_left_joint']
        joint_state.position = pos
        joint_state.effort = [float(x) for x in current]

        self.odom_trans.header.stamp = now.to_msg()
        self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(self.odom_trans)

    def shutdown_motors(self):
        print("HERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRE")
        self.dc.reboot_dynamixel()
        self.dc.end_program()

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
        sleep(.25)
        print("Setup finished")

        print(self.dc.read_pos_torque())
    
    def go_to_start_position(self):
        # Semi-blocking command to move to the start position
        start_position = [0, 0, 0, 0]
        self.dc.go_to_position_all(start_position)
        sleep(2)


    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()
    rclpy.get_default_context().on_shutdown(motor_controller.shutdown_motors)

    rclpy.spin(motor_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # motor_controller.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()