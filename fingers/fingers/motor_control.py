import rclpy
from rclpy.node import Node
from dynamixel_control import Dynamixel
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from time import sleep
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion
from hand_msgs.action import Gripper
from rclpy.action import ActionServer
import threading
from time import sleep



class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')

        # Create a quasi-mutex for disabling motor polling while in action server state
        self.mutex = False

        # Create an instance of the Dynamixel class
        self.dc = Dynamixel(port = '/dev/ttyUSB0')

        self.setup_motors()
        self.go_to_start_position()
        self.closed = False
        self.cancelled = False
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

        # self.motor_pos_sub = self.create_subscription(String, 'topic', self.listener_callback, 10)

        # Set up motor control action server
        
        self.act_serv = ActionServer(self, Gripper, 'gripper_command', self.gipper_command)

    def gipper_command(self, goal):
        command = goal.request.command
        if command == "close-parallel":
            # Start a callback 
            self.open_time = self.create_timer(15, self.cancel_callback)
            self.closed = True
            self.cancelled = False
            
            result = self.close_parallel(goal)
            return result
        elif command == "open":
            self.closed = False
            result = self.open_gripper(goal)
            return result
        else:
            self.get_logger().error('Gripper command "%s" not implemented.' % command)
            result = 0
            return result
        
        #  goal_handle.succeed()
    def cancel_callback(self):
        self.get_logger().error('Opening, no open request in a few seconds!')
        self.cancelled = True
        self.open_gripper(None)
        self.open_time.destroy()

    def open_gripper(self, goal):
    
        self.mutex = True
        if self.open_time:
            self.open_time.destroy()
            
        sleep(.1)
        print("opening gripper")
        self.dc.reboot_dynamixel()
        for id in self.dc.dxls.keys():
            self.dc.packetHandler.reboot(self.dc.portHandler, id)
        sleep(.25)
        for id in self.dc.dxls.keys():
            self.dc.enable_torque(id, True)
        
        self.go_to_start_position()
        self.mutex = False
        
        

        # TODO: Implement error checking and actual feedback here
        if goal:
            goal.succeed()
            result = Gripper.Result()
            result.result = 2
            return result

    def close_parallel(self, goal):
        # Goal is to close and maintain parallel interfaces between distal links
        # 1) set goal current 600 dist, 400 prox???
        # 2) while not at goal current
            # - set target position to be a bit inside the current position
            # - wait a small period of time
        # 

        rate = self.create_rate(10)
        current_target = [600, 800, 600, 800]
        # Set the goal current

        # Start by disabling current pose publisher
        self.mutex = True
        rate.sleep()

        # Set the current goal current
        for i in range(4):
            self.dc.add_parameter(id = i, address = 102, byte_length = 2, value = current_target[i])
        self.dc.send_parameters()

        # Change velocity profile
        for i in range(4):
            self.dc.add_parameter(id = i, address = 112, byte_length = 4, value = 25)
        self.dc.send_parameters()

        # Update motor position until we reach a current thresehold
        rate = self.create_rate(10)
        while not self.check_if_current_at_target(current_target) and not self.cancelled:
            # Publish our current joint pose
            pos, current = self.dc.read_pos_torque() 
            self.publish_pose(pos, current)

            left = self.dc.dxls[0].read_position_rad #+ .05
            right = -left #self.dc.dxls[2].read_position_rad #- .05 
            # new_pos_target = [left+.02, -left, right-.02, -right]
            new_pos_target = [left+2.0, -(left-.02), (right-2.0), -(right+.02)]

            self.dc.go_to_position_all(new_pos_target)
            rate.sleep()
        self.dc.set_speed(10)
        self.mutex = False
        if not self.cancelled:
            goal.succeed()
        else:
            goal.cancelled()
        result = Gripper.Result()
        # TODO: Update to return an actual usable result
        result.result = 2
        return result


    def check_if_current_at_target(self, target_current):
        # Return true if both proximal motors are at their target current
        for i in [0,2]:
            dxl_cur = self.dc.dxls[i].current_torque
            if dxl_cur > 32768:
                dxl_cur = 65536-dxl_cur
            print(f"Current: {dxl_cur}, target: {target_current[i]}")
            if dxl_cur - target_current[i] < -10:
                return False
        return True



    def motor_timer_callback(self):
        if not self.mutex: 
            # Here we publish motor position and effort
            pos, current = self.dc.read_pos_torque() 
            self.publish_pose(pos, current)

            

    def publish_pose(self, position, current):
        # Fixing "negative" currents from max of 2 byte int to be actual negative values
        current = [(lambda i: i-65536.0 if i > 32768 else i)(i) for i in current]

        joint_state = JointState()
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['prox_right_joint', 'dist_right_joint', 'prox_left_joint', 'dist_left_joint']
        joint_state.position = position
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

        self.dc.set_speed(40)
        # Update abs max current
        current_target = [900, 1100, 900, 1100]
        for i in range(4):
            self.dc.add_parameter(id = i, address = 38, byte_length = 2, value = current_target[i])
        self.dc.send_parameters()
        self.dc.setup_all()
        self.dc.update_PID(1000,400,2000)
        # Give it a small break 
        sleep(.25)
        print("Setup finished")

        print(self.dc.read_pos_torque())
    
    def go_to_start_position(self):
        # Semi-blocking command to move to the start position
        self.mutex = True
        sleep(.1)
        self.dc.set_speed(80)
        
        start_position = [-.8, 0, .8, 0]
        self.dc.go_to_position_all(start_position)
        self.mutex = False
        sleep(1.9)
        self.mutex = True
        sleep(.1)
        start_position = [-.8, .8, .8, -.8]
        self.dc.go_to_position_all(start_position)
        self.mutex = False
        sleep(1.9)
        self.mutex = True
        sleep(.1)
        self.dc.set_speed(40)
        self.mutex = False


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
    executor = MultiThreadedExecutor()


    try:
        rclpy.spin(motor_controller, executor=executor)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        motor_controller.mutex = True
        sleep(.1)
        motor_controller.dc.reboot_dynamixel()
        motor_controller.dc.end_program()
        motor_controller.get_logger().info("Dynamixel torque disabled, port closed.")
        pass
    finally:
        # motor_controller.destroy_node()
        rclpy.try_shutdown()
    
    # Spin in a separate thread
    # thread = threading.Thread(target=rclpy.spin, args=(motor_controller, ), daemon=True)
    # thread.start()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # motor_controller.destroy_node()
    # rclpy.shutdown()
    # rate = motor_controller.create_rate(2)
    # while rclpy.ok():
    #     print("broke")
    #     rate.sleep()


if __name__ == '__main__':
    main()