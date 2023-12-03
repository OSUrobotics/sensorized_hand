import rclpy
def main(args=None):
    rclpy.init(args=args)
    print("hi, waiting for msgs")
    

    rclpy.spin()
