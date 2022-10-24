import rclpy
from geometry_msgs.msg import Twist

def callback(data):
    rclpy.loginfo("I heard %s",data.data)

def listener():
    rclpy.init_node('State')
    rclpy.Subscriber("/cmd_vel", Twist, callback)
    rclpy.spin()

