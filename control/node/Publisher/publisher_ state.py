import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64,Float64MultiArray

# class Publisher(Node):

# 	def __init__(self):
# 		super().__init__('Publisher_cart')
# 		self.publisher_= self.create_publisher(Twist,'/robot_first/cmd_vel',15)
# 		self.timer = self.create_timer(0.5, self.timer_callback)
# 		self.i = 0

  
#     def timer_callback(self):
#             msg = Twist()
#             msg.linear.x = 2.0 
#             msg.angular.z = 1.0 
#             self.publisher_.publish(msg)

# def main(args=Node):
# 	rclpy.init(args=args)
#     node = Publisher()
# 	publisher_cart = Publisher()
# 	rclpy.spin(node)
# 	rclpy.shutdown()

# if __name__=='main':
# 	main()

#///

def move_circle():
    pub = rclpy.Publisher(Twist,'robot_first/cmd_vel',  queue_size = 15)
    
    move_cmd = Twist()
    move_cmd.linear.x = 1.0
    move_cmd.angular.z = 1.0
    
    now = rclpy.Time.now()
    rate = rclpy.Rate()
    
    while rclpy.Time.now() < now + rclpy.Duration.from_sec(6):
        pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_circle
    except rclpy.ROSInterruptException:
        pass


