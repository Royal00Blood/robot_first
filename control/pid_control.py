import rclpy
from geometry_msgs.msg import Twist

def move_circle():
    pub = rospy.Publisher('robot_first/cmd_vel', Twist, queue_size = 1)
    
    move_cmd = Twist()
    move_cmd.linear.x = 1.0
    move_cmd.angular.z = 1.0
    
    now = rospy.Time.now()
    rate = rospy.Rate()
    
    while rospy.Time.now() < now + rospy.Duration.from_sec(6):
        pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_circle
    except rospy.ROSInterruptException:
        pass