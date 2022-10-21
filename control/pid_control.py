import rospy
from std_msgm.msg import String

def callback(data):
    rospy.loginfo("I heard %s",data.data)

def listener():
    rospy.init_node('State')
    rospy.Subscriber("chatter",String,callback)
    rospy.spin()

