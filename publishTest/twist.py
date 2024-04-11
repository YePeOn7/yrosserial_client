import rospy
import time
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math

x = 0
y = 0
w = 0
rospy.init_node("publish_test")
pubTwist = rospy.Publisher("/subTwist", Twist, queue_size=10)
time.sleep(0.2)

twistMsg = Twist()
while not rospy.is_shutdown():
    print(f"{x:.2f} {y:.2f} {w:.2f}")

    twistMsg.linear.x = x
    twistMsg.linear.y = y
    twistMsg.angular.z = w

    pubTwist.publish(twistMsg)
    
    x += 0.1
    y += 0.2
    w += 1

    time.sleep(0.1)