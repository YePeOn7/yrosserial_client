import rospy
import time
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math

def setOdometry(x, y, w):
    odometryMsg = Odometry()
    odometryMsg.pose.pose.position.x = x
    odometryMsg.pose.pose.position.y = y

    q = quaternion_from_euler(0, 0, math.radians(w))

    odometryMsg.pose.pose.orientation.x = q[0]
    odometryMsg.pose.pose.orientation.y = q[1]
    odometryMsg.pose.pose.orientation.z = q[2]
    odometryMsg.pose.pose.orientation.w = q[3]

    return odometryMsg

x = 0
y = 0
w = 0
rospy.init_node("publish_test")
pubOdometry = rospy.Publisher("/subOdometry", Odometry, queue_size=10)
time.sleep(0.2)

# odometryMsg = setOdometry(x, y, w)
# pubOdometry.publish(odometryMsg)

while not rospy.is_shutdown():
    print(f"{x:.2f} {y:.2f} {w:.2f}")
    odometryMsg = setOdometry(x, y, w)

    x += 0.1
    y += 0.2
    w += 1

    if(w>360):
        w -= 360

    # q = (odometryMsg.pose.pose.orientation.x, 
    #      odometryMsg.pose.pose.orientation.y, 
    #      odometryMsg.pose.pose.orientation.z, 
    #      odometryMsg.pose.pose.orientation.w)
    
    # a = euler_from_quaternion(q)
    # print(odometryMsg.pose.pose.orientation)
    # print(math.degrees(a[2]))
    # pubOdometry.publish()
        
    pubOdometry.publish(odometryMsg)

    time.sleep(0.1)