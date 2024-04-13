import rospy
import time
from std_msgs.msg import String

rospy.init_node("publish_string_test")
pubString = rospy.Publisher("/subString", String, queue_size=10)
time.sleep(0.2)

x = 0
i = 0
while not rospy.is_shutdown():
    s = f"abcdefghijklmnopqrstuvwxyz {i%10}"
    print(s)
    # if(i == 1):
    #     exit(0)
    i += 1
    pubString.publish(s)
    x += 1
    time.sleep(0.1)