import rospy
import time
from std_msgs.msg import Float32, Float64

f32 = 0
f64 = 0
rospy.init_node("publish_test_float")
pubFloat32 = rospy.Publisher("/subFloat32", Float32, queue_size=10)
pubFloat64 = rospy.Publisher("/subFloat64", Float64, queue_size=10)
time.sleep(0.2)

float32Msg = Float32()
float64Msg = Float64()
while not rospy.is_shutdown():
    float32Msg.data = f32
    float64Msg.data = f64

    pubFloat32.publish(float32Msg)
    pubFloat64.publish(float64Msg)
    
    print(f"{f32:.2f} {f64:.2f}")

    f32 += 0.1
    f64 += 0.1

    # exit(0)

    time.sleep(0.01)