import rospy
from std_msgs.msg import *
import time

L_ARRAY = 20

def incrementWithLimit(data, limit, startValue, inc = 1):
    data += inc
    if(data > limit):
        data = startValue
    return data

def decrementWithLimit(data, limit, startValue, dec = 1):
    data -= dec
    if(data < limit):
        data = startValue
    return data

rospy.init_node("pubIntMulTest")
pubU8M = rospy.Publisher("/sub_u8m", UInt8MultiArray, queue_size=10)
pubS8M = rospy.Publisher("/sub_s8m", Int8MultiArray, queue_size=10)
pubU16M = rospy.Publisher("/sub_u16m", UInt16MultiArray, queue_size=10)
pubS16M = rospy.Publisher("/sub_s16m", Int16MultiArray, queue_size=10)
pubU32M = rospy.Publisher("/sub_u32m", UInt32MultiArray, queue_size=10)
pubS32M = rospy.Publisher("/sub_s32m", Int32MultiArray, queue_size=10)
pubU64M = rospy.Publisher("/sub_u64m", UInt64MultiArray, queue_size=10)
pubS64M = rospy.Publisher("/sub_s64m", Int64MultiArray, queue_size=10)
pubF32M = rospy.Publisher("/sub_f32m", Float32MultiArray, queue_size=10)
pubF64M = rospy.Publisher("/sub_f64m", Float64MultiArray, queue_size=10)
time.sleep(0.1)

u8mMsg = UInt8MultiArray()
s8mMsg = Int8MultiArray()
u16mMsg = UInt16MultiArray()
s16mMsg = Int16MultiArray()
u32mMsg = UInt32MultiArray()
s32mMsg = Int32MultiArray()
u64mMsg = UInt64MultiArray()
s64mMsg = Int64MultiArray()
f32mMsg = Float32MultiArray()
f64mMsg = Float64MultiArray()

u8mMsg.data = [123+i for i in range(L_ARRAY)]
s8mMsg.data = [-123+i for i in range(L_ARRAY)]
u16mMsg.data = [28001+i for i in range(L_ARRAY)]
s16mMsg.data = [-28001+i for i in range(L_ARRAY)]
u32mMsg.data = [1800000001+i for i in range(L_ARRAY)]
s32mMsg.data = [-1800000001+i for i in range(L_ARRAY)]
u64mMsg.data = [8000000000000000001+i for i in range(L_ARRAY)]
s64mMsg.data = [-8000000000000000001+i for i in range(L_ARRAY)]
f32mMsg.data = [i for i in range(L_ARRAY)]
f64mMsg.data = [i for i in range(L_ARRAY)]

while not rospy.is_shutdown():
    # print(s64mMsg.data)

    pubU8M.publish(u8mMsg)
    pubS8M.publish(s8mMsg)
    pubU16M.publish(u16mMsg)
    pubS16M.publish(s16mMsg)
    pubU32M.publish(u32mMsg)
    pubS32M.publish(s32mMsg)
    pubU64M.publish(u64mMsg)
    pubS64M.publish(s64mMsg)
    pubF32M.publish(f32mMsg)
    pubF64M.publish(f64mMsg)

    for i in range(L_ARRAY):
        u8mMsg.data[i] = incrementWithLimit(u8mMsg.data[i], 255, 0)
        s8mMsg.data[i] = decrementWithLimit(s8mMsg.data[i], -127, 127)
        u16mMsg.data[i] = incrementWithLimit(u16mMsg.data[i], 65535, 0)
        s16mMsg.data[i] = decrementWithLimit(s16mMsg.data[i], -32767, 32767)
        u32mMsg.data[i] = incrementWithLimit(u32mMsg.data[i], 4294967295, 0)
        s32mMsg.data[i] = decrementWithLimit(s32mMsg.data[i], -2147483647, 2147483647)
        u64mMsg.data[i] = incrementWithLimit(u64mMsg.data[i], 18446744073709551615, 0)
        s64mMsg.data[i] = decrementWithLimit(s64mMsg.data[i], -9223372036854775808, 9223372036854775808)
        f32mMsg.data[i] = incrementWithLimit(f32mMsg.data[i], 10, -10, 0.1)
        f64mMsg.data[i] = decrementWithLimit(f64mMsg.data[i], -10, 10, 0.1)
    # exit(0)
    time.sleep(0.1)