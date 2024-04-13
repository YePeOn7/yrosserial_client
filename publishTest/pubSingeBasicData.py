import rospy
from std_msgs.msg import *
import time

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

rospy.init_node("pubIntTest")
pubU8 = rospy.Publisher("/sub_u8", UInt8, queue_size=10)
pubS8 = rospy.Publisher("/sub_s8", Int8, queue_size=10)
pubU16 = rospy.Publisher("/sub_u16", UInt16, queue_size=10)
pubS16 = rospy.Publisher("/sub_s16", Int16, queue_size=10)
pubU32 = rospy.Publisher("/sub_u32", UInt32, queue_size=10)
pubS32 = rospy.Publisher("/sub_s32", Int32, queue_size=10)
pubU64 = rospy.Publisher("/sub_u64", UInt64, queue_size=10)
pubS64 = rospy.Publisher("/sub_s64", Int64, queue_size=10)
pubF32 = rospy.Publisher("/sub_f32", Float32, queue_size=10)
pubF64 = rospy.Publisher("/sub_f64", Float64, queue_size=10)
time.sleep(0.1)

u8Msg = UInt8()
s8Msg = Int8()
u16Msg = UInt16()
s16Msg = Int16()
u32Msg = UInt32()
s32Msg = Int32()
u64Msg = UInt64()
s64Msg = Int64()
f32Msg = Float32()
f64Msg = Float64()

u8Msg.data = 123
s8Msg.data = -123
u16Msg.data = 28001
s16Msg.data = -28001
u32Msg.data =  1800000001
s32Msg.data = -1800000001
u64Msg.data =  8000000000000000001
s64Msg.data = -8000000000000000001
a = 1
while not rospy.is_shutdown():
    pubU8.publish(u8Msg)
    pubS8.publish(s8Msg)
    pubU16.publish(u16Msg)
    pubS16.publish(s16Msg)
    pubU32.publish(u32Msg)
    pubS32.publish(s32Msg)
    pubU64.publish(u64Msg)
    pubS64.publish(s64Msg)
    pubF32.publish(f32Msg)
    pubF64.publish(f64Msg)

    u8Msg.data = incrementWithLimit(u8Msg.data, 255, 0)
    s8Msg.data = decrementWithLimit(s8Msg.data, -127, 127)
    u16Msg.data = incrementWithLimit(u16Msg.data, 65535, 0)
    s16Msg.data = decrementWithLimit(s16Msg.data, -32767, 32767)
    u32Msg.data = incrementWithLimit(u32Msg.data, 4294967295, 0)
    s32Msg.data = decrementWithLimit(s32Msg.data, -2147483647, 2147483647)
    u64Msg.data = incrementWithLimit(u64Msg.data, 18446744073709551615, 0)
    s64Msg.data = decrementWithLimit(s64Msg.data, -9223372036854775808, 9223372036854775807)
    f32Msg.data = incrementWithLimit(f32Msg.data, 100, -100, 0.1)
    f64Msg.data = decrementWithLimit(f64Msg.data, -100, 100, 0.1)


    # print(f64Msg.data)
    # exit(0)
    time.sleep(0.1)