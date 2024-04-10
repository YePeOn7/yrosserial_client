#!/usr/bin/python3

from typing import Dict, Optional # Use as a hint for intellisense
import serial
import struct
import time
import rospy
from rospy.impl.tcpros import DEFAULT_BUFF_SIZE
from std_msgs.msg import String, Float32, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import math

HEADER = [5, 9]

rospy.init_node("yrosserial_client")
time.sleep(1)
baudrate = rospy.get_param("baudrate", 1000000)
port = rospy.get_param("port", "/dev/ttyACM0")

# Configure the serial port settings
try:
    serial_port = serial.Serial(port, baudrate=baudrate, timeout=1)
except Exception as e:
    # Handle other types of exceptions
    print(f"Error: {e}")
    exit(-1)

class ReceivingState:
    GET_HEADER1 = 0
    GET_HEADER2 = 1
    GET_LENGTH = 2
    GET_MESSAGE = 3

class MessageType:
    String = 0
    Float32 = 1
    Float64 = 2
    Odometry2d = 3
    Twist2d = 4

messageTypeMap = {
    MessageType.Float32: Float32,
    MessageType.Float64: Float64,
    MessageType.String: String,
    MessageType.Odometry2d: Odometry,
    MessageType.Twist2d: Twist
}

############### Callback ############
class StringCallback:
    def __init__(self, serial, id) -> None:
        self.header = HEADER
        self.serial = serial
        self.id = id

    def calculateChecksum(self, message, length):
        checksum = self.id + length
        for c in message:
            checksum += ord(c)
        return checksum & 0xFF 
    
    def serialize(self, msg) -> bytes:
        length = len(msg) + 4 # 4 from the following parameter: id, mt, null terminator, and checksum
        checksum = self.calculateChecksum(msg, length)
        bytesMsg = msg.encode() + b'\0' # add null terminator
        return struct.pack(f"5b{len(bytesMsg)}sB", self.header[0], self.header[1], length, self.id, MessageType.String, bytesMsg, checksum)

    def callback(self, msg):
        serializedData = self.serialize(msg.data)
        # print(msg.data)
        # print(serializedData.hex())
        serial_port.write(serializedData)

class Float32Callback:
    def __init__(self, serial, id) -> None:
        self.header = HEADER
        self.serial = serial
        self.id = id

    def calculateChecksum(self, message, length):
        checksum = self.id + length + MessageType.Float32
        for c in message:
            checksum += c
        return checksum & 0xFF 
    
    def serialize(self, msg) -> bytes:
        length =  7 # from the following parameter: id, mt, 4bytes data, and checksum
        packedMsg = struct.pack("f",msg)
        # print(packedMsg.hex(" "))
        checksum = self.calculateChecksum(packedMsg, length)
        bytesMsg = struct.pack(f"5B4sB", self.header[0], self.header[1], length, self.id, MessageType.Float32, packedMsg, checksum)
        return bytesMsg

    def callback(self, msg):
        serializedData = self.serialize(msg.data)
        # print(msg.data)
        # print(serializedData.hex(" "))
        serial_port.write(serializedData)

class Float64Callback:
    def __init__(self, serial, id) -> None:
        self.header = HEADER
        self.serial = serial
        self.id = id

    def calculateChecksum(self, message, length):
        checksum = self.id + length + MessageType.Float64
        for c in message:
            checksum += c
        return checksum & 0xFF 
    
    def serialize(self, msg) -> bytes:
        length =  11 # from the following parameter: id, mt, 8bytes data, and checksum
        packedMsg = struct.pack("d",msg)
        # print(packedMsg.hex(" "))
        checksum = self.calculateChecksum(packedMsg, length)
        bytesMsg = struct.pack(f"5B8sB", self.header[0], self.header[1], length, self.id, MessageType.Float64, packedMsg, checksum)
        return bytesMsg

    def callback(self, msg):
        serializedData = self.serialize(msg.data)
        # print(msg.data)
        # print(serializedData.hex(" "))
        serial_port.write(serializedData)

class Publisher(rospy.Publisher):
    def __init__(self, name, messageType : MessageType, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=10):
        if(messageType in messageTypeMap):
            super().__init__(name, messageTypeMap[messageType], subscriber_listener, tcp_nodelay, latch, headers, queue_size)
            self.messageType = messageType
        else:
            raise ValueError(f"Unsupported messageType: {messageType}")

    def publishCustom(self, message):
        if(self.messageType == MessageType.String):
            #dt[0] --> length
            #dt[1] --> topicId
            #dt[2] --> messagetype
            #dt[3] --> string
            #dt[-1] --> checksum
            dt = struct.unpack(f"3B{message[0]-3}sB", message)
            strMessage = dt[3][:-1].decode()
            self.publish(strMessage)
            # print(f"Get Message ({dt[1]}): {strMessage} --> {self.name}")
        elif(self.messageType == MessageType.Float32):
            # for i in message:
            #     print(i, end=" ")
            # print("")
            # print(f"len msg: {len(message)}")
            dt = struct.unpack(f"<3BfB", message)
            float32Msg = Float32()
            float32Msg.data = dt[3]
            self.publish(float32Msg)
            # print(f"Get Float32 Message: {floatMsg:.4f} --> {self.name}")
        elif(self.messageType == MessageType.Float64):
            # for i in message:
            #     print(i, end=" ")
            # print("")
            # print(f"len msg: {len(message)}")
            dt = struct.unpack(f"<3BdB", message)
            float64Msg = Float64()
            float64Msg.data = dt[3]
            self.publish(float64Msg)
            # print(f"Get Float64 Message: {float64Msg:.4f} --> {self.name}")
        elif(self.messageType == MessageType.Odometry2d):
            # for i in message:
            #     print(i, end=" ")
            # print("")
            # print(f"len msg: {len(message)}")
            dt = struct.unpack(f"<3B3fB", message)
            odometryMsgX = dt[3]
            odometryMsgY = dt[4]
            odometryMsgZ = dt[5]

            msg = Odometry()
            msg.pose.pose.position.x = odometryMsgX
            msg.pose.pose.position.y = odometryMsgY

            q = quaternion_from_euler(0, 0, math.radians(odometryMsgZ))
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]
            self.publish(msg)

            # print(f"Get Odometry Message: x: {odometryMsgX:.4f}, y:{odometryMsgY:.4f}, z:{odometryMsgZ:.4f} --> {self.name} -- {q}")
        elif(self.messageType == MessageType.Twist2d):
            # for i in message:
            #     print(i, end=" ")
            # print("")
            # print(f"len msg: {len(message)}")
            dt = struct.unpack(f"<3B3fB", message)
            twistMsgX = dt[3]
            twistMsgY = dt[4]
            twistMsgZ = dt[5]

            msg = Twist()
            msg.linear.x = twistMsgX
            msg.linear.y = twistMsgY
            msg.angular.z = twistMsgZ
            self.publish(msg)
            # print(f"Get Twist Message: x: {twistMsgX:.4f}, y:{twistMsgY:.4f}, z:{twistMsgZ:.4f} --> {self.name}")

class Subscriber():
    def __init__(self, topicName, topicId, messageType : MessageType, serial: serial.Serial):
        if(messageType in messageTypeMap):
            self.callback = None
            if(messageType == MessageType.String):
                self.callback = StringCallback(serial, topicId)
            elif(messageType == MessageType.Float32):
                self.callback = Float32Callback(serial, topicId)
            elif(messageType == MessageType.Float64):
                self.callback = Float64Callback(serial, topicId)
            elif(messageType == MessageType.Odometry2d):
                pass
            elif(messageType == MessageType.Twist2d):
                pass
            
            if(self.callback is not None):
                self.subscriber = rospy.Subscriber(topicName, messageTypeMap[messageType], self.callback.callback)
        else:
            raise ValueError(f"Unsupported messageType: {messageType}")

    

    def callback(msg):
        pass

class PubInfo:
    def __init__(self) -> None:
        self.topicName = None
        self.topicId = None
        self.type = None
        self.publisher : Optional[Publisher] = None

class SubInfo:
    def __init__(self) -> None:
        self.topicName = None
        self.topicId = None
        self.type = None
        self.subscriber : Optional[Subscriber] = None

class PacketRequestTopic:
    def __init__(self, header1 = 0x05, header2 = 0x09, length = 0x02, instruction = 0x01):
        self.header1 = header1
        self.header2 = header2
        self.length = length
        self.instruction = instruction
        self.checksum = self.length + self.instruction

    def serialize(self) -> bytes:
        return struct.pack("5b", self.header1, self.header2, self.length, self.instruction, self.checksum)

def messageTypeStr(messageType : MessageType):
    if(messageType == MessageType.Float32):
        return "Float32"
    elif(messageType == MessageType.Float64):
        return "Float64"
    elif(messageType == MessageType.String):
        return "String"
    elif(messageType == MessageType.Odometry2d):
        return "Odometry2d"
    elif(messageType == MessageType.Twist2d):
        return "Twist"

def processMessage(message):
    global subDict
    global pubDict
    messageLen = message[0]
    # check message[1] --> instruction / TopicId
    # no need to convert message[1] to int. When accessing the bytes variable by using []. it will automatically convert into int
    if(message[1] == 0x02): # response topic
        # print(f"Response Topic is received: {len(message)}. length: {message[0]}")
        # for i in message:
        #     print(i, end=" ")
        # print("")

        # calculate checksum
        checksum = 0
        for i in range(len(message) - 1):
            checksum += message[i]
        checksum &= 0xFF
        # print(f"Checksum: {checksum}")
        dt = struct.unpack(f"5B{message[0]-5}sB", message)

        if(dt[4] == 0): #Sub
            subInfo = SubInfo()
            subInfo.topicId = dt[2]
            subInfo.type = dt[3]
            subInfo.topicName = dt[5][:-1].decode() # need to remove null terminator before decode so it will be valid for rostopic name
            subInfo.subscriber = Subscriber(subInfo.topicName, subInfo.topicId, subInfo.type, serial_port)

            # print("--------- Subscribe ---------")
            # print(f"topicId     : {subInfo.topicId}")
            # print(f"type        : {subInfo.type}")
            # print(f"topicName   : {subInfo.topicName}")
            # print("--------------")
            print(f"Subscribe: {subInfo.topicName} ({messageTypeStr(subInfo.type)}) with topicId: {subInfo.topicId}")
            subDict[subInfo.topicId] = subInfo

        if(dt[4] == 1): #Pub
            pubInfo = PubInfo()
            pubInfo.topicId = dt[2]
            pubInfo.type = dt[3]
            pubInfo.topicName = dt[5][:-1].decode() # need to remove null terminator before decode so it will be valid for rostopic name
            pubInfo.publisher = Publisher(pubInfo.topicName, pubInfo.type)

            # print("--------- Publish ---------")
            # print(f"topicId     : {pubInfo.topicId}")
            # print(f"type        : {pubInfo.type}")
            # print(f"topicName   : {pubInfo.topicName}")
            # print("--------------")
            print(f"Publish: {pubInfo.topicName} ({messageTypeStr(pubInfo.type)}) with topicId: {pubInfo.topicId}")
            pubDict[pubInfo.topicId] = pubInfo # append to dict

        # print(dt)
    elif message[1] == 0x03:
        pass
    elif message[1] >= 10: #topicId
        topicId = message[1]
        if(topicId in pubDict):
            pubInfo = pubDict[topicId]
        else:
            return # the id hasn't been listed

        # calculate checksum
        checksum = 0
        for i in range(len(message) - 1):
            checksum += message[i]
        checksum &= 0xFF
        # print(f"calculated checksum: {checksum} --- obtained checksum: {message[-1]}")
        # print(f"----- get messageType : {messageType}")
        if(checksum == message[-1]):
            pubInfo.publisher.publishCustom(message)

# ----------------- main process -------------------- #
subDict:Dict[int, SubInfo] = {}
pubDict:Dict[int, PubInfo] = {}

rospy.loginfo("Requesting Topic.....")
pubLog = Publisher("/testX", MessageType.String)
packetRequestTopic = PacketRequestTopic()
data = packetRequestTopic.serialize()
serial_port.write(data)
time.sleep(0.01)

receivingState = ReceivingState.GET_HEADER1
messageLength = 0
messageCnt = 0
message = b''

while not rospy.is_shutdown():
    while(serial_port.in_waiting > 0):
        rxData = serial_port.read(1)
        # print(f"received data: {rxData} -- ", end= "")
        # continue
        if(receivingState == ReceivingState.GET_HEADER1):
            if(rxData == b'\x05'):
                # print("Header 1 is received")
                receivingState = ReceivingState.GET_HEADER2
        elif(receivingState == ReceivingState.GET_HEADER2):
            if(rxData == b'\x05'):  receivingState = ReceivingState.GET_HEADER2
            elif(rxData == b'\x09'):
                receivingState = ReceivingState.GET_LENGTH
                # print("Header 2 is received")
            else: receivingState = ReceivingState.GET_HEADER1
        elif(receivingState == ReceivingState.GET_LENGTH):
            messageLength = int.from_bytes(rxData, 'little')
            # print(f"data length: {messageLength} ({rxData})")
            message = rxData #start message with the length data
            messageCnt = 0
            receivingState = ReceivingState.GET_MESSAGE
        elif(receivingState == ReceivingState.GET_MESSAGE):
            # print(f"m[{messageCnt}]: {rxData}")
            # print("getting messages")
            message += rxData
            messageCnt += 1

            if(messageCnt == messageLength):
                # process
                # print(message)
                # print(message[0], rxData[0])
                processMessage(message)

                # reset the state
                messageCnt = 0
                receivingState = ReceivingState.GET_HEADER1

    time.sleep(0.001)

serial_port.close()
