#!/usr/bin/python3

from typing import Dict, Optional # Use as a hint for intellisense
import serial
import struct
import time
import rospy
from rospy.impl.tcpros import DEFAULT_BUFF_SIZE
from std_msgs.msg import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
import traceback

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
    UInt8 = 5
    Int8 = 6
    UInt16 = 7
    Int16 = 8
    UInt32 = 9
    Int32 = 10
    UInt64 = 11
    Int64 = 12
    UInt8MultiArray = 13
    Int8MultiArray = 14
    UInt16MultiArray = 15
    Int16MultiArray = 16
    UInt32MultiArray = 17
    Int32MultiArray = 18
    UInt64MultiArray = 19
    Int64MultiArray = 20
    Float32MultiArray = 21
    Float64MultiArray = 22
    
messageTypeMap = {
    MessageType.Float32: Float32,
    MessageType.Float64: Float64,
    MessageType.String: String,
    MessageType.Odometry2d: Odometry,
    MessageType.Twist2d: Twist,
    MessageType.UInt8: UInt8,
    MessageType.Int8: Int8,
    MessageType.UInt16: UInt16,
    MessageType.Int16: Int16,
    MessageType.UInt32: UInt32,
    MessageType.Int32: Int32,
    MessageType.UInt64: UInt64,
    MessageType.Int64: Int64,
    MessageType.UInt8MultiArray: UInt8MultiArray,
    MessageType.Int8MultiArray: Int8MultiArray,
    MessageType.UInt16MultiArray: UInt16MultiArray,
    MessageType.Int16MultiArray: Int16MultiArray,
    MessageType.UInt32MultiArray: UInt32MultiArray,
    MessageType.Int32MultiArray: Int32MultiArray,
    MessageType.UInt64MultiArray: UInt64MultiArray,
    MessageType.Int64MultiArray: Int64MultiArray,
    MessageType.Float32MultiArray: Float32MultiArray,
    MessageType.Float64MultiArray: Float64MultiArray,
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

class Odometry2dCallback:
    def __init__(self, serial, id) -> None:
        self.header = HEADER
        self.serial = serial
        self.id = id

    def calculateChecksum(self, message, length):
        checksum = self.id + length + MessageType.Odometry2d
        for c in message:
            checksum += c
        return checksum & 0xFF 
    
    def serialize(self, msg : Odometry) -> bytes:
        length =  15 # from the following parameter: id, mt, 12bytes data, and checksum

        q = (msg.pose.pose.orientation.x, 
         msg.pose.pose.orientation.y, 
         msg.pose.pose.orientation.z, 
         msg.pose.pose.orientation.w)

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        [_, _, w] = euler_from_quaternion(q)
        w = math.degrees(w)

        packedMsg = struct.pack("3f",x, y, w)
        # print(packedMsg.hex(" "))
        checksum = self.calculateChecksum(packedMsg, length)
        bytesMsg = struct.pack(f"5B12sB", self.header[0], self.header[1], length, self.id, MessageType.Odometry2d, packedMsg, checksum)
        return bytesMsg

    def callback(self, msg):
        serializedData = self.serialize(msg)
        # print(msg.data)
        # print(serializedData.hex(" "))
        serial_port.write(serializedData)

class Twist2dCallback:
    def __init__(self, serial, id) -> None:
        self.header = HEADER
        self.serial = serial
        self.id = id

    def calculateChecksum(self, message, length):
        checksum = self.id + length + MessageType.Twist2d
        for c in message:
            checksum += c
        return checksum & 0xFF 
    
    def serialize(self, msg:Twist) -> bytes:
        length =  15 # from the following parameter: id, mt, 12bytes data, and checksum

        x = msg.linear.x
        y = msg.linear.y
        w = msg.angular.z

        packedMsg = struct.pack("3f",x, y, w)
        # print(packedMsg.hex(" "))
        checksum = self.calculateChecksum(packedMsg, length)
        bytesMsg = struct.pack(f"5B12sB", self.header[0], self.header[1], length, self.id, MessageType.Twist2d, packedMsg, checksum)
        return bytesMsg

    def callback(self, msg:Twist):
        serializedData = self.serialize(msg)
        # print(msg.data)
        # print(serializedData.hex(" "))
        serial_port.write(serializedData)

class Publisher(rospy.Publisher):
    def __init__(self, name, messageType : MessageType, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=10):
        if(messageType in messageTypeMap):
            super().__init__(name, messageTypeMap[messageType], subscriber_listener, tcp_nodelay, latch, headers, queue_size)
            self.messageType = messageType
            self.arrayType = [getattr(MessageType, i) for i in dir(MessageType) if ("MultiArray" in i)]
            self.nonArrayType = [getattr(MessageType, i) for i in dir(MessageType) if ("MultiArray" not in i and ("Int" in i or "Float" in i))]

            self.mapFormat = {
                MessageType.Float32: "f",
                MessageType.Float64: "d",
                MessageType.String: "s",
                MessageType.Odometry2d: "f",
                MessageType.Twist2d: "f",
                MessageType.UInt8: "B",
                MessageType.Int8: "b",
                MessageType.UInt16: "H",
                MessageType.Int16: "h",
                MessageType.UInt32: "I",
                MessageType.Int32: "i",
                MessageType.UInt64: "Q",
                MessageType.Int64: "q",
                MessageType.UInt8MultiArray: "B",
                MessageType.Int8MultiArray: "b",
                MessageType.UInt16MultiArray: "H",
                MessageType.Int16MultiArray: "h",
                MessageType.UInt32MultiArray: "I",
                MessageType.Int32MultiArray: "i",
                MessageType.UInt64MultiArray: "Q",
                MessageType.Int64MultiArray: "q",
                MessageType.Float32MultiArray: "f",
                MessageType.Float64MultiArray: "d",
            }
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
            try:
                strMessage = dt[3][:-1].decode()
            except UnicodeDecodeError as e:
                print(f"Error decoding byte sequence: {e}")
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
        elif(self.messageType in self.arrayType):
            (length,) = struct.unpack(f"<H", message[3:5])
            byteArraySize = message[0] - 5 # length packet - 5, 5 is lengthInfo, topicId, MessageType and 2 bytes of length info or array
            try:
                data = struct.unpack(f"{length}{self.mapFormat[self.messageType]}", message[5:5+(byteArraySize)])
                msg = messageTypeMap[self.messageType]()
                msg.data = data
                self.publish(msg)
            except Exception as e:
                traceback.print_exc()
                print(e)
        elif(self.messageType in self.nonArrayType):
            byteArraySize = message[0] - 3 # length packet - 3, 3 is lengthInfo, topicId, and MessageType
            try:
                (data,) = struct.unpack(f"{self.mapFormat[self.messageType]}", message[3:3+(byteArraySize)])
                msg = messageTypeMap[self.messageType]()
                msg.data = data
                self.publish(msg)
            except Exception as e:
                traceback.print_exc()
                print(e)
                
class Subscriber():
    def __init__(self, topicName, topicId, messageType : MessageType, serial: serial.Serial):
        self.messageType = messageType
        self.serial = serial
        self.topicId = topicId
        self.arrayType = [getattr(MessageType, i) for i in dir(MessageType) if ("MultiArray" in i)]
        self.nonArrayType = [getattr(MessageType, i) for i in dir(MessageType) if ("MultiArray" not in i and ("Int" in i or "Float" in i))]
        self.mapFormat = {
            MessageType.Float32: "f",
            MessageType.Float64: "d",
            MessageType.String: "s",
            MessageType.Odometry2d: "f",
            MessageType.Twist2d: "f",
            MessageType.UInt8: "B",
            MessageType.Int8: "b",
            MessageType.UInt16: "H",
            MessageType.Int16: "h",
            MessageType.UInt32: "I",
            MessageType.Int32: "i",
            MessageType.UInt64: "Q",
            MessageType.Int64: "q",
            MessageType.UInt8MultiArray: "B",
            MessageType.Int8MultiArray: "b",
            MessageType.UInt16MultiArray: "H",
            MessageType.Int16MultiArray: "h",
            MessageType.UInt32MultiArray: "I",
            MessageType.Int32MultiArray: "i",
            MessageType.UInt64MultiArray: "Q",
            MessageType.Int64MultiArray: "q",
            MessageType.Float32MultiArray: "f",
            MessageType.Float64MultiArray: "d",
        }
        self.format = self.mapFormat[self.messageType]
        if(messageType in messageTypeMap):
            if(messageType in self.nonArrayType or messageType in self.arrayType):
                self.subscriber = rospy.Subscriber(topicName, messageTypeMap[messageType], self.callback)
            else:
                self.callback = None
                if(messageType == MessageType.String):
                    self.callback = StringCallback(serial, topicId)
                elif(messageType == MessageType.Odometry2d):
                    self.callback = Odometry2dCallback(serial, topicId)
                elif(messageType == MessageType.Twist2d):
                    self.callback = Twist2dCallback(serial, topicId)
            
                if(self.callback is not None):
                    self.subscriber = rospy.Subscriber(topicName, messageTypeMap[messageType], self.callback.callback)
        else:
            raise ValueError(f"Unsupported messageType: {messageType}")
        
    def calculateChecksum(self, data, l_packet):
        sum = self.messageType + self.topicId + l_packet
        for i in data:
            sum += i
        return sum & 0xFF

    def callback(self, msg):
        if(self.messageType in self.nonArrayType):
            d = struct.pack(f"<{self.format}", msg.data)
            l_packet = len(d) + 3
            checksum = self.calculateChecksum(d, l_packet)

            serializedData = struct.pack(f"5B{len(d)}sB", *HEADER, l_packet, self.topicId, self.messageType, d, checksum)
            # print("non array -->", serializedData.hex(" "))
            self.serial.write(serializedData)
        elif(self.messageType in self.arrayType):
            l = len(msg.data)
            d = struct.pack(f"<H{l}{self.format}", l, *msg.data)
            l_packet = len(d) + 3
            checksum = self.calculateChecksum(d, l_packet)
            serializedData = struct.pack(f"5B{len(d)}sB", *HEADER, l_packet, self.topicId, self.messageType, d, checksum)
            # print("array -->", serializedData.hex(" "))
            self.serial.write(serializedData)

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
    attrs = [i for i in dir(MessageType) if (not callable(i) and not i.startswith("__"))]
    mt = {}
    for i in attrs:
        mt[getattr(MessageType, i)] = i

    if(messageType in mt):
        return mt[messageType]
    else:
        return "undefined"

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

# Clean rx buffer before start
while(serial_port.in_waiting):
    serial_port.read()
    
rospy.loginfo("Requesting Topic.....")
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
