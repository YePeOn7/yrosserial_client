#!/usr/bin/python3

from typing import List
import serial
import struct
import time
import rospy

class PubInfo:
    def __init__(self) -> None:
        self.topicName = None
        self.topicId = None
        self.type = None

class SubInfo:
    def __init__(self) -> None:
        self.topicName = None
        self.topicId = None
        self.type = None

class PacketRequestTopic:
    def __init__(self, header1 = 0x05, header2 = 0x09, length = 0x02, instruction = 0x01):
        self.header1 = header1
        self.header2 = header2
        self.length = length
        self.instruction = instruction
        self.checksum = self.length + self.instruction

    def serialize(self) -> bytes:
        return struct.pack("5b", self.header1, self.header2, self.length, self.instruction, self.checksum)

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

def processMessage(message):
    global subList
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
            subInfo.topicName = dt[5].decode()

            # print("--------- Subscribe ---------")
            # print(f"topicId     : {subInfo.topicId}")
            # print(f"type        : {subInfo.type}")
            # print(f"topicName   : {subInfo.topicName}")
            # print("--------------")
            print(f"Subscribe: {subInfo.topicName} with topicId: {subInfo.topicId}")
            subList.append(subInfo)

        if(dt[4] == 1): #Pub
            pubInfo = PubInfo()
            pubInfo.topicId = dt[2]
            pubInfo.type = dt[3]
            pubInfo.topicName = dt[5].decode()

            # print("--------- Publish ---------")
            # print(f"topicId     : {pubInfo.topicId}")
            # print(f"type        : {pubInfo.type}")
            # print(f"topicName   : {pubInfo.topicName}")
            # print("--------------")
            print(f"Publish: {pubInfo.topicName} with topicId: {pubInfo.topicId}")
            pubList.append(pubInfo)

        # print(dt)
    elif message[1] == 0x03:
        pass
    elif message[1] >= 10: #topicId
        pubInfo = PubInfo()
        pubInfo.topicId = message[1]
        messageType = message[2] ######################################### Consider to be remove, since this info can be obtained from publist

        for pub in pubList:
            if(pubInfo.topicId == pub.topicId):
                pubInfo = pub
                break

        if(pubInfo.type == None):
            return

        # calculate checksum
        checksum = 0
        for i in range(len(message) - 1):
            checksum += message[i]
        checksum &= 0xFF
        # print(f"calculated checksum: {checksum} --- obtained checksum: {message[-1]}")
        # print(f"----- get messageType : {messageType}")
        if(checksum == message[-1]):
            if(pubInfo.type == MessageType.String):
                #dt[0] --> length
                #dt[1] --> topicId
                #dt[2] --> messagetype
                #dt[3] --> string
                #dt[-1] --> checksum
                dt = struct.unpack(f"3B{message[0]-3}sB", message)
                strMessage = dt[3].decode()
                print(f"Get Message ({dt[1]}): {strMessage} --> {pubInfo.topicName}")
            elif(pubInfo.type == MessageType.Float32):
                # for i in message:
                #     print(i, end=" ")
                # print("")
                # print(f"len msg: {len(message)}")
                dt = struct.unpack(f"<3BfB", message)
                floatMsg = dt[3]
                print(f"Get Float32 Message: {floatMsg:.4f} --> {pubInfo.topicName}")
            elif(pubInfo.type == MessageType.Float64):
                # for i in message:
                #     print(i, end=" ")
                # print("")
                # print(f"len msg: {len(message)}")
                dt = struct.unpack(f"<3BdB", message)
                float64Msg = dt[3]
                print(f"Get Float64 Message: {float64Msg:.4f} --> {pubInfo.topicName}")
            elif(pubInfo.type == MessageType.Odometry2d):
                # for i in message:
                #     print(i, end=" ")
                # print("")
                # print(f"len msg: {len(message)}")
                dt = struct.unpack(f"<3B3fB", message)
                odometryMsgX = dt[3]
                odometryMsgY = dt[4]
                odometryMsgZ = dt[5]
                print(f"Get Odometry Message: x: {odometryMsgX:.4f}, y:{odometryMsgY:.4f}, z:{odometryMsgZ:.4f} --> {pubInfo.topicName}")
            elif(pubInfo.type == MessageType.Twist2d):
                # for i in message:
                #     print(i, end=" ")
                # print("")
                # print(f"len msg: {len(message)}")
                dt = struct.unpack(f"<3B3fB", message)
                twistMsgX = dt[3]
                twistMsgY = dt[4]
                twistMsgZ = dt[5]
                print(f"Get Twist Message: x: {twistMsgX:.4f}, y:{twistMsgY:.4f}, z:{twistMsgZ:.4f} --> {pubInfo.topicName}")


# ----------------- main process -------------------- #
subList:List[SubInfo] = []
pubList:List[PubInfo] = []

rospy.init_node("yrosserial_client")
baudrate = rospy.get_param("baudrate", 1000000)
port = rospy.get_param("port", "/dev/ttyACM0")

# Configure the serial port settings
try:
    serial_port = serial.Serial(port, baudrate=baudrate, timeout=1)
except Exception as e:
    # Handle other types of exceptions
    print(f"Error: {e}")
    exit(-1)

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
