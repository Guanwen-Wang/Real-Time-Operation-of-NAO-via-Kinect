#!/usr/bin/env python
import socket
import time
from time import ctime
import naoqi
from naoqi import ALProxy
import almath
 
server = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
server.bind(("127.0.0.1",8888))
print("udp server start")

dcm = ALProxy("DCM","192.168.191.3",9559)
alautonomousProxy = ALProxy("ALAutonomousLife","192.168.191.3",9559)
motionProxy  = ALProxy("ALMotion","192.168.191.3",9559)
postureProxy = ALProxy("ALRobotPosture","192.168.191.3",9559)
RhandName  = 'RHand'
LhandName  = 'LHand'

if alautonomousProxy.getState()=="solitary" or alautonomousProxy.getState() == "interactive":
    alautonomousProxy.setState("disabled")
    
motionProxy.wakeUp()
pChainName="Arms"
pEnable=True
isEnabled = True
motionProxy.wbEnable(isEnabled)

# Legs are constrained in a plane
stateName  = "Fixed"
supportLeg = "Legs"
motionProxy.wbFootState(stateName, supportLeg)

# Constraint Balance Motion
isEnable   = True
supportLeg = "Legs"
motionProxy.wbEnableBalanceConstraint(isEnable, supportLeg)

success = motionProxy.setCollisionProtectionEnabled(pChainName, pEnable)
if (not success):
    print("Failed to enable collision protection")
#postureProxy.goToPosture("Stand", 0.5)
time.sleep(1)

dcm.createAlias([
"UpHalfBody",
[
"HeadPitch/Position/Actuator/Value",
"HeadYaw/Position/Actuator/Value",
"LShoulderRoll/Position/Actuator/Value",
"LShoulderPitch/Position/Actuator/Value",
"LElbowRoll/Position/Actuator/Value",
"LElbowYaw/Position/Actuator/Value",
"LWristYaw/Position/Actuator/Value",
"RShoulderRoll/Position/Actuator/Value",
"RShoulderPitch/Position/Actuator/Value",
"RElbowRoll/Position/Actuator/Value",
"RElbowYaw/Position/Actuator/Value",
"RWristYaw/Position/Actuator/Value",
]
])
#dcm.createAlias([
#"DownHalfBody",
#[
#"LHipPitch/Position/Actuator/Value",
#"LKneePitch/Position/Actuator/Value",
#"RHipPitch/Position/Actuator/Value",
#"RKneePitch/Position/Actuator/Value",
#]
#])
dcm.createAlias([
"UpHalfBodyStiffness",
[
"HeadPitch/Hardness/Actuator/Value",
"HeadYaw/Hardness/Actuator/Value",
"LShoulderRoll/Hardness/Actuator/Value",
"LShoulderPitch/Hardness/Actuator/Value",
"LElbowRoll/Hardness/Actuator/Value",
"LElbowYaw/Hardness/Actuator/Value",
"LWristYaw/Hardness/Actuator/Value",
"RShoulderRoll/Hardness/Actuator/Value",
"RShoulderPitch/Hardness/Actuator/Value",
"RElbowRoll/Hardness/Actuator/Value",
"RElbowYaw/Hardness/Actuator/Value",
"RWristYaw/Hardness/Actuator/Value",
"LHipPitch/Hardness/Actuator/Value",
"LKneePitch/Hardness/Actuator/Value",
"RHipPitch/Hardness/Actuator/Value",
"RKneePitch/Hardness/Actuator/Value",
]
])

data=0;
dcm.setAlias(["UpHalfBodyStiffness","ClearAll","time-mixed",[[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]],[[1.0, dcm.getTime(0)]]]])


while True:
    data,addr = server.recvfrom(1024)
    if data == "quit":
        dcm.setAlias(["UpHalfBodyStiffness","ClearAll","time-mixed",[[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]],[[0.0, dcm.getTime(0)]]]])
        postureProxy.goToPosture("Stand", 0.2)
        break
    if data == "               ":
        continue
    text = data
    order=text.split(" ")
    dcm.setAlias(["UpHalfBody","ClearAll","time-mixed",[[[float(order[0])*almath.TO_RAD, dcm.getTime(1000)]],[[float(order[1])*almath.TO_RAD, dcm.getTime(1000)]],[[float(order[2])*almath.TO_RAD, dcm.getTime(1000)]],[[float(order[3])*almath.TO_RAD, dcm.getTime(1000)]],[[float(order[4])*almath.TO_RAD, dcm.getTime(1000)]],[[float(order[5])*almath.TO_RAD, dcm.getTime(1000)]],[[float(order[6])*almath.TO_RAD, dcm.getTime(1000)]],[[float(order[7])*almath.TO_RAD, dcm.getTime(1000)]],[[float(order[8])*almath.TO_RAD, dcm.getTime(1000)]],[[float(order[9])*almath.TO_RAD, dcm.getTime(1000)]],[[float(order[10])*almath.TO_RAD, dcm.getTime(1000)]],[[float(order[11])*almath.TO_RAD, dcm.getTime(1000)]]]])
    if order[17]=="Closed":
        motionProxy.closeHand(RhandName)
    if order[17]=="Open":
        motionProxy.openHand(RhandName)
    if order[16]=="Closed":
        motionProxy.closeHand(LhandName)
    if order[16]=="Open":
        motionProxy.openHand(LhandName)    
    print addr
    print text

print("udp server finish")
server.close()




 
