
# producer_consumer_queue  

from Queue import Queue  

import random  

import threading  

import time   

import multiprocessing

import socket

from time import ctime

import naoqi

from naoqi import ALProxy

import almath

import qi
import argparse
import sys
import motion


order=['0','1','2','3','4','5','6','7','8','9','10','11','12','13','14','15','16','17','18','19','20','21','22','23','24']
#order[0]&order[1]
#order[2]-LShoulderRoll
#order[3]-LShoulderPitch
#order[4]-LElbowRoll
#order[5]-LElbowYaw
#order[6]-LWristYaw
#order[7]-RShoulderRoll
#order[8]-RShoulderPitch
#order[9]-RElbowRoll
#order[10]-RElbowYaw
#order[11]-RWristYaw
#order[12]-LHipRoll
#order[13]-LKneePitch
#order[14]-RHipRoll
#order[15]-RKneePitch
#order[16]-LHand
#order[17]-RHand
#order[18]-LHipPitch
#order[19]-RHipPitch
#order[20]-LAnklePitch
#order[21]-RAnklePitch
#order[22]-LHipYawPitch
#order[23]-RHipYawPitch
#order[24]-LAnkleRoll
#order[25]-RAnkleRoll

ip="192.168.191.2"
dcm = ALProxy("DCM",ip ,9559)
alautonomousProxy = ALProxy("ALAutonomousLife",ip ,9559)
motionProxy  = ALProxy("ALMotion",ip ,9559)
postureProxy = ALProxy("ALRobotPosture",ip ,9559)
RhandName  = 'RHand'
LhandName  = 'LHand'

motionProxy.rest()
if alautonomousProxy.getState()=="solitary" or alautonomousProxy.getState() == "interactive":
    alautonomousProxy.setState("disabled")
postureProxy.goToPosture("Stand", 0.2)  
postureProxy.goToPosture("StandInit", 0.2)
pChainName="Arms"
pEnable=True
#isEnabled = True
#motionProxy.wbEnable(isEnabled)

# Legs are constrained in a plane

#stateName  = "Fixed"
#supportLeg = "Legs"
#motionProxy.wbFootState(stateName, supportLeg)

# Constraint Balance Motion

#isEnable   = True
#supportLeg = "Legs"
#motionProxy.wbEnableBalanceConstraint(isEnable, supportLeg)

success = motionProxy.setCollisionProtectionEnabled(pChainName, pEnable)
if (not success):
    print("Failed to enable collision protection")
postureProxy.goToPosture("Stand", 0.5)
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
"LHand/Position/Actuator/Value",
"RHand/Position/Actuator/Value"
]
])
dcm.createAlias([
"DownHalfBody",
[
"LHipRoll/Position/Actuator/Value",
"LKneePitch/Position/Actuator/Value",
"RHipRoll/Position/Actuator/Value",
"RKneePitch/Position/Actuator/Value",
"LHipPitch/Position/Actuator/Value",
"RHipPitch/Position/Actuator/Value",
"LAnklePitch/Position/Actuator/Value",
"RAnklePitch/Position/Actuator/Value",
"LHipYawPitch/Position/Actuator/Value",
"RHipYawPitch/Position/Actuator/Value",
"LAnkleRoll/Position/Actuator/Value",
"RAnkleRoll/Position/Actuator/Value"
]
])
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
"LHipRoll/Hardness/Actuator/Value",
"LKneePitch/Hardness/Actuator/Value",
"RHipRoll/Hardness/Actuator/Value",
"RKneePitch/Hardness/Actuator/Value",
"LHand/Hardness/Actuator/Value",
"RHand/Hardness/Actuator/Value",
"LHipPitch/Hardness/Actuator/Value",
"RHipPitch/Hardness/Actuator/Value",
"LAnklePitch/Hardness/Actuator/Value",
"RAnklePitch/Hardness/Actuator/Value",
"LHipYawPitch/Hardness/Actuator/Value",
"RHipYawPitch/Hardness/Actuator/Value",
"LAnkleRoll/Hardness/Actuator/Value",
"RAnkleRoll/Hardness/Actuator/Value"
]
])

data=0;
dcm.setAlias(["UpHalfBodyStiffness","ClearAll","time-mixed",[[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.5, dcm.getTime(0)]],
                                                            [[0.7, dcm.getTime(0)]],
                                                            [[0.7, dcm.getTime(0)]]])

#Producer thread  


class Producer(threading.Thread):  

    def __init__(self, t_name, queue):  
        threading.Thread.__init__(self, name=t_name)  
        self.data=queue  
        self.lock = threading.Lock()

    def run(self):  
        server = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        server.bind(("127.0.0.1",8888))
        print("udp server start")
        
        while True:
            text,addr = server.recvfrom(1024)
            print text
            print addr
            if text == "quit":            
                print("udp server finish")
                server.close()  
                self.lock.acquire()
                if self.data.empty():
                    for j in range(25):
                        self.data.put('quit')
                        print " producing %s \n" %('quit')
                else:
                    while not self.data.empty():
                        sss=self.data.get()  
                        print " %s erasing  %s \n " %( self.getName(),'quit')
                    for j in range(25):
                        self.data.put('quit')  
                        print " producing %s \n " %('quit')
                self.lock.release()
                break
            if text == "               ":
                continue
            order=text.split(" ")


    #print "%s: %s is producing %d to the queue!\n" %(time.ctime(), self.getName(), i)  
            self.lock.acquire() #º”…œÀ¯
            if self.data.empty():
                for j in range(25):
                    self.data.put(order[j])
                    print " producing %s \n" %(order[j])
            else:
                while not self.data.empty():
                    sss=self.data.get()  
                    print " %s erasing  %s \n " %( self.getName(),sss)
                for j in range(25):
                    self.data.put(order[j])  
                    print " producing %s \n " %(order[j])
            self.lock.release()
            time.sleep(0.03)
        time.sleep(3)
        print "%s: %s finished!" %(time.ctime(), self.getName())  



#Consumer thread  

class Consumer(threading.Thread):  
    def __init__(self, t_name, queue):  
        threading.Thread.__init__(self, name=t_name)  
        self.data=queue  
        self.lock = threading.Lock()
        
    def run(self):
     
        parser = argparse.ArgumentParser()
        parser.add_argument("--ip", type=str, default=ip,
                            help="Robot IP address. On robot or Local Naoqi: use ip.")
        parser.add_argument("--port", type=int, default=9559,
                            help="Naoqi port number")
        args = parser.parse_args()
        session = qi.Session()
        try:
            session.connect("tcp://" + args.ip + ":" + str(args.port))
        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
                   "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)
            
        motion_service  = session.service("ALMotion")         
        name1 = "Body"
        name2 = "LAnkleRoll"
        name3 = "RAnkleRoll"
        frame = motion.FRAME_ROBOT
        useSensors = True
        useSensorValues = True
        #------------
        while True:
            self.lock.acquire()
            if self.data.full():    
                for j in range(25):                
                    val = self.data.get() 
                    if val=='quit':
                        #postureProxy.goToPosture("StandInit", 0.2)
                        #dcm.setAlias(["UpHalfBodyStiffness","ClearAll","time-mixed",[[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]]]])
                        #print "%s: %s is consuming. %s in the queue is consumed!\n" %(time.ctime(), self.getName(), val)  
                        motionProxy.rest()
                        break; 
                    order[j]=val
                self.lock.release()
            else:
                self.lock.release()
                time.sleep(0.1)
                continue
            dcm.setAlias(["UpHalfBody","ClearAll","time-mixed",[[[float(order[0])*almath.TO_RAD, dcm.getTime(750)]],[[float(order[1])*almath.TO_RAD, dcm.getTime(750)]],[[float(order[2])*almath.TO_RAD, dcm.getTime(750)]],[[float(order[3])*almath.TO_RAD, dcm.getTime(750)]],[[float(order[4])*almath.TO_RAD, dcm.getTime(750)]],[[float(order[5])*almath.TO_RAD, dcm.getTime(750)]],[[float(order[6])*almath.TO_RAD, dcm.getTime(750)]],[[float(order[7])*almath.TO_RAD, dcm.getTime(750)]],[[float(order[8])*almath.TO_RAD, dcm.getTime(750)]],[[float(order[9])*almath.TO_RAD, dcm.getTime(750)]],[[float(order[10])*almath.TO_RAD, dcm.getTime(750)]],[[float(order[11])*almath.TO_RAD, dcm.getTime(750)]],[[float(order[16]), dcm.getTime(750)]],[[float(order[17]), dcm.getTime(750)]]]])      
            dcm.setAlias(["DownHalfBody","ClearAll","time-mixed",[[[float(order[12])*almath.TO_RAD, dcm.getTime(750)]],
                                                                  [[float(order[13])*almath.TO_RAD, dcm.getTime(750)]],
                                                                  [[float(order[14])*almath.TO_RAD, dcm.getTime(750)]],
                                                                  [[float(order[15])*almath.TO_RAD, dcm.getTime(750)]],
                                                                  [[float(order[18])*almath.TO_RAD, dcm.getTime(750)]],
                                                                  [[float(order[19])*almath.TO_RAD, dcm.getTime(750)]],
                                                                  [[float(order[20])*almath.TO_RAD, dcm.getTime(750)]],
                                                                  [[float(order[21])*almath.TO_RAD, dcm.getTime(750)]],
                                                                  [[float(order[22])*almath.TO_RAD, dcm.getTime(750)]],
                                                                  [[float(order[22])*almath.TO_RAD, dcm.getTime(750)]],
                                                                  [[float(order[23])*almath.TO_RAD, dcm.getTime(750)]],
                                                                  [[float(order[24])*almath.TO_RAD, dcm.getTime(750)]]
                                                                  ]])
     
            f0 = file("H:\\angleList.txt","a+")
            aaa = motion_service.getAngles("RKneePitch", useSensors)
            bbb = motion_service.getAngles("RHipRoll", useSensors)
            ccc = motion_service.getAngles("LShoulderPitch",useSensors)
            ddd = motion_service.getAngles("RShoulderPitch",useSensors)
            b=[str(aaa),",",str(bbb),",",str(ccc),",",str(ddd),"\n"]
            f0.writelines(b)
            f0.close()

            f4 = file("H:\\angleOrder.txt","a+")
            c=[str(float(order[15])*almath.TO_RAD),",",str(float(order[14])*almath.TO_RAD),",",str(float(order[3])*almath.TO_RAD),",",str(float(order[8])*almath.TO_RAD),"\n"]
            f4.writelines(c)
            f4.close()            
     
            pos = motion_service.getCOM(name1, frame, useSensors)
            f1 = file("H:\\CoM.txt", "a+")
            a = [str(pos[0]),",",str(pos[1]),",",str(pos[2]),"\n"]
            f1.writelines(a)
            f1.close()
 
            pos1 = motion_service.getPosition(name2, frame, useSensorValues)
            f2 = file("H:\\LAnkleRoll.txt", "a+")
            f2.writelines(str(pos1))
            f2.writelines("\n")
            f2.close()
      
            pos2 = motion_service.getPosition(name3, frame, useSensorValues)
            f3 = file("H:\\RAnkleRoll.txt", "a+")
            f3.writelines(str(pos2))
            f3.writelines("\n")
            f3.close()
            
        postureProxy.goToPosture("StandInit", 0.2)
        motionProxy.rest()
        print "%s: %s finished!" %(time.ctime(), self.getName())  



#Main thread  

def main():  



    queue = Queue(25)  

    producer = Producer('Pro.', queue)  

    consumer = Consumer('Con.', queue)  

    producer.start()  

    consumer.start()  

    producer.join()  

    consumer.join()  

    postureProxy.goToPosture("StandInit", 0.2)
    print 'All threads terminate!'  



if __name__ == '__main__':  

    main()  
