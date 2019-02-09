 #-*- coding:utf-8 -*-
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

       
#机器人控制部分↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
order=['0','1','2','3','4','5','6','7','8','9','10','11','12','13','14','15','16','17','18','19','20','21','22','23']
#order[0]&order[1] 头部
#order[2]-LShoulderRoll
#order[3]-ShoulderPitch
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
#order[20]-LAnkleRoll
#order[21]-RAnkleRoll
#order[22]-LAnklePitch
#order[23]-RAnklePitch

ip="172.26.161.2"
dcm = ALProxy("DCM",ip,9559)
alautonomousProxy = ALProxy("ALAutonomousLife",ip,9559)
motionProxy  = ALProxy("ALMotion",ip,9559)
postureProxy = ALProxy("ALRobotPosture",ip,9559)
RhandName  = 'RHand'
LhandName  = 'LHand'

motionProxy.rest()
if alautonomousProxy.getState()=="solitary" or alautonomousProxy.getState() == "interactive":
    alautonomousProxy.setState("disabled")
postureProxy.goToPosture("Stand", 0.2)  
postureProxy.goToPosture("StandInit", 0.2)
pChainName="Arms"
pEnable=True

# Activate Whole Body Balancer
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
"RHipPitch/Position/Actuator/Value"
"LAnkleRoll/Position/Actuator/Value",
"RAnkleRoll/Position/Actuator/Value",
"LAnklePitch/Position/Actuator/Value",
"RAnklePitch/Position/Actuator/Value"
]
])
dcm.createAlias([
"BodyStiffness",
[
"HeadPitch/Hardness/Actuator/Value", #0
"HeadYaw/Hardness/Actuator/Value",   #1
"LShoulderRoll/Hardness/Actuator/Value",  #2
"LShoulderPitch/Hardness/Actuator/Value", #3
"LElbowRoll/Hardness/Actuator/Value",  #4
"LElbowYaw/Hardness/Actuator/Value",  #5
"LWristYaw/Hardness/Actuator/Value",  #6
"RShoulderRoll/Hardness/Actuator/Value",  #7
"RShoulderPitch/Hardness/Actuator/Value", #8
"RElbowRoll/Hardness/Actuator/Value",  #9
"RElbowYaw/Hardness/Actuator/Value",   #10
"RWristYaw/Hardness/Actuator/Value",   #11
"LHipRoll/Hardness/Actuator/Value",    #12
"LKneePitch/Hardness/Actuator/Value",  #13
"RHipRoll/Hardness/Actuator/Value",    #14
"RKneePitch/Hardness/Actuator/Value",  #15
"LHand/Hardness/Actuator/Value",       #16
"RHand/Hardness/Actuator/Value",       #17
"LHipPitch/Hardness/Actuator/Value",   #18
"RHipPitch/Hardness/Actuator/Value",     #19
"LAnkleRoll/Hardness/Actuator/Value",  #20
"RAnkleRoll/Hardness/Actuator/Value",  #21
"LAnklePitch/Hardness/Actuator/Value", #22
"RAnklePitch/Hardness/Actuator/Value"  #23
]
])

data=0;
dcm.setAlias(["BodyStiffness","ClearAll","time-mixed",[[[0.5, dcm.getTime(0)]],  #1
                                                             [[0.5, dcm.getTime(0)]],  #2
                                                             [[0.5, dcm.getTime(0)]],  #3
                                                             [[0.5, dcm.getTime(0)]],  #4
                                                             [[0.5, dcm.getTime(0)]],  #5
                                                             [[0.5, dcm.getTime(0)]],  #6
                                                             [[0.5, dcm.getTime(0)]],  #7
                                                             [[0.5, dcm.getTime(0)]],  #8 
                                                             [[0.5, dcm.getTime(0)]],  #9
                                                             [[0.5, dcm.getTime(0)]],  #10
                                                             [[0.5, dcm.getTime(0)]],  #11
                                                             [[0.5, dcm.getTime(0)]],  #12
                                                             [[0.5, dcm.getTime(0)]],  #13
                                                             [[0.5, dcm.getTime(0)]],  #14
                                                             [[0.5, dcm.getTime(0)]],  #15
                                                             [[0.5, dcm.getTime(0)]],  #16
                                                             [[0.5, dcm.getTime(0)]],  #17
                                                             [[0.5, dcm.getTime(0)]],  #18
                                                             [[0.5, dcm.getTime(0)]],   #19
                                                             [[0.8, dcm.getTime(0)]],  #20
                                                             [[0.8, dcm.getTime(0)]],  #21
                                                             [[0.8, dcm.getTime(0)]],  #22
                                                             [[0.8, dcm.getTime(0)]]   #23
                                                             ]])

#机器人初始化部分  完↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
  
#Producer thread  

  
class Producer(threading.Thread):  
  
    def __init__(self, t_name, queue):  
  
        threading.Thread.__init__(self, name=t_name)  
  
        self.data=queue  
        self.lock = threading.Lock()
#生产者建立连接初始化
    def run(self):  
        server = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        server.bind(("127.0.0.1",8888))
        print("udp server start")          
#从客户端接收数据↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 
        while True:
            text,addr = server.recvfrom(1024)
            print text
            print addr
            if text == "quit":            
                print("udp server finish")
                server.close()  
                
                self.lock.acquire() #加上锁
                if self.data.empty():
                    for j in range(24):
                        self.data.put('quit')
                        print " producing %s \n" %('quit')
                else:
                    while not self.data.empty():
                        sss=self.data.get()  
                        print " %s erasing  %s \n " %( self.getName(),'quit')
                    for j in range(24):
                        self.data.put('qut')  
                        print " producing %s \n " %('quit')
                self.lock.release() #释放锁    
                break
            if text == "               ":
                continue
            order=text.split(" ")

     
#从客户端接收数据↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

       # for i in range(10):  
  
      #      print "%s: %s is producing %d to the queue!\n" %(time.ctime(), self.getName(), i)  
            self.lock.acquire() #加上锁
            if self.data.empty():
                for j in range(24):
                    self.data.put(order[j])
                    print " producing %s \n" %(order[j])
            else:
                while not self.data.empty():
                    sss=self.data.get()  
                    print " %s erasing  %s \n " %( self.getName(),sss)
                for j in range(24):
                    self.data.put(order[j])  
                    print " producing %s \n " %(order[j])
            self.lock.release() #释放锁
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
        while True:
            self.lock.acquire() #加上锁
            if  self.data.full():    
                for j in range(24):                
                    val = self.data.get() 
                    if val=='quit':
                        #postureProxy.goToPosture("StandInit", 0.2)
                        #dcm.setAlias(["UpHalfBodyStiffness","ClearAll","time-mixed",[[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]],[[-0.1, dcm.getTime(0)]]]])
                        #print "%s: %s is consuming. %s in the queue is consumed!\n" %(time.ctime(), self.getName(), val)  
                        motionProxy.rest()
                        break; 
                    order[j]=val
                self.lock.release()
            else :
                self.lock.release() #释放锁
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
                                                                  [[float(order[23])*almath.TO_RAD, dcm.getTime(750)]]
                                                                  ]])
        postureProxy.goToPosture("StandInit", 0.2)
        motionProxy.rest()
        print "%s: %s finished!" %(time.ctime(), self.getName())  
  
   
  
#Main thread  
  
def main():  
  
   

#多线程部分初始化↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    queue = Queue(24)  
      
    producer = Producer('Pro.', queue)  
  
    consumer = Consumer('Con.', queue)  
    
    producer.start()  
  
    consumer.start()  
  
    producer.join()  
  
    consumer.join()  
    
    postureProxy.goToPosture("StandInit", 0.2)
    print 'All threads terminate!'  
  
#多线程部分初始化↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑



  
if __name__ == '__main__':  
  
    main()  
