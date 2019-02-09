
import time
import argparse
from naoqi import ALProxy
import socket
from time import ctime
import naoqi
import almath

def main(robotIP, PORT = 9559):
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    alautonomousProxy = ALProxy("ALAutonomousLife", robotIP, PORT)
    if alautonomousProxy.getState()=="solitary" or alautonomousProxy.getState() == "interactive":
        alautonomousProxy.setState("disabled")
    # Wake up robot
    motionProxy.wakeUp()
    # Send robot to Stand Init
    postureProxy.goToPosture("StandInit", 0.5)
    # Example showing body zero position
    # Instead of listing each joint, you can use a the name "Body"
    names  = "Body"
    # We still need to specify the correct number of target angles, so
    # we need to find the number of joints that this Nao has.
    # Here we are using the getBodyNames method, which tells us all
    # the names of the joints in the alias "Body".
    # We could have used this list for the "names" parameter.
    # Using 10% of maximum joint speed
    maxSpeedFraction  = 0.1
    while True:
        data,addr = server.recvfrom(1024)
        text = str(data)
        if text == "quit":
            motionProxy.rest()
            break
        if text == "test" :
            continue
        order=data.split(" ")
        if order[0]==None:
            continue
        motionProxy.angleInterpolationWithSpeed(names, [[float(order[1])*almath.TO_RAD],[float(order[0])*almath.TO_RAD],\
                                                        [float(order[3])*almath.TO_RAD],[float(order[2])*almath.TO_RAD],\
                                                        [float(order[5])*almath.TO_RAD],[float(order[4])*almath.TO_RAD],\
                                                        [float(order[6])*almath.TO_RAD],[0],\
                                                        [0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],\
                                                        [float(order[8])*almath.TO_RAD],\
                                                        [float(order[7])*almath.TO_RAD],\
                                                        [float(order[10])*almath.TO_RAD],\
                                                        [float(order[9])*almath.TO_RAD],\
                                                        [float(order[11])*almath.TO_RAD],[0]], maxSpeedFraction)
        server.sendto('done' , ("127.0.0.1",8888)) 
        print addr
        print order
        print order[0]
    print("udp server finish")
    server.close()

    # Go to rest position


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.191.3",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")
    server = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    server.bind(("127.0.0.1",8888))
    print("udp server start")
    args = parser.parse_args()
    main(args.ip, args.port)
