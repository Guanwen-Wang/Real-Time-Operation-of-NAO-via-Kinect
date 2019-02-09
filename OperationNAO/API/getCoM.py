# -*- encoding: UTF-8 -*-

"""Example: Use getCOM Method"""

import qi
import argparse
import sys
import motion


def main(session):
    """
    This example uses the getCOM method.
    """
    # Get the service ALMotion.

    motion_service  = session.service("ALMotion")

    # Example showing how to get the COM position of "HeadYaw".
    name = "Body"
    frame = motion.FRAME_TORSO
    
    useSensors = True
    #pos = motion_service.getCOM(name, frame, useSensors)
    #print "HeadYaw COM Position: x = ", pos[0], " y:", pos[1], " z:", pos[2]
    for i in range(1,10):
        pos = motion_service.getCOM(name, frame, useSensors)
        f = file("H:\\b.txt", "a+")
        a=[str(pos[0]),",",str(pos[1]),",",str(pos[2]),"\n"]
        f.writelines(a)
        f.close()    


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.191.2",
                        help="Robot IP address. On robot or Local Naoqi: use '192.168.191.2'.")
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
    main(session)