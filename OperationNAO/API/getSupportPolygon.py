# -*- encoding: UTF-8 -*-

"""Example: Use getSupportPolygon Method"""

import qi
import argparse
import sys
import time
import pygame


# display settings
displayScale = 1000 # pixels/m
screenWidth = 700   # pixels
screenHeight = 700  # pixels

screenColor = pygame.Color("white")
polygonCommandColor = pygame.Color("green")
polygonSensorColor = pygame.Color("red")
comColor = pygame.Color("blue")

def almotionToPygame(point):
    """
    Links ALMotion to Pygame.
    """
    return [int(-point[1] * displayScale + 0.5 * screenWidth),
            int(-point[0] * displayScale + 0.5 * screenHeight)]

def drawPolygon(screen, data, color) :
    """
    Draw a Polygon.
    """
    if (data != []):
        pygame.draw.lines(screen, color, True, map(almotionToPygame, data), 1)

def drawPoint(screen, point, color) :
    """
    Draw a Point.
    """
    pygame.draw.circle(screen, color, almotionToPygame(point), 3)

def main(session, frame) :
    """
    This example uses the getSupportPolygon method.
    """
    # Get the service ALMotion.

    motion_service  = session.service("ALMotion")

    pygame.init()
    screen = pygame.display.set_mode((screenWidth, screenHeight))
    pygame.display.set_caption("Robot Support Polygon")

    running = True
    while (running):
        # Check if user has clicked 'close'
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(screenColor)

        supportPolygonCommand = motion_service.getSupportPolygon(frame, False)
        drawPolygon(screen, supportPolygonCommand, polygonCommandColor)

        supportPolygonSensor = motion_service.getSupportPolygon(frame, True)
        drawPolygon(screen, supportPolygonSensor, polygonSensorColor)

        com = motion_service.getCOM("Body", frame, False)
        drawPoint(screen, com, comColor)

        pygame.display.flip()
        time.sleep(0.1)
    pygame.quit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.191.2",
                        help="Robot IP address. On robot or Local Naoqi: use '192.168.191.2'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    parser.add_argument("--frame", type=str, choices =['world', 'robot'], default = 'world')

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    if (args.frame == 'world'):
        frame = 1
    else:
        frame = 2
    main(session, frame)