# SDP Team 12 LiDAR Navigation Code
# Date created: 9/20/23
# Last modified date: 9/30/23
# Summary: Navigate around obstacles in front of robot using LiDAR in autonomous mode
# BetterAvoidWithGoal: performs avoidance with previous states but with another added constraint of minimizing distance to goal

# How to run the file from command line:
# rosrun PKGNAME ROS_LiDAR_Navigation_Code.py --start='(x1, y1)' --goal='(x2, y2)'

# Import system packages
import math
import time
import argparse
import numpy as np
import math

# Import ROS specific packages
import rclpy
from sensor_msgs.msg import LaserScan

class BetterAvoidWithGoal:
    def __init__(self, startCoords=(0, 0), goalCoords=(0, 0)):
        self.start_time = time.time()
        print("Starting Better Navigation Algorithm With Goal...")

        # TODO: Update actionable distances for LiDAR detection 
        self.distance = ...
        self.ignore = ...

        # TODO: Set up to obtain wheel position = 2*pi*R_wheel * (dTicks / ticksPerRev)
        self.rpos = ...
        self.lpos = ...

        # TODO: Find out the width of the robot to get wheel separation
        self.wheel_separation = ...

        self.currentState = None
        self.previousState = None
        self.currentStateChanged = True

        self.startCoords = startCoords
        self.goalCoords = goalCoords
        self.phi = 0

        self.movements = {
            'left': 0.15 * 2,
            'front': 0.15,
            'right': -0.15 * 2,
            'back': -0.15
        }

        self.sub = rclpy.Subscriber('/scan', LaserScan, self.computeBetterRegions)
        rclpy.spin()

    def computeBetterRegions(self, msg):
        if self.startCoords == self.goalCoords:
            rclpy.signal_shutdown("Goal reached! Ending autonomous mode...")

        # min_angle = -pi, max_angle = pi, and they both point in front of stretch, where x axis is
        # calculate a difference from min_angle using unit circle, and then use that to get range index
        fleft = int((math.pi/6) / msg.angle_increment)
        left = int((math.pi/3) / msg.angle_increment)
        fright = int((11*math.pi/6) / msg.angle_increment)
        right = int((5*math.pi/3) / msg.angle_increment)
        b1 = int((5*math.pi/6) / msg.angle_increment)
        b2 = int((7*math.pi/6) / msg.angle_increment)

        fleftClosest = min(i for i in msg.ranges[fleft:left] if i > self.ignore)
        frontClosest = min(i for i in msg.ranges[0:fleft] + msg.ranges[fright:] if i > self.ignore)
        frightClosest = min(i for i in msg.ranges[right:fright] if i > self.ignore)
        backClosest = min(i for i in msg.ranges[b1:b2] if i > self.ignore)

        # Here we want to backup and move away from obstacles when they get too close
        regions = {
        'backup': fleftClosest <= self.distance or frontClosest <= self.distance or frightClosest <= self.distance,
        'gofront': backClosest <= self.distance + 0.15,
        'fleft': fleftClosest <= 2*self.distance and fleftClosest > self.distance,
        'front':  frontClosest <= 2*self.distance and frontClosest > self.distance,
        'fright':  frightClosest <= 2*self.distance and frightClosest > self.distance
        }

        self.takeBetterAction(regions)
        if self.currentStateChanged: # only print the state when it changes
            print("Current: {}, Previous: {}".format(self.currentState, self.previousState))
            self.previousState = self.currentState

    def takeBetterAction(self, regions):
        # Keep track of a previous state so we do not deadlock between 2 actions forever
        # Stretch can detect an obstacle in fright, turn left to avoid, and detect in front, turn right to avoid, and continue doing this forever
        # Based on the previous state, actions will change
        # Left is positive, right is negative

        xm = 0
        xr = 0

        dX, dY, dPhi = self.getPos()

        possibleActions = {
            'left': (0, 0),
            'front': (-dX, -dY),
            'right': (0, 0),
            'back': (dX, dY)
        }

        d = {
            'left': 0,
            'right': 0,
            'front': 0,
            'back': 0
        }
        for action in possibleActions:
            d[action] = self.calculateManhattanDistance(self.startCoords + action)

        obs, delay = self.getLiDARDetectedObstacles(regions)

        remainingActions = set(possibleActions.keys()) - set(obs)
        
        actionToTake = list(d.keys())[list(d.values()).index(min(d[possibleActions[a]] for a in remainingActions))]
        self.startCoords += possibleActions[actionToTake]
        self.phi += dPhi

        if actionToTake == 'left' or actionToTake == 'right':
            xr = self.movements[actionToTake]
        else:
            xm = self.movements[actionToTake]

        # if xm != 0:
        #     self.move_base(xm)
        
        # if xr != 0:
        #     self.rotate_base(xr)

        time.sleep(delay)

    def getPos(self):
        # TODO: get new wheel position the same way
        dRPos =  ... - self.rpos
        dLPos = ... - self.lpos
        dS = (dRPos + dLPos) / 2

        dPhi = (dRPos - dLPos) / self.wheel_separation
        dX = dS*math.cos(self.phi + (dPhi / 2))
        dY = dS*math.sin(self.phi + (dPhi / 2))

        # TODO: Update the old wheel positions
        self.rpos = ...
        self.lpos = ...
        return dX, dY, dPhi

    def getLiDARDetectedObstacles(self, regions):
        o = []
        tempState = self.currentState
        oldDelay = 0.1
        newDelay = 0.2
        delay = oldDelay # updates delay based on action calculated to allow sufficient time 

        # Figure out where the obstacles are at according to LiDAR detected regions
        # Single region detected by a normal size object
        if regions['front'] and not regions['fright'] and not regions['fleft']: # Obstacle only in the front
            tempState = 'front'
            o.append('front')

            # Need to consider both sides 
            if self.previousState == 'fright' or self.previousState == 'front and fright':
                o.append('right')
                delay = newDelay
            elif self.previousState == 'fleft' or self.previousState == 'front and fleft':
                o.append('left')
                delay = newDelay
        elif regions['fright'] and not regions['front'] and not regions['fleft']: # Obstacle only on fright
            tempState = 'fright'
            o.append('right')

            # Only need to consider if something was in front since you turn left anyways
            if self.previousState == 'front' or self.previousState == 'front and fleft':
                o.remove('right')
                o.append('left')
                delay = newDelay
        elif regions['fleft'] and not regions['front'] and not regions['fright']: # Obstacle only on fleft
            tempState = 'fleft'
            o.append('left')

            # Only need to consider if something was in front since you turn right anyways
            if self.previousState == 'front' or self.previousState == 'front and fright':
                o.remove('left')
                o.append('right')
                delay = newDelay

        # Multiple regions detected because of large object
        elif regions['front'] and regions['fright'] and not regions['fleft']: # Obstacle only on front and fright
            tempState = 'front and fright'
            o.append('front')
            o.append('right')

            if self.previousState == 'fleft' or self.previousState == 'front and fleft' or self.previousState == 'front':
                o.remove('right')
                o.append('left')
                delay = newDelay
        elif regions['front'] and regions['fleft'] and not regions['fright']: # Obstacle only on front and fleft
            tempState = 'front and fleft'
            o.append('front')
            o.append('left')

            if self.previousState == 'fright' or self.previousState == 'front and fright' or self.previousState == 'front':
                o.remove('left')
                o.append('right')
                delay = newDelay
        elif regions['fleft'] and regions['fright'] and not regions['front']: # Obstacle only on fright and fleft
            tempState = 'fleft and fright'
            o.append('left')
            o.append('right')
        elif regions['front'] and regions['fleft'] and regions['fright']: # Obstacle all over the front sections
            tempState = 'all'
            o.append('front')
            o.append('left')
            o.append('right')
        elif regions['gofront']:
            tempState = 'gofront'
            o.append('back')

            if self.previousState == 'backup':
                o.append('front')
                delay = newDelay
        else:
            tempState = 'nothing'

        # Too close, backup. Evaluate as a separate case
        if regions['backup']: 
            tempState = 'backup'
            o.append('front')

        # If state did not change, we do not update previous state. Or else both can become the same and we lose our previous information
        if tempState == self.currentState:
            self.currentStateChanged = False
        else:
            self.currentStateChanged = True

        self.currentState = tempState

        return o, delay

    def calculateManhattanDistance(self, currentCoord):
        return abs(self.goalCoords[0] - currentCoord[0]) + abs(self.goalCoords[1] - currentCoord[1])
    
    def calculateEuclideanDistance(self, currentCoord):
        return math.sqrt((self.goalCoords[0] - currentCoord[0])**2 + (self.goalCoords[1] - currentCoord[1])**2)

    # TODO: Implement movement commands for robot
    def move_base(self, x):
        # Use distance
        # Move motors

    def rotate_base(self, theta):
        # Use distance
        # Move motors

if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument('--start', default=(0, 0), type=str, help='what are the start coordinates')
    args.add_argument('--goal', default=(0, 0), type=str, help='what are the goal coordinates')
    args, unknown = args.parse_known_args()

    start = eval(args.start)
    goal = eval(args.goal)

    rclpy.init_node('avoid_obstacles_with_goal')
    BetterAvoidWithGoal(start, goal)