#!/usr/bin/python3
GREEN = 0
RED = 1
YELLOW = 2

ObservedThreshold = 20

import numpy as np
import situation
from datetime import datetime
from maneuver import driveBetween

WAIT_STATE = 2 #seconds before going to another state
# passed: Delete old buoys and get info
# foundGreen: Find red and call pathing
# foundRed: Find green and call pathing
# foundNothing: Check general area ahead and call the found (RED/GREEN) depending what it finds
# pathing: Path through next pair of buoys

class ObstacleCourse:
    def __init__(self,missionControl):
        self.missionControl = missionControl
        self.boat = missionControl.boat
        self.startPos = self.boat.pos
        self.state = self.passed
        self.moving = False
        self.timeSinceState = datetime.now()
        self.midpoints = [self.startPos]
    
    def mission(self):
        self.state()
    
    def passed(self):  # find buoys and dock
        if (self.moving):
            self.moving = False
            self.boat.clearBuoys(100,behind=True)
            return
        
        buoyDict = self.boat.getBuoyDict()
        
        if RED in buoyDict.keys() and GREEN in buoyDict.keys():
            self.state = self.pathing
            # self.state()
            print("Found buoys, obstacle-passed")
            return
        elif not(RED in buoyDict.keys() or GREEN in buoyDict.keys()):
            if all([buoyDict[RED][0].timesSeen,buoyDict[RED][0].timesSeen]>10): return
            action = self.foundNothing
            actionText = "Looking for any buoy, obstacle-passed"
        elif not GREEN in buoyDict.keys():
            action = self.foundRed
            actionText = "Looking for green, obstacle-passed"
        elif not RED in buoyDict.keys():
            action = self.foundGreen
            print(buoyDict)
            actionText = "Looking for red, obstacle-passed"
        
        if (datetime.now()-self.timeSinceState).total_seconds()>WAIT_STATE:
            self.state = action
            print(actionText)
            # self.state()
            return
            

    def foundGreen(self):  # search for red or give up
        buoyDict = self.boat.getBuoyDict()
        if RED in buoyDict.keys():
            self.state = self.pathing
            self.moving = False
            # self.state()
            print("Found red, obstacle-foundGreen")
            return
        if self.moving:
            if len(self.missionControl.waypoints)==1:
                #Turn 45* towards assumed red position
                turn = ((buoyDict[GREEN][0].pos-self.midpoints[-1]).tan()+np.pi/4+np.pi)%(2*np.pi)-np.pi
                self.boat.keep_heading(turn)
                if abs(self.boat.pos.heading-turn)<np.pi/18 or abs(self.boat.pos.heading-turn)>2*np.pi-np.pi/18:
                    print("Give up on red, obstacle-foundGreen")
                    self.state = self.passed
                    self.moving = False
                    # self.state()
            return
        print(buoyDict)
        print("moving to green, obstacle-foundGreen")
        halfToGreen = (self.midpoints[-1]+buoyDict[GREEN][0].pos)/2
        self.moving = True
        self.missionControl.addWaypoints([self.midpoints[-1],halfToGreen],clear=True)
    
    def foundRed(self):  # search for green or give up
        buoyDict = self.boat.getBuoyDict()
        if GREEN in buoyDict.keys():
            self.state = self.pathing
            self.moving = False
            # self.state()
            print("Found green, obstacle-foundRed")
            return
        if self.moving:
            if len(self.missionControl.waypoints)==1:
                #Turn 45* towards assumed green position
                turn = ((buoyDict[RED][0].pos-self.midpoints[-1]).tan()-np.pi/4+np.pi)%(2*np.pi)-np.pi
                self.boat.keep_heading(turn)
                if abs(self.boat.pos.heading-turn)<np.pi/18 or abs(self.boat.pos.heading-turn)>2*np.pi-np.pi/18:
                    print("Give up on green, obstacle-foundRed")
                    self.state = self.passed
                    self.moving = False
                    # self.state()
            return
        
        print("moving to red, obstacle-foundRed")
        halfToRed = (self.midpoints[-1]+buoyDict[RED][0].pos)/2
        self.moving = True
        self.missionControl.addWaypoints([self.midpoints[-1],halfToRed],clear=True)
        
    def foundNothing(self):  # Look for any buoy
        buoyDict = self.boat.getBuoyDict()
        if GREEN in buoyDict.keys():
            self.state = self.foundGreen
            self.moving = False
            # self.state()
            print("Found green, obstacle-foundNothing")
            return
        if RED in buoyDict.keys():
            self.state = self.foundRed
            self.moving = False
            # self.state()
            print("Found red, obstacle-foundNothing")
            return
        if self.moving:
            if len(self.missionControl.waypoints)==1:
                print("Give up, obstacle-foundNothing")
                self.missionControl.missionDone = True
            return
        lastMid = self.midpoints[-1]
        diff = situation.Position(np.cos(lastMid.heading),np.sin(lastMid.heading))
        self.missionControl.addWaypoints([lastMid,lastMid+diff*4],clear = True)
        self.moving = True
    
    def pathing(self):
        if self.moving:
            if len(self.missionControl.waypoints)==1:
                self.state=self.passed
                # self.state()
                print("Arrived, obstacle-pathing")
            return
        buoyDict = self.boat.getBuoyDict()
        for i in range(len(buoyDict[RED])):
            red = buoyDict[RED][i]
            if red.timesSeen > ObservedThreshold:
                if i == len(buoyDict[RED])-1:
                    return
                break
        for i in range(len(buoyDict[GREEN])):
            green = buoyDict[GREEN][i]
            if red.timesSeen > ObservedThreshold:
                if i == len(buoyDict[GREEN])-1:
                        return
                break
        self.missionControl.addWaypoints(driveBetween(red.pos,green.pos,margin=1.0),clear=True)
        self.moving = True
        rad = (self.missionControl.waypoints[1]-self.missionControl.waypoints[0]).tan()
        mid = (self.missionControl.waypoints[1]+self.missionControl.waypoints[0])/2
        mid.heading = rad
        self.midpoints.append(mid)
        print("Found path, obstacle-pathing")