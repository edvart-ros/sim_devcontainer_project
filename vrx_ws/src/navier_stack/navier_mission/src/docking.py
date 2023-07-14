#!/usr/bin/python3
GREEN = 0
RED = 1
YELLOW = 2
import numpy as np
import situation
from datetime import datetime

#BUOYMARGIN = 0.8

class Docking:
    def __init__(self,missionControl):
        self.missionControl = missionControl
        self.boat = missionControl.boat
        self.missionStage = 0
        self.missionList = [self.mission0, self.mission1, self.mission2]
        self.moving = False
        self.first = True
        self.destination = None
        self.startPoint = None
        
    def setup(self):
        self.boat.keep_heading(self.boat.pos.heading)
        
    def mission(self):
        if (self.first):
            self.setup()
            self.first = False
        self.missionList[self.missionStage]()
    
    def mission0(self):  # find buoys and dock
        #print("mission0")
        if self.startPoint == None:
            self.startPoint = self.boat.pos
            
        if (self.moving):
            if len(self.missionControl.waypoints)==1:
                self.missionStage +=1
                self.moving = False
                #self.boat.clearBuoys()
                self.startTime = datetime.now()
                print("Advancing to mission1")
            return
        
        buoyDict = self.boat.getBuoyDict()
        
        # find red first, rotate and search for green
        if not RED in buoyDict.keys():
            self.missionControl.searchMode = ["linear", 9]
            self.missionControl.searchTarget = {RED:1}
            self.missionControl.task = self.missionControl.search
            print("Could not find RED")
            return
        if not GREEN in buoyDict.keys():
            self.missionControl.searchMode = ["rotate", self.boat.pos.heading, 2*np.pi]
            self.missionControl.searchTarget = {GREEN:1}
            self.missionControl.task = self.missionControl.search
            print("Could not find GREEN")
            return
        
        if len(buoyDict[RED]) + len(buoyDict[GREEN]) > 2:
            print("Too many buoys registered, clearing oldest.")
            self.boat.clearLeastSeen({GREEN:1,RED:1})
            return
        
        print("Both buoys found")

        #  Navigation
        green = buoyDict[GREEN][0].pos
        red = buoyDict[RED][0].pos
        #boatToRed = self.boat.pos-red
        #first = boatToRed/abs(boatToRed) + green
        first = red
        #  Move close to red
        #tempPos = red - ((self.boat.pos - red)/(abs(self.boat.pos - red)))*BUOYMARGIN
        self.missionControl.addWaypoints([self.boat.pos, first],clear=True)
        #  Move close to green
        #tempPos = green - ((tempPos - green)/(abs(tempPos - green)))*BUOYMARGIN
        #self.missionControl.waypoints.append(green)
        #  Reverse to midpoint
        midpoint = (red + green)/2
        shift = (green - red) * 0.7
        offset = (rotateVector(green - red, np.pi / 2) / abs(green - red)) * 0
        self.missionControl.addWaypoints(red + shift + offset)
        #self.missionControl.addWaypoints(red + offset)
        self.destination = red + shift + offset
        #self.destination = red + offset
        
        self.moving = True
        print("found the path")

    def mission1(self):  # stay docked
        print("mission1")
        timeElapsed = (datetime.now() - self.startTime).total_seconds()
        print("Time elapsed ", timeElapsed)
        if timeElapsed > 35:
            self.missionStage +=1
            self.moving = False
            print("Advancing to mission2")
            return
    
        if self.moving:
            if len(self.missionControl.waypoints) == 1:
                #self.missionStage +=1
                self.moving = False
                #self.boat.clearBuoys()
            print("Returning to midpoint")
            return
        
        if abs(self.boat.pos - self.destination) > 1.5:
            #red = self.boat.getBuoyDict()[RED][0].pos
            #green = self.boat.getBuoyDict()[GREEN][0].pos
            #if abs(self.boat.pos - red) > abs(self.boat.pos - green):
            #    self.missionControl.waypoints = [self.boat.pos, red, green, self.destination]
            #else:
            #    self.missionControl.waypoints = [self.boat.pos, green, red, self.destination]
            self.missionControl.addWaypoints([self.boat.pos, self.destination],clear=True)
            self.moving = True
        
        #self.boat.keep_heading(self.boat.pos.heading)
    
    def mission2(self):
        if not self.moving:
            buoyDict = self.boat.getBuoyDict()
            green = buoyDict[GREEN][0].pos
            red = buoyDict[RED][0].pos
            temp = self.boat.pos + (rotateVector(green - red, -np.pi/10) / abs(green - red)) * 4.0
            self.missionControl.addWaypoints([self.boat.pos, temp, self.startPoint],clear=True)
            self.moving = True
            print("Returning to startpoint")
        
# assumes angle in radians
def rotateVector(vector, angle):
    x1 = vector.getX()
    y1 = vector.getY()
    
    x2 = np.cos(angle) * x1 - np.sin(angle) * y1
    y2 = np.sin(angle) * x1 + np.cos(angle) * y1
    return situation.Position(x2, y2)
