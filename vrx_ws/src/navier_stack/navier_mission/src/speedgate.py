#!/usr/bin/python3
GREEN = 0
RED = 1
YELLOW = 2
import numpy as np
import situation
import datetime
#from mission_control import MissionControl
class SpeedGate:
    def __init__(self,missionControl)->None:
        self.missionControl = missionControl
        self.boat = missionControl.boat
        self.missionStage = 0
        self.missionList = [self.mission0,self.mission1,self.mission2,self.mission3,self.mission4]
        self.moving = False
        self.first = True
        self.missionStart = datetime.datetime.now()
        
    def setup(self)->None:
        self.boat.keep_heading(self.boat.pos.heading)
        
    def mission(self)->None:
        if (self.first):
            self.setup()
            self.first =False
        self.missionList[self.missionStage]()
    
    def mission0(self)->None:
        if (self.moving):
            if len(self.missionControl.waypoints)==1:
                self.missionStage +=1
                self.moving = False
                self.boat.clearBuoys(distance=100,behind=True)
                print("Arrived - speed 0")
            return
        buoyDict = self.boat.getBuoyDict()
        if (self.missionStart-datetime.datetime.now()).total_seconds()>1000:
            return
        if (not all([i in buoyDict.keys() for i in [GREEN,RED]])):
            print("Search-rotate - speed 0")
            self.missionControl.searchMode = ["rotate",self.boat.pos.heading,2*np.pi]
            self.missionControl.searchTarget = {GREEN:1,RED:1}
            self.missionControl.task = self.missionControl.search
            return
        if len(buoyDict[RED]) + len(buoyDict[GREEN])>2:
            print("Clear-Closest (inverse) - speed 0")
            self.boat.clearClosest({GREEN:1,RED:1},inverse=True)
            return
        green = buoyDict[GREEN][0]
        red = buoyDict[RED][0] 
        if any(np.array([green.timesSeen,red.timesSeen])<10):
            return
        self.midpoint = (green.pos+red.pos)/2
        temp = red.pos-self.midpoint
        tempPos = situation.Position(-temp.getY(),temp.getX())*3/abs(temp)
        if (self.boat.simulation):
            path = [self.boat.pos,self.midpoint+tempPos,self.midpoint,self.midpoint-tempPos]
        else:
            path = [self.midpoint+tempPos,self.midpoint-tempPos]
        self.missionControl.addWaypoints(path,clear=True)
        self.moving = True
        print("found the path - speed 0")

    def mission1(self)->None:
        if (self.moving):
            if len(self.missionControl.waypoints)==1:
                self.missionStage +=1
                self.moving = False
                self.boat.restoreDeleted()
                self.boat.clearBuoys(distance=100,behind=True)
                print("Arrived - speed 1")
            return
        buoyDict = self.boat.getBuoyDict()
        if not all([i in buoyDict.keys() for i in [YELLOW]]):
            #Do Search
            self.missionControl.searchMode = ["linear",20]
            self.missionControl.searchTarget = {YELLOW:1}
            self.missionControl.task = self.missionControl.search
            print("Search-linear - speed 1")
            return
        if len(buoyDict[YELLOW])>1:
            print("Clear-LastSeen - speed 1")
            self.boat.clearLeastSeen({YELLOW:1})
            return
        yellow = buoyDict[YELLOW][0]
        #if yellow.timesSeen<10:
        #    return
        temp = self.boat.pos-yellow.pos
        temp = temp/abs(temp)*7
        wayP = [self.boat.pos]
        for i in range(4):
            temp = situation.Position(-temp.getY(),temp.getX())
            wayP.append(yellow.pos+temp)
        self.missionControl.addWaypoints(wayP,clear=True)
        self.moving = True
        print("found the path - speed 1")
    
    def mission2(self)->None: #Skipped
        if self.moving:
            if len(self.missionControl.waypoints)==1:
                self.missionStage +=1
                self.moving = False
                self.boat.clearBuoys(distance=100,behind=True)
                print("Arrived - speed 2")
            return
        self.moving = True
        home = self.boat.pos-self.midpoint
        wayP = home/abs(home)*5
        self.missionControl.addWaypoints(self.boat.pos,wayP+self.midpoint,clear=True)
        print("pathing to gate - speed 2")

    def mission3(self)->None:
        if (self.moving):
            if len(self.missionControl.waypoints)==1:
                self.missionStage +=1
                self.moving = False
                print("Arrived - speed 3")
            return
        buoyDict = self.boat.getBuoyDict()
        if not all([i in buoyDict.keys() for i in [GREEN,RED]]):
            #Do Search
            print("Search-linearP - speed 3")
            self.missionControl.searchMode = ["linearP",self.midpoint]
            self.missionControl.searchTarget = {RED:1,GREEN:1}
            self.missionControl.task = self.missionControl.search
            return
        if len(buoyDict[RED]) + len(buoyDict[GREEN])>2:
            print("Clear-LeastSeen - speed 3")
            self.boat.clearLeastSeen({GREEN:1,RED:1})
            return
        green = buoyDict[GREEN][0]
        red = buoyDict[RED][0]
        if any(np.array([green.timesSeen,red.timesSeen])<10):
            return
        midpoint = (green.pos+red.pos)/2
        temp = red.pos-midpoint
        tempPos = situation.Position(-temp.getY(),temp.getX())*3/abs(temp)
        if (self.boat.simulation):
            self.missionControl.addWaypoints([self.boat.pos,self.midpoint-tempPos,self.midpoint+tempPos],clear=True)
        else:
            self.missionControl.addWaypoints([self.midpoint-tempPos,self.midpoint+tempPos],clear=True)
        self.moving = True
        print("found the path - speed 3")
    
    def mission4(self)->None:
        #Hold position
        if (self.moving): return
        self.boat.keep_heading(self.boat.pos.heading)
        self.moving = True
        self.missionControl.missionDone = True
