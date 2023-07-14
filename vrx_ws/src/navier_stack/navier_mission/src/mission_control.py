#!/usr/bin/python3
import situation
import obstacle_course
import speedgate
import docking
import rospy
import numpy as np
import math
from typing import Union,Iterable,Dict
from situation import Position

ARRIVED = 2.5  #Variabelen for om du har nådd et waypoint
VISIONMARGIN = 2  # Half the vision width of the camera, minus a margin
ARRIVED_ANGLE = np.pi/18
ROTATE_INCREMENT = np.pi/2

mission = "speedgate"

class MissionControl:
    def __init__(self,mission:str)->None:
        self.boat = situation.Boat(self)
        self.missionStage = 0
        self.missionStart = None
        self.missionDone = False
        self.searchMode = None
        self.searchTarget = None  # Green, Red, Yellow
        self.moving = False
        self.knownBuoys = None
        self.waypoints = []
        self.waypointHistory = []
        self.missionObject = None
        self.waypointChanged = False
        if mission == "dock":
            self.missionObject = docking.Docking(self)
            self.mission = self.missionObject.mission
        elif mission == "speedgate":
            self.missionObject = speedgate.SpeedGate(self)
            self.mission = self.missionObject.mission
        elif mission == "obstacle_course":
            self.missionObject = obstacle_course.ObstacleCourse(self)
            self.mission = self.missionObject.mission
        elif mission == "collision":
            self.mission = self.collisionAvoidance
        else:
            print("wrong mission name")
        self.task = self.mission
        
    def run(self)->None:
        if self.task == None:
            self.task = self.mission
        self.task()
        self.waypoint()
    
    def addWaypoints(self,wayP:Iterable[Position],clear:bool=False)->None:
        if clear:
            self.waypointHistory.extend(self.waypoints)
            self.waypoints = []
        
        if type(wayP) == list:
            self.waypoints.extend(wayP)
        else:
            self.waypoints.append(wayP)
        self.waypointChanged = True
    
    def waypoint(self)->None:
        while (len(self.waypoints) > 1) and abs(self.boat.pos-self.waypoints[1])<ARRIVED:
            #rospi.logInfo("Reached Waypoint: "+waypoints[0])
            self.waypointHistory.append(self.waypoints.pop(0))
            self.waypointChanged = True
        if len(self.waypoints) > 1 and self.waypointChanged:
            self.boat.set_waypoint([self.waypoints[0].coords[0],self.waypoints[0].coords[1]],[self.waypoints[1].coords[0],self.waypoints[1].coords[1]])
            self.waypointChanged = False

    def search(self)->None:
        if (self.searchMode[0] == "point" and len(self.searchMode) == 2):  # "point" p0
            self.pointSearch()
        elif (self.searchMode[0] == "linear" and len(self.searchMode) == 2):  # "linear" distance
            self.linearSearch(self.searchMode[1])
        elif (self.searchMode[0] == "linearP" and len(self.searchMode) == 2):  # "linearP" destination
            self.linearSearchP(self.searchMode[1])
        elif (self.searchMode[0] == "area" and len(self.searchMode) == 3):  # "area" p0 p1
            self.areaSearch(self.searchMode[1], self.searchMode[2])
        elif (self.searchMode[0] == "rotate" and len(self.searchMode) == 3):  # "rotate" lastGoal amount
            self.rotateSearch(self.searchMode[1], self.searchMode[2])
        else:
            print("INVALID SEARCH MODE")
            
            
    def pointSearch(self, maxDistance:float)->None:
        #  Sector Search.
        return
    
    def rotateSearch(self,goal:float,amount:float)->None:
        if not self.moving:
            self.knownBuoys = self.boat.getBuoyDict()
            self.moving = True
        
        diff = abs(self.boat.pos.heading-goal)
        if diff<ARRIVED_ANGLE or diff>np.pi*2-ARRIVED_ANGLE:
            if amount<0:
                self.moving = False
                print("RotateSearch Failed")
            else:
                self.searchMode[1] = (goal + np.pi + ROTATE_INCREMENT)%(2*np.pi) - np.pi
                goal = self.searchMode[1]
                self.searchMode[2] -= diff
        self.boat.keep_heading(goal)
        
        self.isSearchSuccessful()
    
    
    def linearSearch(self, distance:float)->None:
        #  Set initial route based on chosen distance
        if not self.moving:
            self.knownBuoys = self.boat.getBuoyDict()
            route = situation.Position(distance*np.cos(self.boat.pos.heading), distance*np.sin(self.boat.pos.heading))
            self.addWaypoints([self.boat.pos, self.boat.pos + route],clear=True)
            self.moving = True
        
        #  Checking if boat has ended its initial route
        if len(self.waypoints) < 2:
            self.moving = False
            
        self.isSearchSuccessful()
        
    def linearSearchP(self, destination:Position)->None:
        #  Set initial route based on chosen destination point
        if not self.moving:
            self.knownBuoys = self.boat.getBuoyDict()
            self.addWaypoints([self.boat.pos, destination],clear=True)
            self.moving = True
        
        #  Checking if boat has ended its initial route
        if len(self.waypoints) < 2:
            self.moving = False
            
        self.isSearchSuccessful()
    
    
    def areaSearch(self, p0:Position, p1:Position)->None:
        #  Set initial route based on chosen distance
        if not self.moving:
            self.knownBuoys = self.boat.getBuoyDict()
            
            #  Starting search at nearest corner of rectangle
            currentPos = self.boat.pos
            self.addWaypoints(currentPos)
            startPoint = None
            if (abs(currentPos - p1) < abs(currentPos - p0)):
                margin = -VISIONMARGIN
                startPoint = p1
            else:
                margin = VISIONMARGIN
                startPoint = p0
            self.addWaypoints(situation.Position(startPoint.x + margin, startPoint.y + margin))
                
            #  Determining whether to take horizontal or vertical long paths
            if abs((p1 - p0).y) > abs((p1 - p0).x):
                #  Vertical paths
                #  dy: the path from one vertical end to the other, including margin
                #  numdx: the number of horizontal intervals
                dy = (margin/abs(margin)) * (p1.y - p0.y - 2*abs(margin))
                numdx = math.ceil((p1.x - p0.x - 2*abs(margin))/(2*VISIONMARGIN))
                
                # TODO: if numdx < 0
                
                #  Current x and y coordinate
                cx = startPoint.x + margin
                cy = startPoint.y + margin
                
                cy += dy
                self.addWaypoints(situation.Position(cx, cy))
                dy = -dy
                
                for i in range(numdx):
                    cx += margin
                    self.addWaypoints(situation.Position(cx, cy))
                    cy += dy
                    self.addWaypoints(situation.Position(cx, cy))
                    dy = -dy
            else:
                #  Horizontal paths
                #  dx: the path from one horizontal end to the other, including margin
                #  numdy: the number of vertical intervals
                dx = (margin/abs(margin)) * (p1.x - p0.x - 2*abs(margin))
                numdy = math.ceil((p1.y - p0.y - 2*abs(margin))/(2*VISIONMARGIN))
                
                # TODO: if numdy < 0
                
                #  Current x and y coordinate
                cx = startPoint.x + margin
                cy = startPoint.y + margin
                
                cx += dx
                self.addWaypoints(situation.Position(cx, cy))
                dx = -dx
                
                for i in range(numdy):
                    cy += margin
                    self.addWaypoints(situation.Position(cx, cy))
                    cx += dx
                    self.addWaypoints(situation.Position(cx, cy))
                    dx = -dx
                
            self.moving = True
        
        #  Checking if boat has ended its initial route
        if len(self.waypoints) < 2:
            self.moving = False
            
        self.isSearchSuccessful()
     
    
    def isSearchSuccessful(self)->None:
        if self.moving:
            self.knownBuoys = self.boat.getBuoyDict()
            allTargetsAchieved = True
            for key in self.searchTarget.keys():
                if key not in self.knownBuoys.keys():
                    allTargetsAchieved = False
                    print("Missing color")
                    break
            
                if len(self.knownBuoys[key]) < self.searchTarget[key]:
                    allTargetsAchieved = False
                    print("Missing amount")
                    break
            
            if allTargetsAchieved:
                self.moving = False
                self.addWaypoints([],clear=False)
                self.task = None
                print("Search completed successfully")
        else:
            self.task = None
            print("Search unsuccessful...")

# ARRIVED = 1 # For manuell testing ha mindre strenge krav
rospy.init_node('speedgate_node')
mc = MissionControl(mission) #dock, speedgate, obstacle, collision
rospy.spin()
