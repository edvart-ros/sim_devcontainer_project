from __future__ import annotations
from datetime import datetime
import numpy as np
from typing import Union,Iterable,Dict
import rospy
from navier_msgs.msg import ControlGoal, ControlCancel, BuoyLocate
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import colorsys
from visualization_msgs.msg import Marker

ODOM_FRAME = 'odom'

BUOYSPEED = 0.2 #Bestemmer mengden drift
UPDATE_RATE = 10 # So updates once every X time pos is updated
PRINT_RATE = 50

class Position:
    def __init__(self,x:float,y:float,heading:float=None)->None:
        self.coords = np.array([x,y])
        self.heading = heading
    def __add__(self,other:Position)->Position:
        if isinstance(other, Position):
            temp = self.coords+other.coords
            return Position(temp[0],temp[1])
        # elif hasattr(other,"pos"):
        #     try:
        #         return self.__add__(other.pos)
        #     except:
        #         raise Exception(str(type(other))+" doesn't have attribute 'pos' correctly")
        #     warnings.SyntaxWarning("object:"+str(other)+" should use pos attribute")
        else:
            raise Exception("Must add separate Positions together, not: "+str(self)+"+"+str(other))
    def __sub__(self,other:Position)->Position:
        if isinstance(other, Position):
            temp = self.coords-other.coords
            return Position(temp[0],temp[1])
        # elif hasattr(other,"pos"):
        #     try:
        #         return self.__sub__(other.pos)
        #     except:
        #         raise Exception(str(type(other))+" doesn't have attribute 'pos' correctly")
        else:
            raise Exception("Must subtract separate Positions from eachother, not: "+str(self)+"-"+str(other))
    def __mul__(self,other:Union[float,Position])->Union[float,Position]:
        if isinstance(other, Position):
            return np.dot(self.coords,other.coords)
        # elif hasattr(other,"pos"):
        #     try:
        #         return self.__mul__(other.pos)
        #     except:
        #         raise Exception(str(type(other))+" doesn't have attribute 'pos' correctly")
        else:
            try:
                temp = self.coords*other
                return Position(temp[0],temp[1])+Position(0,0)
            except:
                raise Exception("Must multiply separate Positions together, not: "+str(self)+"*"+str(other))
    def __truediv__(self,other)->Position:
        if isinstance(other, Position):
            raise Exception("You can't divide two objects of class: "+str(type(self)))
        # elif hasattr(other,"pos"):
        #     return self.__truediv__(other.pos)
        else:
            try:
                temp = self.coords/other
                return Position(temp[0],temp[1])+Position(0,0)
            except:
                raise Exception("Must divide separate Positions from eachother, not: "+str(self)+"/"+str(other))
    def __abs__(self)->float:
        return np.linalg.norm(self.coords)
    def __str__(self)->str:
        return str(self.coords[0])+","+str(self.coords[1])
    def getX(self)->float:
        return self.coords[0]
    def getY(self)->float:
        return self.coords[1]
    def setPos(self,x:float,y:float)->None:
        self.coords = np.array([x,y])
    def tan(self)->float:
        radians = np.arctan2(self.coords[1],self.coords[0]) # gives -pi/2 to pi/2
        return radians
class Buoy:
    def __init__(self,x:float,y:float,color:int)->None: #Color might be string also, but currently int
        self.pos = Position(x,y)
        self.color = color #"green","yellow","red"
        self.firstSeen = datetime.now()
        self.reObserved = datetime.now()
        self.timesSeen = 1
        self.deleted = False
    
    # def __init__(self,pos,color):
    #     self.coords = Position(pos)
        
    #     self.color = color #"green","yellow","red"
    #     self.firstSeen = datetime.now()
    #     self.reObserved = datetime.now()
    #     self.timesSeen = 1


    def getPos(self,reference:Position=None,heading:bool=False)->Position:
        if reference == None:
            return self.pos
        if not heading:
            return self.pos-reference
        if heading:
            try:
                heading = reference.pos.heading
            except:
                raise Exception("Reference doesn't contain heading: "+str(reference))
        rotationMatrix = np.array([[np.cos(heading),np.sin(heading)],[np.sin(heading),np.cos(heading)]])
        return (self.pos-reference)*rotationMatrix
    
    def __eq__(self,other:Buoy)->bool:
        if self.firstSeen < other.firstSeen:
            obj1,obj2 = self,other
        else:
            obj2,obj1 = self,other
        if obj1.color != obj2.color:
            #print("wrong color")
            return False #Change when confidence lvls added
        if np.linalg.norm(obj1.pos-obj2.pos)>(datetime.now() - obj1.reObserved).total_seconds()*BUOYSPEED+0.5:
            #print("wrong location",obj1.pos,obj2.pos)
            return False
        return True
    
    def __str__(self)->str:
        return str(self.color)+","+str(self.pos)+" - "+str(self.timesSeen)
        
class BuoyStorage:
    def __init__(self)->None:
        self.buoys = []
    def update(self,observations:list,update=True):
        for i in observations:
            for j in self.buoys:
                if i == j:
                    #print("found old object at:",i.pos)
                    if update:
                        j.pos = i.pos
                        j.reObserved = i.firstSeen
                        j.timesSeen += 1
                    break
            else:
                print("new object at:",i.pos)
                pass
                self.buoys.append(i)
    def clear(self,keep:bool=False)->None:
        if keep:
            for i in self.buoys:
                i.deleted = True
        else:
            self.buoys = []
    
    # def filterBuoy(self)->None:
    #     pass

class Boat:
    def __init__(self, MC)->None: 
        self.mc = MC
        self.mcUpdate = 0
        self.buoyPrint = 0
        self.buoys = BuoyStorage()
        self.pos = Position(0,0,heading=0)
        self.bottomLeft = None
        self.upperRight = None
        self.pos = Position(0,0,heading=0)
        self.los_pub      = rospy.Publisher('/control_server/goal', ControlGoal, queue_size=10)
        self.odom_sub     = rospy.Subscriber('/wamv/odometry', Odometry, self.odom_callback, queue_size=1)
        self.buoys_sub    = rospy.Subscriber('/buoy_locations', BuoyLocate, self.buoy_callback, queue_size=1)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 10)
        
        self.simulation = False
        self.simulator = None
        
    def compArea(self,pos1:Position,pos2:Position)->None:#pos1 er nede venstre av kokurranse området. pos2 er øvre høyre
        self.bottomLeft = pos1
        self.upperRight = pos2
        #NOT IMPLEMENTED

    def set_marker_color(self, colorcode: int, marker: Marker) -> Marker:
        color_map = {0: (0, 1, 0, 1), 1: (1, 0, 0, 1), 2: (1, 1, 0, 1)} # green, red, yellow
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color_map.get(colorcode, (0, 0, 0, 1))
        return marker

    def publish_single_buoy_marker(self, color_int, location, buoy_id):
        marker = Marker()
        marker.header.frame_id = ODOM_FRAME
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 1

        # Set the scale of the marker
        marker.scale.x = 0.175
        marker.scale.y = 0.175
        marker.scale.z = 0.175

        # Set the color of the marker
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1

        # Set the lifetime of the marker
        marker.lifetime = rospy.Duration(10)

        # Set the pose of the marker
        marker.pose.orientation.w = 1.0
        marker.id = buoy_id
        marker.pose.position.x = location[0]
        marker.pose.position.y = location[1]
        marker.pose.position.z = 0
        marker = self.set_marker_color(color_int, marker)
        self.marker_pub.publish(marker)
        return
        
    def updatePos(self,x:float,y:float,heading:float)->None:
        self.pos = Position(x,y,heading=heading)
        self.mcUpdate +=1
        if self.mcUpdate >= UPDATE_RATE:
            self.mc.run()
            self.mcUpdate = 0
            self.buoyPrint+=1
            if self.buoyPrint >= PRINT_RATE:
                buoyPrint = self.getBuoyDict(distance=200)
                outputText="Boat: "+str(self.pos)+" heading: "+str(self.pos.heading)+"\n"
                for i in buoyPrint.keys():
                    outputText+=str(i)+":\n"
                    for j,pos in enumerate(buoyPrint[i]):
                        self.publish_single_buoy_marker(i,(pos.pos.getX(),pos.pos.getY()),100+int(i)*10+j)
                        outputText += "  "+str(j)+": "+",".join(str(pos).split(",")[1:])+"\n"
                print(outputText)
                self.buoyPrint=0
        
        #self.transformCoor()
    
    def updateBuoy(self,buoyObjects)->None: #argument is a object with arrays as the attributes x, y and color
        buoyList = []
        for i in range(len(buoyObjects.x)):
            buoyList.append(Buoy(buoyObjects.x[i],buoyObjects.y[i],buoyObjects.color[i]))
        self.buoys.update(buoyList)
    
    def getBuoys(self,distance:float=20,includeDeleted:bool=False)->Iterable[Buoy]:
        liste = []
        for i in self.buoys.buoys:
            if abs(self.pos-i.pos)<distance and (includeDeleted or (not i.deleted)):
                liste.append(i)
        return liste
    
    def getBuoyDict(self,distance:float=20,sort:str="distance",includeDeleted:bool=False)->Dict[Union[int,str],Iterable[Buoy]]:
        liste = self.getBuoys(distance=distance,includeDeleted=includeDeleted)
        buoyDict = {}
        if sort == "distance":
            liste = sorted(liste,key=lambda x: abs(self.pos-x.pos))
        for i in liste:
            if i.color in buoyDict.keys():
                buoyDict[i.color].append(i)
            else:
                buoyDict[i.color] = [i]
        return buoyDict
    
    def clearBuoys(self,distance:float=0,timeSince:float=100,behind:bool=False,keep:bool=True)->None:
        buoys = self.buoys.buoys
        distance = np.array([False if abs(self.pos -i.pos)<distance else True for i in buoys])
        if behind:
            behind = np.array([True if self.behind(i) else False for i in buoys])
        else:
            behind = np.zeros(len(buoys))>0
        timeDelete = np.array([True if (datetime.now()-i.reObserved).total_seconds()>timeSince else False for i in buoys])
        
        remove = timeDelete + behind + distance
        # print(behind + distance,time,behind,distance)
        for i in range(len(buoys)-1,-1,-1):
            if remove[i]:
                if keep:
                    buoys[i].deleted=True
                else:
                    buoys.pop(i)
                    
    # Dict[Union[str,int],int] is an argument where Union refers to color and int to amount of that color
    def clearOldest(self, dictAmount:Dict[Union[str,int],int],keep:bool=True)->None:
        dictCurrent = self.getBuoyDict(includeDeleted=False)
        for i in list(dictCurrent.keys()):
            if (i not in dictAmount.keys()) or len(dictCurrent[i])<=dictAmount[i]:
                continue
            times = [j.reObserved for j in dictCurrent[i]]
            sortedList = sorted(zip(times,dictCurrent[i],range(len(times))),key= lambda x : x[0],reverse=True)
            for k in sortedList[dictAmount[i]:]:
                if keep:
                    self.buoys.buoys[k[2]].deleted=True
                    print("Buoy tagged as deleted "+str(self.buoys.buoys[k[2]]))
                else:
                    print("Deleted buoy: "+str(self.buoys.buoys.pop(k[2])))
    
    # Dict[Union[str,int],int] is an argument where Union refers to color and int to amount of that color
    def clearLeastSeen(self, dictAmount:Dict[Union[str,int],int],keep:bool=True)->None:
        dictCurrent = self.getBuoyDict()
        for i in list(dictCurrent.keys()):
            if (i not in dictAmount.keys()) or len(dictCurrent[i])<=dictAmount[i]:
                continue
            timesSeen = [j.timesSeen for j in dictCurrent[i]]
            sortedList = sorted(zip(timesSeen,dictCurrent[i],range(len(timesSeen))),key= lambda x : x[0],reverse=True)
            summen = 0
            for j in range(dictAmount[i]):
                summen+=sortedList[j][0]
            if summen<100:
                print("not Observed enough: "+str(summen))
                continue
            for k in sortedList[dictAmount[i]:]:
                if keep:
                    self.buoys.buoys[k[2]].deleted=True
                    print("Buoy tagged as deleted "+str(self.buoys.buoys[k[2]]))
                else:
                    print("Deleted buoy: "+str(self.buoys.buoys.pop(k[2])))
    
    # Dict[Union[str,int],int] is an argument where Union refers to color and int to amount of that color
    def clearClosest(self, dictAmount:Dict[Union[str,int],int],keep:bool=True,inverse:bool=False)->None:
        dictCurrent = self.getBuoyDict()
        for i in list(dictCurrent.keys()):
            if (i not in dictAmount.keys()) or len(dictCurrent[i])<=dictAmount[i]:
                continue
            distance = [abs(j.getPos(reference=self.pos)) for j in dictCurrent[i]]
            sortedList = sorted(zip(distance,dictCurrent[i],range(len(distance))),key= lambda x : x[0],reverse=not inverse)
            summen = 0
            for j in range(dictAmount[i]):
                summen+=sortedList[j][0]
            if summen<10:
                print("not Observed enough: "+str(summen))
                continue
            for k in sortedList[dictAmount[i]:]:
                if keep:
                    self.buoys.buoys[k[2]].deleted=True
                    print("Buoy tagged as deleted "+str(self.buoys.buoys[k[2]]))
                else:
                    print("Deleted buoy: "+str(self.buoys.buoys.pop(k[2])))
    
    def restoreDeleted(self)->None:
        for i in self.buoys.buoys:
            i.deleted = False

    def behind(self,buoy:Buoy)->bool:
        vectorA = buoy.pos-self.pos
        vectorB = Position(np.cos(self.pos.heading),np.sin(self.pos.heading))
        result = vectorA*vectorB
        # print(result)
        return True if result<0 else False
    
    def buoy_callback(self, msg)->None: #msg is a object with arrays as the attributes x, y and color
        self.updateBuoy(msg)

    #ROS-Code
    def set_waypoint(self, point_0, point_1):
        goal= ControlGoal()
        goal.type =  'LOS'
        goal.x_list = point_0[0], point_1[0]
        goal.y_list = point_0[1], point_1[1]
        self.los_pub.publish(goal)

    #ROS-Code
    def odom_callback(self, msg):
        #convert orientation to euler
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.updatePos(position.x,position.y,yaw)
    #ROS-Code
    def keep_heading(self, angle):
        print("called keep heading which is not implemented yet")
        return
        # mode_msg = AutoMode()
        # maneuver_msg = Maneuver()
        # mode_msg.auto_mode = 'MANEUVER'
        # maneuver_msg.maneuver = 'keep_heading'
        # maneuver_msg.parameter = angle
        # self.mode_pub.publish(mode_msg)
        # self.maneuver_pub.publish(maneuver_msg)
    ##Sim-Code
    # def set_waypoint(self, point_0:Position, point_1:Position)->None:
    #     if not self.simulation:
    #         print("Remember to use the ROS set_waypoint when not simulation is used!")
    #         raise Exception("Comment this code and uncomment the one above")
    #     self.simulator.destination = Position(point_1[0],point_1[1])
    ##Sim-Code
    # def keep_heading(self, angle:float)->None:
    #     if not self.simulation:
    #         print("Remember to use the ROS set_waypoint when not simulation is used!")
    #         raise Exception("Comment this code and uncomment the one above")
    #     heading = self.pos*1 # Don't remove will not work. (Just to get new Position object)
    #     heading.heading = angle
    #     self.simulator.destination = heading

    
# start = datetime.now()
# boat = Boat()
# buoyDict1 = {"x":[1,1],"y":[2,2],"color":["green","red"]}
# boat.updateBuoy(TestRos(buoyDict1))
# time.sleep(5)
# end = datetime.now()
# buoyDict2 = {"x":[-3],"y":[10],"color":["green"]}
# boat.updateBuoy(TestRos(buoyDict2))
# print("diff:",end.second-start.second)
# boat.updatePos(4,-2,90)
# boat.clearBuoys(distance=10,behind=True)
# print(boat.getBuoys())

# pos1 = Position(4,3)
# pos2 = Position(4,8)
# print(pos1+pos2)
# print(pos1-pos2)
# print(pos1*pos2)
# print(pos1*3)
# print(pos1/2) 