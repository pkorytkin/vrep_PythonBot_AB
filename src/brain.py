#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
import geometry_msgs.msg
import math

from typing import List
class Vector3(geometry_msgs.msg.Vector3):
    def __init__(self,x:float=0,y:float=0,z:float=0):
        self.x=x
        self.y=y
        self.z=z
    def __sub__(self,other):
        if(type(other)==Vector3):
            return Vector3(self.x-other.x,self.y-other.y,self.z-other.z)
    def __add__(self,other):
        if(type(other)==Vector3):
            return Vector3(self.x+other.x,self.y+other.y,self.z+other.z)
    def __mul__(self,other):
        if(type(other)==Vector3):
            return self.x*other.x+self.y*other.y+self.z*other.z
        elif(type(other)==float):
            return Vector3(self.x*other,self.y*other,self.z*other)
    def __str__(self) -> str:
        return "x:"+str(self.x)+" y:"+str(self.y)+" z:"+str(self.z)
    def Module(self):
        return float(math.sqrt((self.x)**2+(self.y)**2+(self.z)**2))
    
    def normalize(self):
        dist=self.Module()
        if(dist!=0):
            return Vector3(self.x/dist,self.y/dist,self.z/dist)
        else:
            return self

StartPoseSaved:bool=False
#Позиция куда нужно попасть
TargetPose:Vector3=Vector3(0,0,0)

#Глобальная переменная для нынешней позиции
CurrentPose:geometry_msgs.msg.Pose2D=geometry_msgs.msg.Pose2D()
#Сохранённая стартовая позиция
StartPose:geometry_msgs.msg.Pose2D=geometry_msgs.msg.Pose2D()
#Сообщение поворота отправляемое в Topic
twist:geometry_msgs.msg.Twist=geometry_msgs.msg.Twist()
#Целевая точка маршрута
#CurrentPointID:int=0
#Сохранённые координаты точек маршрута
#PointList:List[Vector3]=[]
#PointList=[]
#Rate
r:rospy.Rate

LinearSpeed:float=1.

poseSub:rospy.Subscriber
cmd_velPub:rospy.Publisher

ForwardLaser=0
BackwardLaser=0
LeftLaser=0
RightLaser=0
#2.5 2.5 - -2.5 -2.5
#Карта собранная лидаром
MapSize=50+1
#Map2D=[[0]*MapSize]*MapSize
Map2D=[ [0]*MapSize for _ in range(MapSize) ]

rightBottomCorner=Vector3(-2.5,-2.5,0)

def DistanceVector3(fromPosition:Vector3,toPosition:Vector3=Vector3()):
    return (fromPosition-toPosition).Module()
def DotProduct(fromVector:Vector3,toVector:Vector3):
    return fromVector*toVector
def CrossProduct(fromVector:Vector3,toVector:Vector3):
    return fromVector.x*toVector.y-fromVector.y*toVector.x
def Angle3D(fromVector:Vector3,toVector:Vector3):
    #https://www.wikihow.com/Find-the-Angle-Between-Two-Vectors
    
    fromVector=fromVector.normalize()
    toVector=toVector.normalize()
    fromVectorAngle=math.atan2(fromVector.y,fromVector.x)
    toVectorAngle=math.atan2(toVector.y,toVector.x)
    
    #print("from="+str(fromVectorAngle)+" to="+str(toVectorAngle))
    angle= abs(toVectorAngle-fromVectorAngle)
    if(toVectorAngle<fromVectorAngle):
        return -angle
    return angle
    
    #angle=math.atan2(CrossProduct(fromVector,toVector),DotProduct(fromVector,toVector))
    #angle=math.acos(DotProduct(fromVector,toVector))
    #cross=CrossProduct(fromVector,toVector)
    #sign=1
    #if(DotProduct())
    #if (fromVector.x*toVector.y-fromVector.y*toVector.x<0):
    #    sign=-1
    #angle=sign*math.acos(fromVector*toVector/(DistanceVector3(fromVector)*DistanceVector3(toVector))).real
    """while angle>math.pi*2:
        angle-=math.pi*2
    while angle<-math.pi*2:
        angle+=math.pi*2
    
    if(angle>math.pi):
        angle-=math.pi*2
    if(angle<-math.pi):
        angle+=math.pi*2
    return angle"""
#Преобразование локальной позиции черепахи в мировую
def LocalPositionToWorld(Vector:Vector3):
    return CurrentPosition()+Vector
#Мировая позиция в локальную
def WorldToLocalPosition(Vector:Vector3):
    return Vector-CurrentPosition()
#Нынешняя позция
def CurrentPosition():
    #z=CurrentPose.z
    position=Vector3(x=CurrentPose.x,y=CurrentPose.y)
    return position
#Нынешняя ротация в углах эйлера
def CurrentRotation():
    rotation=Vector3(x=0,y=0,z=CurrentPose.theta)
    return rotation
#Нынешней глобальный вектор вперёд
def CurrentGlobalForward():
    return Vector3(x=math.cos(CurrentPose.theta).real,y=math.sin(CurrentPose.theta).real,z=0)
#Локальный вектор вперёд x=1
def CurrentLocalForward():
    return Vector3(1,0,0)

def subscriber_pose(pose:geometry_msgs.msg.Pose2D):
    global CurrentPose
    #rospy.loginfo(pose)
    #Сохранение стартовой позиции
    global StartPoseSaved
    global StartPose
    if(not StartPoseSaved):
        StartPose=pose
        StartPoseSaved=True
    #Сохранение нынешней позиции
    CurrentPose=pose
    #print("Saved CurrentPose="+str(pose))
    pass
def subscriber_target_pose(targetPose:geometry_msgs.msg.Vector3):
    global TargetPose
    TargetPose=Vector3(targetPose.x,targetPose.y,targetPose.z)
    pass
def subscriber_top_laser(lidar:geometry_msgs.msg.Vector3):
    global TargetPose
    #TargetPose=Vector3(targetPose.x,targetPose.y,targetPose.z)
    tetha=lidar.x+CurrentPose.theta
    distance=lidar.z
    
    positionVector=Vector3(math.cos(tetha),math.sin(tetha),0).normalize()*distance
    positionVector+=CurrentPosition()
    position=positionVector-rightBottomCorner
    #print(CurrentPose.theta,tetha,lidar.z)
    
    #if(tetha>math.pi/2 and tetha<math.pi/2):
        
    #print(position)#,CurrentPosition())
    #position=position*MapSize
    #print(position)
    X_Point=int(position.x/5*(MapSize-1))
    Y_Point=int(position.y/5*(MapSize-1))
    if(X_Point<0):
        X_Point=0
    if(Y_Point<0):
        Y_Point=0
    print(X_Point,Y_Point)
    Map2D[Y_Point][X_Point]=1
    #print(Map2D)
    for y in range(MapSize):
        print(Map2D[MapSize-1-y])
    
    pass
def PrepareGlobals():
    global StartPoseSaved
    global CurrentPose
    global StartPose
    global r
    StartPose=geometry_msgs.msg.Pose2D()
    #Нужно сохранить стартовую позицию
    StartPoseSaved=False
    
    CurrentPose=geometry_msgs.msg.Pose2D()
    #print("Reseted CurrentPose="+str(CurrentPose))
    
    
    rospy.init_node("brain")
    r=rospy.Rate(60)#60hz

def CurrentTargetPoint():
    return TargetPose

def PrepareWorkers():
    print("Angle="+str(math.degrees(Angle3D(Vector3(1,0,0),Vector3(-1,0,0)))))
    PrepareGlobals()
    global poseSub
    global targetPoseSub
    global topLaserSub
    global cmd_velPub
    poseSub=rospy.Subscriber("pose",geometry_msgs.msg.Pose2D,tcp_nodelay=True,queue_size=1,callback=subscriber_pose)
    
    targetPoseSub=rospy.Subscriber("TargetPosition",geometry_msgs.msg.Vector3,tcp_nodelay=True,queue_size=1,callback=subscriber_target_pose)
 
    topLaserSub=rospy.Subscriber("TopLaser",geometry_msgs.msg.Vector3,tcp_nodelay=True,queue_size=1,callback=subscriber_top_laser)
    
    cmd_velPub=rospy.Publisher("cmd_vel",geometry_msgs.msg.Twist,tcp_nodelay=True,queue_size=1)
    #Спим несколько раз чтоб дождаться позиции в subscriber
    r.sleep()
    r.sleep()
    r.sleep()
    r.sleep()

def sign(value):
    if(value<=0):
        return -1
    return 1
def Worker():
    global CurrentPointID
    global PointsCount
    while not(rospy.is_shutdown()):
        
        #print("Tetha="+str(CurrentPose.theta))
        #print("CurrentPointID="+str(CurrentPointID))
        #print("CurrentTargetPoint="+str(CurrentTargetPoint()))
        #print("CurrentPosition="+str(CurrentPosition()))
        VectorToPoint=CurrentTargetPoint()-CurrentPosition()
        #print("VectorToPoint="+str(VectorToPoint))
        currentGlobalForward=CurrentGlobalForward()
        #print("CurrentGlobalForward="+str(currentGlobalForward))
        AngleFromForwardToPoint=Angle3D(currentGlobalForward,VectorToPoint)
        #print("AngleFromForwardToPoint="+str(AngleFromForwardToPoint))
        DistanceToPoint=DistanceVector3(CurrentPosition(),CurrentTargetPoint())
        twist.angular=Vector3(0,0,0)
        twist.linear=Vector3()
        #print("DistanceToPoint="+str(DistanceToPoint))
        if(DistanceToPoint<-0.05):
            twist.linear=-1*CurrentLocalForward()
        elif(abs(AngleFromForwardToPoint)>0.05):
            if(abs(AngleFromForwardToPoint)>0.1):
                twist.angular.z=max(abs(AngleFromForwardToPoint),0.1)*sign(AngleFromForwardToPoint)
            else:
                twist.angular.z=max(abs(AngleFromForwardToPoint),0.01)*sign(AngleFromForwardToPoint)
        else:
            if(DistanceToPoint>0.1):
                twist.linear=CurrentLocalForward()*max(DistanceToPoint,0.1)*LinearSpeed#(CurrentLocalForward()*max(DistanceToPoint,0.01)).convertToMSG()
            elif(DistanceToPoint<-0.1):
                twist.linear=-1*CurrentLocalForward()*max(DistanceToPoint,0.1)*LinearSpeed#(CurrentLocalForward()*(-max(DistanceToPoint,0.01))).convertToMSG()
            #else:
                #CurrentPointID+=1
                #if(CurrentPointID==PointsCount):
                 #   CurrentPointID=0
        
        #twist.linear.x=1
        #twist.angular.z=-1
        
        
        #cmd_velPub.publish(twist)
        
        #print(twist)
        r.sleep()



if __name__ == '__main__':
    try:
        
        PrepareWorkers()
        #Заготавливаем точки маршрута
        
        Worker()
    except rospy.ROSInterruptException:
        pass
