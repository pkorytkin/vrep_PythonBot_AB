#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Система отсчёта - система координат + измерение времени.
#СК - система декартова правая. Математическая абстракция точка не имеющая размера и других физических характеристик.
#Начало СК - пересечение 3х плоскостей и без координат. Пересечение 2х - прямая. а если 3х то точка.
#Где центр отсчёта
#Маленьких детей просят не показывать пальцем, а нужно описать логически. Где не тут, а описать.
#Киниматика у робота 
#Кинематика в механике, геометрический язык в целом. Наука или раздел механики заниается изучением траекторий движения тел.
#Если говорим о кинематике устройства, то по каким траекториям оно может двигаться.
#Кинематика 4х колёсного, 3х осных, гесенечных у всех у них возможности геометрических перемещений разные .
#Минимальный радиус поворота робота
#Что такое рояльное колеса в след раз обсудим особенности и специфику такого вида колеса для задач робототехники.

#Вторая характеристика - динамическая.
#Динамика - раздел механики.
#Динамика изучает причины движения по той или иной траектории или если нужно по какой, то то какие силы должны быть приложены. 2й закон ньютона. F=ma 

#Движение в СК из точки 0,5 в точку 0, -5 по прямой
#Движение происходит в модельной среде VREP
#разбитой на дискретной
#Мною выбрана среда VREP в этой среде поле делится на условные квадраты
#Я реализую движение робата из точки 0,5 в точку 0 -5
########################################################################Изображение данного видиния в виде данных лидара, почему не ровно
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

LinearSpeed:float=5.

poseSub:rospy.Subscriber
cmd_velPub:rospy.Publisher

ForwardLaser=0
BackwardLaser=0
LeftLaser=0
RightLaser=0
#2.5 2.5 - -2.5 -2.5
#Карта собранная лидаром
#MapSize=50+1
MapSize=200
#Map2D=[[0]*MapSize]*MapSize
Map2D=[ [0]*MapSize for _ in range(MapSize) ]

rightBottomCorner=Vector3(-2.5,-2.5,0)

TargetPointFromPathfinder=Vector3(0,0,0)
HasPointFromPathfinder=False

SelectedTargetDistanceOnPath=0.2

def DistanceVector3(fromPosition:Vector3,toPosition:Vector3=Vector3()):
    return (fromPosition-toPosition).Module()
def DotProduct(fromVector:Vector3,toVector:Vector3):
    return fromVector*toVector
def CrossProduct(fromVector:Vector3,toVector:Vector3):
    return fromVector.x*toVector.y-fromVector.y*toVector.x
def Angle2D(fromVector:Vector3,toVector:Vector3):
    #https://www.wikihow.com/Find-the-Angle-Between-Two-Vectors
    
    #fromVector=fromVector.normalize()
    #toVector=toVector.normalize()
    '''fromVectorAngle=math.atan2(fromVector.y,fromVector.x)
    toVectorAngle=math.atan2(toVector.y,toVector.x)

    
    #print("from="+str(fromVectorAngle)+" to="+str(toVectorAngle))
    #angle= abs(toVectorAngle-fromVectorAngle)
    angle= toVectorAngle-fromVectorAngle
    while(angle>math.pi*2):
        angle-=math.pi/2
    while(angle<-math.pi*2):
        angle+=math.pi/2
    if(angle>math.pi*1.5):
        angle-=math.pi*1.5
    if(angle<-math.pi*1.5):
        angle+=math.pi*1.5
    #if(toVectorAngle<fromVectorAngle):
    #    return -angle
    return angle
    '''
    
    return math.atan2(fromVector.x*toVector.y-fromVector.y*toVector.x,fromVector.x*toVector.x+fromVector.y*toVector.y);
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
    #print(distance)
    if(distance>0):
        positionVector=Vector3(math.cos(tetha),math.sin(tetha),0).normalize()*abs(distance-0.2)
        positionVector+=CurrentPosition()
        
        (X_Point,Y_Point)=fromWorldToMatrix(positionVector)
        
        Map2D[Y_Point][X_Point]=-1
        
    
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
    global TargetPointFromPathfinder
    #(X_Pose,Y_Pose)=fromWorldToMatrix(TargetPose)
    #Map2D[Y_Pose][X_Pose]=-2
    #return TargetPose
    if(HasPointFromPathfinder):
        return TargetPointFromPathfinder
    else:
        return TargetPose
TargetPoint=Vector3()
def CurrentPath():
    global TargetPointFromPathfinder
    global HasPointFromPathfinder
    global SelectedTargetDistanceOnPath
    (My_X_Pose,My_Y_Pose)=fromWorldToMatrix(CurrentPosition())
    (Target_X_Pose,Target_Y_Pose)=fromWorldToMatrix(TargetPose)
    #Map2D[Y_Pose][X_Pose]=-2
    #return TargetPose
    #WorkMap=Map2D.copy()
    #WorkMap = [x[:] for x in Map2D]

    #WorkMap=
    WorkMap=[ [0]*MapSize for _ in range(MapSize) ]
    #Map2D
    for y in range(MapSize):
        for x in range(MapSize):
            WorkMap[y][x]=Map2D[y][x]
    
    MarkMap(WorkMap,My_X_Pose,My_Y_Pose,Target_X_Pose,Target_Y_Pose)
    Path=findPath(WorkMap,Target_X_Pose,Target_Y_Pose)
    Path.reverse()
    print(Path)
    
    if(len(Path)>1):
        #CurrentTargetPoint=Path[1]#Y,X
        print("GoTo"+str(Path[1]))
        
        TargetPathPoint=Path[len(Path)-1]
        HasTarget=False
        for i in range(len(Path)):
            if(CurrentPosition()-fromMatrixToWorldPosition(Path[i][0],Path[i][1])).Module()>SelectedTargetDistanceOnPath:
                TargetPathPoint=Path[i]
                HasTarget=True
                break
        
        HasPointFromPathfinder=HasTarget
        

        if(HasTarget):
            TargetPointFromPathfinder=fromMatrixToWorldPosition(TargetPathPoint[0],TargetPathPoint[1])
            print("Distance to target="+str( (TargetPointFromPathfinder-CurrentPosition()).Module() ))
            print("Repaired Position: "+str(fromMatrixToWorldPosition(TargetPathPoint[0],TargetPathPoint[1])))
    else:
        HasPointFromPathfinder=False
    
    #Disable Pathfinding
    #HasPointFromPathfinder=False
    print()
    #Path
    '''for y in range(MapSize-1,-1,-1):
        #print(WorkMap[MapSize-1-y])
        word=""
        for x in range(MapSize):
            markIt=False
            for from_Y, from_X in Path:
                if(y==from_Y and x== from_X):
                    markIt=True
                    break
            if(x==My_X_Pose and y== My_Y_Pose):
                    word+="R"
            
            elif(x==Target_X_Pose and y== Target_Y_Pose):
                    word+="T"
            elif(markIt):
                    #word+="+"
                    word+=str(WorkMap[y][x])
            elif(WorkMap[y][x]<0):
                word+="Z"
                
            else:
                word+="0"
        print(word)'''
    pass
def findPath(WorkMap,from_X,from_Y):
    Path=[]
    Path.append((from_Y,from_X))
    current_X=from_X
    current_Y=from_Y
    for id in range(WorkMap[from_Y][from_X]-1,0,-1):
        #print("TargetID="+str(WorkMap[current_Y][current_X]-1))
        massive=findLower(WorkMap,current_X,current_Y,WorkMap[current_Y][current_X]-1)
        if(len(massive)==2):
            current_Y,current_X=massive
        else:
            current_Y=from_Y
            current_X=from_X
        #print("ID="+str(id)+" Y="+str(current_Y)+" X="+str(current_X))
        Path.append((current_Y,current_X))
    #print(Path)
    return Path
def findLower(WorkMap,from_X,from_Y,NextID):
    if(from_X-1>0):
        if(WorkMap[from_Y][from_X-1]==NextID):
            #print(from_Y,from_X-1)
            return (from_Y,from_X-1)
    if(from_X+1<MapSize):
        if(WorkMap[from_Y][from_X+1]==NextID):
            #print(from_Y,from_X+1)
            return (from_Y,from_X+1)
    if(from_Y-1>=0):
        if(WorkMap[from_Y-1][from_X]==NextID):
            #print(from_Y-1,from_X)
            return (from_Y-1,from_X)
    if(from_Y+1<MapSize):
        if(WorkMap[from_Y+1][from_X]==NextID):
            #print(from_Y+1,from_X)
            return (from_Y+1,from_X)
    print("Target not found "+str(NextID))
    
    data=""
    for y in range(MapSize-1,-1,-1):
        #print(WorkMap[MapSize-1-y])
        word=""
        for x in range(MapSize):
            word+=str(WorkMap[y][x])+"\t"
            
        data+=(word+"\n")
    
    f = open("error.txt","w")
    f.write(data)
    f.close()
    pass
def MarkMap(WorkMap,from_X,from_Y,to_X,to_Y):
    WorkMap[from_Y][from_X]=1
    nextTargets=[(from_Y,from_X)]
    newTargets=[]
    lastID=2
    while(WorkMap[to_Y][to_X]==0):
        #newTargets.clear()
        if(len(nextTargets)>0):
            for value_Y,value_X in nextTargets:
                #print(value_Y,value_X)
                #print(value)
                massive=(MarkNeibours(WorkMap,value_X,value_Y,lastID))
                for work_Y,work_X in massive:
                    #print("Working"+str(work_Y)+" and "+str(work_X))
                    newTargets.append((work_Y,work_X))
            nextTargets.clear()
            #nextTargets.(newTargets)
            for work_Y,work_X in newTargets:
                nextTargets.append((work_Y,work_X))
            
            #print("nextTargets from new="+str(nextTargets))
            newTargets.clear()
            lastID+=1
        else:
            break
        #break
        
    pass
def MarkNeibours(WorkMap,X,Y,nextID):
    #id=WorkMap[Y][X]
    #print("MarkNeibours "+str(X)+" and "+str(Y))
    NextTargets=[]
    if(X-1>0):
        if(WorkMap[Y][X-1]==0):
            WorkMap[Y][X-1]=nextID
            NextTargets.append((Y,X-1))
            #MarkNeibours(WorkMap,X-1,Y)
        
    if(X+1<MapSize):
        if(WorkMap[Y][X+1]==0):
            WorkMap[Y][X+1]=nextID
            NextTargets.append((Y,X+1))
            #MarkNeibours(WorkMap,X+1,Y)
    if(Y-1>=0):
        if(WorkMap[Y-1][X]==0):
            WorkMap[Y-1][X]=nextID
            NextTargets.append((Y-1,X))
            #MarkNeibours(WorkMap,X,Y-1)
    if(Y+1<MapSize):
        if(WorkMap[Y+1][X]==0):
            WorkMap[Y+1][X]=nextID
            NextTargets.append((Y+1,X))
            #MarkNeibours(WorkMap,X,Y+1)
    #print("Next targets="+str(NextTargets))
    return NextTargets
    
def PrepareWorkers():
    print("Angle="+str(math.degrees(Angle2D(Vector3(1,0,0),Vector3(-1,0,0)))))
    PrepareGlobals()
    global poseSub
    global targetPoseSub
    global topLaser1Sub
    global topLaser2Sub
    global topLaser3Sub
    global topLaser4Sub
    global cmd_velPub
    poseSub=rospy.Subscriber("pose",geometry_msgs.msg.Pose2D,tcp_nodelay=True,queue_size=1,callback=subscriber_pose)
    
    targetPoseSub=rospy.Subscriber("TargetPosition",geometry_msgs.msg.Vector3,tcp_nodelay=True,queue_size=1,callback=subscriber_target_pose)
 
    topLaser1Sub=rospy.Subscriber("TopLaser1",geometry_msgs.msg.Vector3,tcp_nodelay=True,queue_size=1,callback=subscriber_top_laser)
    topLaser2Sub=rospy.Subscriber("TopLaser2",geometry_msgs.msg.Vector3,tcp_nodelay=True,queue_size=1,callback=subscriber_top_laser)
    topLaser3Sub=rospy.Subscriber("TopLaser3",geometry_msgs.msg.Vector3,tcp_nodelay=True,queue_size=1,callback=subscriber_top_laser)
    topLaser4Sub=rospy.Subscriber("TopLaser4",geometry_msgs.msg.Vector3,tcp_nodelay=True,queue_size=1,callback=subscriber_top_laser)
    
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
        AngleFromForwardToPoint=Angle2D(currentGlobalForward,VectorToPoint)
        #print("AngleFromForwardToPoint="+str(AngleFromForwardToPoint))
        DistanceToPoint=DistanceVector3(CurrentPosition(),CurrentTargetPoint())
        twist.angular=Vector3(0,0,0)
        twist.linear=Vector3()
        #print("DistanceToPoint="+str(DistanceToPoint))
        print("Angle="+str(AngleFromForwardToPoint))
        if(DistanceToPoint<-0.05):
            twist.linear=-1*CurrentLocalForward()
        elif(abs(AngleFromForwardToPoint)>0.15):
            if(abs(AngleFromForwardToPoint)>0.3):
                twist.angular.z=max(abs(AngleFromForwardToPoint),0.1)*sign(AngleFromForwardToPoint)
                #print("FAST from angle="+str(currentGlobalForward)+" toAngle="+str(VectorToPoint))
            else:
                twist.angular.z=max(abs(AngleFromForwardToPoint),0.01)*sign(AngleFromForwardToPoint)
                #print("SLOW")
            #print(twist.angular.z)
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
        
        
        cmd_velPub.publish(twist)
        
        #print(twist)
        CurrentPath()
        r.sleep()


def fromWorldToMatrix(worldPosition:Vector3):
    position=worldPosition-rightBottomCorner
    X_Point=int(position.x/5*(MapSize-1))
    Y_Point=int(position.y/5*(MapSize-1))
    if(X_Point<0):
        X_Point=0
    if(Y_Point<0):
        Y_Point=0
    
    if(X_Point>=MapSize):
        X_Point=MapSize-1
    if(Y_Point>=MapSize):
        Y_Point=MapSize-1
    return (X_Point,Y_Point)
def fromMatrixToWorldPosition(y,x):
    return (Vector3(x*5/(MapSize-1),y*5/(MapSize-1),0)+rightBottomCorner)
if __name__ == '__main__':
    try:
        
        PrepareWorkers()
        #Заготавливаем точки маршрута
        
        Worker()
    except rospy.ROSInterruptException:
        pass
