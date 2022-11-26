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
#Длину маршрута посчитать. не монотонная зависимоть выбора маршрута от альнодействия информационной системы. Если меняем дальнодействие информационной системы, т о маршрут будет меняться не непрерывно, а будут скачки.
#
# В какой СК работаем и как связано с отображаемой сколько единиц имеют клетки
# 
# #######################################################################Изображение данного видиния в виде данных лидара, почему не ровно
import math
import rospy
import geometry_msgs.msg
import math
from Vector3 import *
from MapWorker import *
from Math2 import *
import cv2
from threading import Thread
#Отображать позицию
PrintCalculateDistance=False

StartPoseSaved:bool=False
#Позиция куда нужно попасть из среды моделирования пробрасывается
TargetPositionFromVREP:Vector3=Vector3(0,0,0)

#Глобальная переменная для нынешней позиции
CurrentPose:geometry_msgs.msg.Pose2D=geometry_msgs.msg.Pose2D()
#Сохранённая стартовая позиция
StartPose:geometry_msgs.msg.Pose2D=geometry_msgs.msg.Pose2D()
#Сообщение поворота отправляемое в Topic
twist:geometry_msgs.msg.Twist=geometry_msgs.msg.Twist()

r:rospy.Rate

LinearSpeed:float=2

poseSub:rospy.Subscriber
cmd_velPub:rospy.Publisher
Path=[]
#2.5 2.5 - -2.5 -2.5 - смещения в симуляторе
#Карта собранная лидаром
#MapSize=50+1

#Целевая точка маршрута в глобальных координатах
TargetPointFromPathfinder=Vector3(0,0,0)
#Целевая точка маршрута существует True, иначе False
HasPointFromPathfinder=False

#Контейнер карты
MapContainer=MapWorker()
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
def subscriber_target_pose(targetPose:geometry_msgs.msg.Vector3):
    global TargetPositionFromVREP
    #print("SetTarget to"+str(Vector3(targetPose.x,targetPose.y,targetPose.z)))
    TargetPositionFromVREP=Vector3(targetPose.x,targetPose.y,targetPose.z)
def subscriber_top_laser(lidar:geometry_msgs.msg.Vector3):
    MapContainer.AddObstacle(lidar,CurrentPosition(),CurrentPose.theta)
def PrepareGlobals():
    global StartPoseSaved
    global CurrentPose
    global StartPose
    global r
    #Нужно сохранить стартовую позицию
    StartPose=geometry_msgs.msg.Pose2D()
    StartPoseSaved=False
    CurrentPose=geometry_msgs.msg.Pose2D()
    
    #Запускаем ноду мозга
    rospy.init_node("brain")
    r=rospy.Rate(60)
    print("Node ready")
def CurrentTargetPoint():
    global TargetPointFromPathfinder
    if(HasPointFromPathfinder):
        #print("pathfinded target="+str(TargetPointFromPathfinder))
        #return TargetPointFromPathfinder
    
        #(My_X_Pose,My_Y_Pose)=MapContainer.fromWorldToMatrix(CurrentPosition(),MapContainer.PathfinderMapSize)
        CurrentPos=CurrentPosition()
        bestDistance=100000
        bestTarget=MapContainer.fromMatrixToWorldPosition(Path[0][0],Path[0][1],MapContainer.PathfinderMapSize)
        for id in range(min(5,len(Path)),len(Path)):
            wordPosition=MapContainer.fromMatrixToWorldPosition(Path[id][0],Path[id][1],MapContainer.PathfinderMapSize)
            distance=Vector3.DistanceVector3(wordPosition,CurrentPos)
            if(distance<bestDistance):
                bestTarget=wordPosition
                bestDistance=distance
            else:
                break
        return bestTarget
                
                
    
    else:
        #print("vrep target="+str(TargetPositionFromVREP))
        return TargetPositionFromVREP
TargetPoint=Vector3()

def PrepareSubscribers():
    global poseSub
    global targetPoseSub
    global topLaser1Sub
    global topLaser2Sub
    global topLaser3Sub
    global topLaser4Sub
    
    poseSub=rospy.Subscriber("pose",geometry_msgs.msg.Pose2D,tcp_nodelay=True,queue_size=1,callback=subscriber_pose)
    
    targetPoseSub=rospy.Subscriber("TargetPosition",geometry_msgs.msg.Vector3,tcp_nodelay=True,queue_size=1,callback=subscriber_target_pose)
 
    topLaser1Sub=rospy.Subscriber("TopLaser1",geometry_msgs.msg.Vector3,tcp_nodelay=True,queue_size=1,callback=subscriber_top_laser)
    topLaser2Sub=rospy.Subscriber("TopLaser2",geometry_msgs.msg.Vector3,tcp_nodelay=True,queue_size=1,callback=subscriber_top_laser)
    topLaser3Sub=rospy.Subscriber("TopLaser3",geometry_msgs.msg.Vector3,tcp_nodelay=True,queue_size=1,callback=subscriber_top_laser)
    topLaser4Sub=rospy.Subscriber("TopLaser4",geometry_msgs.msg.Vector3,tcp_nodelay=True,queue_size=1,callback=subscriber_top_laser)
    
    print("Subscribers ready")
def PreparePublishers():
    global cmd_velPub
    cmd_velPub=rospy.Publisher("cmd_vel",geometry_msgs.msg.Twist,tcp_nodelay=True,queue_size=1)
    print("Publishers ready")
def PrepareWorkers():
    print("Current Angle="+str(math.degrees(Vector3.Angle2D(Vector3(1,0,0),Vector3(-1,0,0)))))
    
    PrepareGlobals()
    
    PrepareSubscribers()
    PreparePublishers()
    #Спим несколько раз чтоб дождаться позиции в subscriber
    r.sleep()
    r.sleep()
    r.sleep()
    r.sleep()



MovePoints=[]
def CalculateDistanceForAllPoints(points):
    dist=0
    for id in range(1,len(points)):
        dist+=Vector3.DistanceVector3(points[id-1],points[id])
        
    return dist*2

def CalculatePathLength():
    #Сохраняем точки, чтоб подсчитывать пройденый путь.
    pose=CurrentPosition()
    if(pose.x!=0 or pose.y!=0 or pose.z!=0):
        if(len(MovePoints)==0):
            MovePoints.append(pose)
        else:
            if(Vector3.DistanceVector3(MovePoints[len(MovePoints)-1],pose)>0.01):
                MovePoints.append(pose)
    if(PrintCalculateDistance):
        print(CalculateDistanceForAllPoints(MovePoints))
    
def WorkMovement(printLog=False):
    #print(CurrentTargetPoint())
    VectorToPoint=CurrentTargetPoint()-CurrentPosition()
    
    currentGlobalForward=CurrentGlobalForward()
    
    AngleFromForwardToPoint=Vector3.Angle2D(currentGlobalForward,VectorToPoint)
    
    DistanceToPoint=Vector3.DistanceVector3(CurrentPosition(),CurrentTargetPoint())
    twist.angular=Vector3(0,0,0)
    twist.linear=Vector3()
    
    
    if(DistanceToPoint<-0.05):
        twist.linear=-1*CurrentLocalForward()
    elif(abs(AngleFromForwardToPoint)>0.3):
        print("Rotation")
        if(abs(AngleFromForwardToPoint)>1):
            twist.angular.z=max(abs(AngleFromForwardToPoint),0.1)*Math2.sign(AngleFromForwardToPoint)
            #print("FAST from angle="+str(currentGlobalForward)+" toAngle="+str(VectorToPoint))
        else:
            twist.angular.z=max(abs(AngleFromForwardToPoint),0.01)*Math2.sign(AngleFromForwardToPoint)
            #print("SLOW")
    else:
        print("Forward")
        if(HasPointFromPathfinder):
            if(DistanceToPoint>0.1):
                twist.linear=CurrentLocalForward()*LinearSpeed
            elif DistanceToPoint<-0.1:
                twist.linear=CurrentLocalForward()*-1*LinearSpeed
            else:
                twist.linear=CurrentLocalForward()*DistanceToPoint*4
            '''if(DistanceToPoint>0.05):
                
                twist.linear=CurrentLocalForward()*LinearSpeed#(CurrentLocalForward()*max(DistanceToPoint,0.01)).convertToMSG()
            elif(DistanceToPoint<-0.05):
                twist.linear=-1*CurrentLocalForward()*max(min(DistanceToPoint,1),0.1)*LinearSpeed#(CurrentLocalForward()*(-max(DistanceToPoint,0.01))).convertToMSG()
        '''
        else:
            twist.linear=CurrentLocalForward()*-1
    
    
    #twist.linear.x=1
    #twist.angular.z=-1
    #print("AngleFromForwardToPoint="+str(AngleFromForwardToPoint))

    #Публикуем направление движения
    #print("Distance="+str(DistanceToPoint))
    #print("CurrentTargetPoint="+str(CurrentTargetPoint()))
    #print("twist="+str(twist))
    cmd_velPub.publish(twist)
    if(printLog):
        print(CurrentPosition())
        print("Angle="+str(AngleFromForwardToPoint))
        print("Tetha="+str(CurrentPose.theta))
        print("CurrentTargetPoint="+str(CurrentTargetPoint()))
        print("CurrentPosition="+str(CurrentPosition()))
        print("VectorToPoint="+str(VectorToPoint))
        print("CurrentGlobalForward="+str(currentGlobalForward))
        print("AngleFromForwardToPoint="+str(AngleFromForwardToPoint))
        print("DistanceToPoint="+str(DistanceToPoint))
def WorkPathfinder():
    global HasPointFromPathfinder
    global TargetPointFromPathfinder
    global Path
    (HasPointFromPathfinder,TargetPointFromPathfinder,Path)=MapContainer.CurrentPath(CurrentPosition(),TargetPositionFromVREP,CurrentGlobalForward())
    #print(HasPointFromPathfinder,Path)
    #Y [0], X[1]
    
    
def ShowMapImage():
    global Path
    cv2.namedWindow('map', cv2.WINDOW_NORMAL)
    while not (rospy.is_shutdown()):
        PathLocal=np.copy(Path)
        if(len(PathLocal)>0):
            img = np.zeros([MapContainer.PathfinderMapSize,MapContainer.PathfinderMapSize,3])
            MapScale=int(MapContainer.PathfinderMapSize/MapContainer.GlobalMapSize)
            #print(MapScale)
            #Красим Obstacle
            for y in range(MapContainer.PathfinderMapSize):
                for x in range(MapContainer.PathfinderMapSize):
                    if(MapContainer.GlobalMap2D[int(y/MapScale)][int(x/MapScale)]==-1):
                        img[x][y][:]=1
            
            '''for y in range(MapContainer.GlobalMapSize):
                for x in range(MapContainer.GlobalMapSize):
                    if(MapContainer.GlobalMap2D[y][x]==-1):
                        for y_i in range(0,MapScale):
                            for x_i in range(0,MapScale):
                                img[y*MapScale+y_i][x*MapScale+x_i][:]=1
                                '''
                                
            '''HasWall=False
            for y_i in range(0,MapScale):
                for x_i in range(0,MapScale):
                    if(MapContainer.GlobalMap2D[int(y*MapScale+y_i)][int(x*MapScale+x_i)]!=0):
                        HasWall=True
                        
            if(HasWall):
                img[x][y][:]=1'''
                #print(HasWall)
            #else:
            #img[y][x][:]=0
            #print(img)
            #Красим путь
            PathPoints=len(PathLocal)
            for id in range(PathPoints):
                if(len(PathLocal[id])==2):
                    (Path_Y,Path_X)=PathLocal[id]
                    if(img[Path_X][Path_Y][0]!=0 and img[Path_X][Path_Y][1]!=0 and img[Path_X][Path_Y][2]!=0):
                        img[Path_X][Path_Y][:]=0.5
                    else:
                        img[Path_X][Path_Y][:]=0
                        img[Path_X][Path_Y][1]=(1/(PathPoints+20))*(id+20)
            #Красим робота
            (My_X_Pose,My_Y_Pose)=MapContainer.fromWorldToMatrix(CurrentPosition(),MapContainer.PathfinderMapSize)
            (NextPoint_X_Pose,NextPoint_Y_Pose)=MapContainer.fromWorldToMatrix(CurrentTargetPoint(),MapContainer.PathfinderMapSize)
            NextPoint_X_Pose=int(NextPoint_X_Pose)
            NextPoint_Y_Pose=int(NextPoint_Y_Pose)
            My_X_Pose=int(My_X_Pose)
            My_Y_Pose=int(My_Y_Pose)
            #Красим цель
            (Target_X_Pose,Target_Y_Pose)=MapContainer.fromWorldToMatrix(TargetPositionFromVREP,MapContainer.PathfinderMapSize)
            Target_X_Pose=int(Target_X_Pose)
            Target_Y_Pose=int(Target_Y_Pose)
            

            for y_i in range(0,MapScale):
                for x_i in range(0,MapScale):
                    img[My_X_Pose+x_i][My_Y_Pose+y_i][:]=0
                    img[My_X_Pose+x_i][My_Y_Pose+y_i][0]=1
                    img[Target_X_Pose+x_i][Target_Y_Pose+y_i][:]=0
                    img[Target_X_Pose+x_i][Target_Y_Pose+y_i][2]=1
                    
                    #img[Target_X_Pose+x_i][Target_Y_Pose+y_i][:]=0
                    #img[Target_X_Pose+x_i][Target_Y_Pose+y_i][2]=1

            img[NextPoint_X_Pose][NextPoint_Y_Pose][:]=0
            img[NextPoint_X_Pose][NextPoint_Y_Pose][1:3]=1
            '''img[My_X_Pose][My_Y_Pose][:]=0
            img[My_X_Pose][My_Y_Pose][0]=1
            img[Target_X_Pose][Target_Y_Pose][:]=0
            img[Target_X_Pose][Target_Y_Pose][2]=1'''
            
            
            cv2.imshow("map",img)
            #print("Show image")
            cv2.waitKey(1)
def Worker():
    #global CurrentPointID
    #global PointsCount
    while not(rospy.is_shutdown()):
        CalculatePathLength()
        WorkMovement()
        #CurrentPath()
        WorkPathfinder()
        r.sleep()



if __name__ == '__main__':
    try:
        
        PrepareWorkers()
        #Заготавливаем точки маршрута
        #cv2.namedWindow("map", cv2.WINDOW_NORMAL)
        thread = Thread(target = ShowMapImage)
        thread.start()
        
        Worker()
        
    except rospy.ROSInterruptException:
        pass
