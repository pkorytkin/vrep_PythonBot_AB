import numpy as np
from Vector3 import *
import cv2
class MapWorker:
    #Число дискрет глобальной карты собираемой лидаром
    GlobalMapSize=100
    #Глобальная карта
    GlobalMap2D=[]
    #Правый нижний угол карты для перевода между матрицей и координатами фактическими
    
    rightBottomCorner=Vector3(-2.5,-2.5,0)
    #Размеры робота в глобальных координатах
    
    RobotSize=0.3/2
    #Число дискрет карты для построения маршрута
    PathfinderMapSize=200#Больше GlobalMap2D
    def __init__(self) -> None:
        self.GlobalMap2D=[ [0]*self.GlobalMapSize for _ in range(self.GlobalMapSize) ]
        
    def AddObstacle(self,lidar:geometry_msgs.msg.Vector3,CurrentPosition,theta):
            
        tetha=lidar.x+theta
        distance=lidar.z
        if(distance>self.RobotSize):
            #for obstacleDistance in np.arange(distance-self.RobotSize,distance+self.RobotSize/2,self.RobotSize/10):
            obstacleDistance=distance
            #print("From "+str(Vector3(math.cos(tetha),math.sin(tetha),0))+" "+str(Vector3(math.cos(tetha),math.sin(tetha),0).normalize()))
            positionVector=Vector3(math.cos(tetha),math.sin(tetha),0).normalize()*obstacleDistance
            #print("obstacle"+str(obstacleDistance)+" "+str(Vector3(math.cos(tetha),math.sin(tetha),0).normalize()*obstacleDistance))
            #print(positionVector,CurrentPosition,obstacleDistance)
            positionVector+=CurrentPosition
            
            (X_Point,Y_Point)=self.fromWorldToMatrix(positionVector,self.GlobalMapSize)
            
            self.GlobalMap2D[Y_Point][X_Point]=-1
    LastPath=None
    def CurrentPath(self,CurrentPosition,TargetPositionFromVREP,CurrentGlobalForward):
        #global TargetPointFromPathfinder
        #global HasPointFromPathfinder
        #global RobotSize
        TargetPointFromPathfinder=TargetPositionFromVREP
        MapScale=int(self.PathfinderMapSize/self.GlobalMapSize)
        #MapSize=self.GlobalMapSize/self.PathfinderMapSize
        
        
        (Target_X_Pose,Target_Y_Pose)=self.fromWorldToMatrix(TargetPositionFromVREP,self.PathfinderMapSize)
        '''
        '''
        Target_X_Pose=int(Target_X_Pose)
        Target_Y_Pose=int(Target_Y_Pose)
        #My_X_Pose=int(Target_X_Pose2)
        #My_Y_Pose=int(Target_Y_Pose2)
        (My_X_Pose,My_Y_Pose)=self.fromWorldToMatrix(CurrentPosition,self.PathfinderMapSize)
        My_X_Pose=int(My_X_Pose)
        My_Y_Pose=int(My_Y_Pose)
        #Target_X_Pose=int(My_X_Pose2)
        #Target_Y_Pose=int(My_Y_Pose2)
        PathfinderMap=[ [0]*self.PathfinderMapSize for _ in range(self.PathfinderMapSize) ]
        #PathfinderMap=np.zeros((self.PathfinderMapSize,self.PathfinderMapSize),dtype=np.int16)

        
        
        #Map2D
        for y in range(self.GlobalMapSize):
            for x in range(self.GlobalMapSize):
                #HasWall=False
                #for y_i in range(0,MapScale):
                #    for x_i in range(0,MapScale):
                if(self.GlobalMap2D[y][x]==-1):
                    for y_i in range(0,MapScale):
                        for x_i in range(0,MapScale):
                            PathfinderMap[y*MapScale+y_i][x*MapScale+x_i]=self.GlobalMap2D[y][x]
                            #HasWall=True
                            
                #if(HasWall):
                #    PathfinderMap[y][x]=1
                            

        ErodeCount=9
        #ErodeCount=int((self.RobotSize/(5/self.PathfinderMapSize)*1.5))
        self.ErodeMap(PathfinderMap,ErodeCount)
        
        for y_i in range(0,MapScale):
            for x_i in range(0,MapScale):
                PathfinderMap[My_Y_Pose+y_i][My_X_Pose+x_i]=0
                PathfinderMap[Target_Y_Pose+y_i][Target_X_Pose+x_i]=0
        
        self.MarkMap(PathfinderMap,My_X_Pose,My_Y_Pose,Target_X_Pose,Target_Y_Pose)
        
        #
        
        #print("ErodeCount="+str(ErodeCount))
        
        Path=self.findPath(PathfinderMap,Target_X_Pose,Target_Y_Pose)
        Path.reverse()
        
        #print(Path)
        
        if(len(Path)>1):
            #CurrentTargetPoint=Path[1]#Y,X
            print("GoTo"+str(Path[1]))
            
            TargetPathPoint=Path[1]
            
            #if()
            
            HasTarget=True
            #angle=math.degrees(Vector3.Angle2D(CurrentGlobalForward,self.fromMatrixToWorldPosition(TargetPathPoint[0],TargetPathPoint[1],self.PathfinderMapSize)-CurrentPosition))
            #distance=Vector3.DistanceVector3(self.fromMatrixToWorldPosition(TargetPathPoint[0],TargetPathPoint[1],self.PathfinderMapSize),CurrentPosition)
            #print(angle)
            '''if(abs(angle)<0.09):
                for i in range(len(Path)):
                    if(CurrentPosition-self.fromMatrixToWorldPosition(Path[i][0],Path[i][1],self.PathfinderMapSize)).Module()>self.RobotSize*2:
                        TargetPathPoint=Path[i]
                        #HasTarget=True
                        break'''
            #else:
                #HasTarget=False
            HasPointFromPathfinder=HasTarget

            if(HasTarget):
                
                
                TargetPointFromPathfinder=self.fromMatrixToWorldPosition(TargetPathPoint[0],TargetPathPoint[1],self.PathfinderMapSize)
                
                
                #print("Distance to target="+str( (TargetPointFromPathfinder-CurrentPosition).Module() ))
                #print("Repaired Position: "+str(fromMatrixToWorldPosition(self,TargetPathPoint[0],TargetPathPoint[1],self.PathfinderMapSize)))
        else:
            HasPointFromPathfinder=False
        
        
        return (HasPointFromPathfinder,TargetPointFromPathfinder,Path)
        #Disable Pathfinding
        #HasPointFromPathfinder=False
        
    def ErodeMap(self,SourceMap,ErodeCount=1):
        for id in range(ErodeCount):
            mapCopy=np.copy(SourceMap)
            for x in range(self.PathfinderMapSize):
                for y in range(self.PathfinderMapSize):
                    if(mapCopy[x][y]<0):
                        if(x-1>0):
                            SourceMap[x-1][y]=-1
                        if(x+1<self.PathfinderMapSize):
                            SourceMap[x+1][y]=-1
                        if(y-1>=0 ):
                            SourceMap[x][y-1]=-1
                        if(y+1<self.PathfinderMapSize):
                            SourceMap[x][y+1]=-1
                        
    def findPath(self,WorkMap,from_X,from_Y):
        Path=[]
        Path.append((from_Y,from_X))
        current_X=from_X
        current_Y=from_Y
        
        #print("CurrentPosition="+str(current_X)+" "+str(current_Y))
        #atemptOfFind=0
        #for atemptID in range(100):
        #    GoodDone=True
        '''if(WorkMap[from_Y][from_X]==0):
            print("Target was not marked!")
            Path.append((from_Y,from_X))
            
            return Path
        else:'''
        #for id in range(WorkMap[from_Y][from_X]-1,0,-1):
        while(True):
            #print("TargetID="+str(WorkMap[current_Y][current_X]-1))
            if(WorkMap[current_Y][current_X]<0):
                break
            massive=self.findLower(WorkMap,current_X,current_Y,WorkMap[current_Y][current_X]-1,self.PathfinderMapSize)
            if(len(massive)==2):
                current_Y,current_X=massive
                Path.append((current_Y,current_X))
                #print("found "+str(massive)+str(WorkMap[current_Y][current_X]))
            else:
                print("Next target not found "+str(len(Path)))
                #Try again find
                #atemptOfFind+=1
                #GoodDone=False
                break
            '''else:
                current_Y=from_Y
                current_X=from_X'''
            #print("ID="+str(id)+" Y="+str(current_Y)+" X="+str(current_X))
            #if(GoodDone):
            #    break
        #print("FindPath="+str(Path))
        return Path
    def findLower(self,WorkMap,from_X,from_Y,NextID,MapSize):

        if(from_X-1>0 ):
            if(WorkMap[from_Y][from_X-1]==NextID):
                #print(from_Y,from_X-1)
                return (from_Y,from_X-1)
        if(from_X+1<MapSize  ):
            if(WorkMap[from_Y][from_X+1]==NextID):
                #print(from_Y,from_X+1)
                return (from_Y,from_X+1)
        if(from_Y-1>=0  ):
            if(WorkMap[from_Y-1][from_X]==NextID):
                #print(from_Y-1,from_X)
                return (from_Y-1,from_X)
        if(from_Y+1<MapSize ):
            if(WorkMap[from_Y+1][from_X]==NextID):
                #print(from_Y+1,from_X)
                return (from_Y+1,from_X)
        
        #WorkMap[from_Y][from_X]=0
        print("Target not found "+str(NextID))
        return []

    #Помечает карту волнами
    def MarkMap(self,WorkMap,from_X,from_Y,to_X,to_Y):
        WorkMap[from_Y][from_X]=1
        nextTargets=[(from_Y,from_X)]
        newTargets=[]
        lastID=2
        if(WorkMap[to_Y][to_X]!=0):
            print("Mark not started1")
        while(WorkMap[to_Y][to_X]==0):
            #newTargets.clear()
            if(len(nextTargets)>0):
                for value_Y,value_X in nextTargets:
                    #print(value_Y,value_X)
                    #print(value)
                    massive=(self.MarkNeibours(WorkMap,value_X,value_Y,lastID))
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
    def MarkNeibours(self,WorkMap,X,Y,nextID):
        #id=WorkMap[Y][X]
        #print("MarkNeibours "+str(X)+" and "+str(Y))
        NextTargets=[]
        if(X-1>0):
            if(WorkMap[Y][X-1]==0):
                WorkMap[Y][X-1]=nextID
                NextTargets.append((Y,X-1))
                #MarkNeibours(WorkMap,X-1,Y)
            
        if(X+1<self.PathfinderMapSize):
            if(WorkMap[Y][X+1]==0):
                WorkMap[Y][X+1]=nextID
                NextTargets.append((Y,X+1))
                #MarkNeibours(WorkMap,X+1,Y)
        if(Y-1>=0):
            if(WorkMap[Y-1][X]==0):
                WorkMap[Y-1][X]=nextID
                NextTargets.append((Y-1,X))
                #MarkNeibours(WorkMap,X,Y-1)
        if(Y+1<self.PathfinderMapSize):
            if(WorkMap[Y+1][X]==0):
                WorkMap[Y+1][X]=nextID
                NextTargets.append((Y+1,X))
                #MarkNeibours(WorkMap,X,Y+1)
        #print("Next targets="+str(NextTargets))
        return NextTargets
    #Преобразование локальной позиции черепахи в мировую
    @staticmethod
    def LocalPositionToWorld(CurrentPosition,Vector:Vector3):
        return CurrentPosition+Vector
    #Мировая позиция в локальную
    @staticmethod
    def WorldToLocalPosition(CurrentPosition,Vector:Vector3):
        return Vector-CurrentPosition

    def fromWorldToMatrix(self,worldPosition:Vector3,MapSize):
        position=worldPosition-self.rightBottomCorner
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
    def fromMatrixToWorldPosition(self,y,x,MapSize):
        return (Vector3(x*5/(MapSize-1),y*5/(MapSize-1),0)+self.rightBottomCorner)