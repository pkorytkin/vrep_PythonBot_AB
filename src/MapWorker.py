import numpy as np
from Vector3 import *
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
            obstacleDistance=distance
            positionVector=Vector3(math.cos(tetha),math.sin(tetha),0).normalize()*obstacleDistance
            positionVector+=CurrentPosition
            (X_Point,Y_Point)=self.fromWorldToMatrix(positionVector,self.GlobalMapSize)
            
            self.GlobalMap2D[Y_Point][X_Point]=-1
    LastPath=None
    def CurrentPath(self,CurrentPosition,TargetPositionFromVREP,CurrentGlobalForward):
        TargetPointFromPathfinder=TargetPositionFromVREP
        MapScale=int(self.PathfinderMapSize/self.GlobalMapSize)
        (Target_X_Pose,Target_Y_Pose)=self.fromWorldToMatrix(TargetPositionFromVREP,self.PathfinderMapSize)
        Target_X_Pose=int(Target_X_Pose)
        Target_Y_Pose=int(Target_Y_Pose)
        (My_X_Pose,My_Y_Pose)=self.fromWorldToMatrix(CurrentPosition,self.PathfinderMapSize)
        My_X_Pose=int(My_X_Pose)
        My_Y_Pose=int(My_Y_Pose)
        PathfinderMap=[ [0]*self.PathfinderMapSize for _ in range(self.PathfinderMapSize) ]
        #Map2D
        for y in range(self.GlobalMapSize):
            for x in range(self.GlobalMapSize):
                if(self.GlobalMap2D[y][x]==-1):
                    for y_i in range(0,MapScale):
                        for x_i in range(0,MapScale):
                            PathfinderMap[y*MapScale+y_i][x*MapScale+x_i]=self.GlobalMap2D[y][x]
        ErodeCount=9
        self.ErodeMap(PathfinderMap,ErodeCount)
        
        for y_i in range(0,MapScale):
            for x_i in range(0,MapScale):
                PathfinderMap[My_Y_Pose+y_i][My_X_Pose+x_i]=0
                PathfinderMap[Target_Y_Pose+y_i][Target_X_Pose+x_i]=0
        
        self.MarkMap(PathfinderMap,My_X_Pose,My_Y_Pose,Target_X_Pose,Target_Y_Pose)
        Path=self.findPath(PathfinderMap,Target_X_Pose,Target_Y_Pose)
        Path.reverse()
        if(len(Path)>1):
            print("GoTo"+str(Path[1]))
            TargetPathPoint=Path[1]
            HasPointFromPathfinder=True
            TargetPointFromPathfinder=self.fromMatrixToWorldPosition(TargetPathPoint[0],TargetPathPoint[1],self.PathfinderMapSize)
        else:
            HasPointFromPathfinder=False
        #Disable Pathfinding
        #HasPointFromPathfinder=False
        return (HasPointFromPathfinder,TargetPointFromPathfinder,Path)
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
        while(True):
            if(WorkMap[current_Y][current_X]<0):
                break
            massive=self.findLower(WorkMap,current_X,current_Y,WorkMap[current_Y][current_X]-1,self.PathfinderMapSize)
            if(len(massive)==2):
                current_Y,current_X=massive
                Path.append((current_Y,current_X))
            else:
                #print("Next target not found "+str(len(Path)))
                break
        return Path
    def findLower(self,WorkMap,from_X,from_Y,NextID,MapSize):
        if(from_X-1>0 ):
            if(WorkMap[from_Y][from_X-1]==NextID):
                return (from_Y,from_X-1)
        if(from_X+1<MapSize  ):
            if(WorkMap[from_Y][from_X+1]==NextID):
                return (from_Y,from_X+1)
        if(from_Y-1>=0  ):
            if(WorkMap[from_Y-1][from_X]==NextID):
                return (from_Y-1,from_X)
        if(from_Y+1<MapSize ):
            if(WorkMap[from_Y+1][from_X]==NextID):
                return (from_Y+1,from_X)
        #print("Target not found "+str(NextID))
        return []
    #Помечает карту волнами
    def MarkMap(self,WorkMap,from_X,from_Y,to_X,to_Y):
        WorkMap[from_Y][from_X]=1
        nextTargets=[(from_Y,from_X)]
        newTargets=[]
        lastID=2
        if(WorkMap[to_Y][to_X]!=0):
            print("Mark not started")
        while(WorkMap[to_Y][to_X]==0):
            if(len(nextTargets)>0):
                for value_Y,value_X in nextTargets:
                    massive=(self.MarkNeibours(WorkMap,value_X,value_Y,lastID))
                    for work_Y,work_X in massive:
                        newTargets.append((work_Y,work_X))
                nextTargets.clear()
                for work_Y,work_X in newTargets:
                    nextTargets.append((work_Y,work_X))
                newTargets.clear()
                lastID+=1
            else:
                break
        pass
    def MarkNeibours(self,WorkMap,X,Y,nextID):
        NextTargets=[]
        if(X-1>0):
            if(WorkMap[Y][X-1]==0):
                WorkMap[Y][X-1]=nextID
                NextTargets.append((Y,X-1))
        if(X+1<self.PathfinderMapSize):
            if(WorkMap[Y][X+1]==0):
                WorkMap[Y][X+1]=nextID
                NextTargets.append((Y,X+1))
        if(Y-1>=0):
            if(WorkMap[Y-1][X]==0):
                WorkMap[Y-1][X]=nextID
                NextTargets.append((Y-1,X))
        if(Y+1<self.PathfinderMapSize):
            if(WorkMap[Y+1][X]==0):
                WorkMap[Y+1][X]=nextID
                NextTargets.append((Y+1,X))
        return NextTargets
    #Преобразование локальной позиции в мировую
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