#Тестирование волнового алгоритма

MapSize=10
Map2D=[ [0]*MapSize for _ in range(MapSize) ]

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


MarkMap(Map2D,5,5,MapSize-1,MapSize-1)

for y in range(MapSize):
    print(Map2D[MapSize-1-y])
pass