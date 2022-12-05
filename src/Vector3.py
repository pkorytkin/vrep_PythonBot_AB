import numpy as np
import geometry_msgs.msg
import math
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
        elif(type(other)==float or type(other)==np.float32 or type(other)==np.float64 or type(other)==int or type(other)==np.int16):
            return Vector3(self.x*other,self.y*other,self.z*other)
        else:
            print("TYPE NOT FOUND "+str(type(other)))
            raise Exception("Type NOT FOUND"+str(type(other)))
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
    @staticmethod
    def DistanceVector3(fromPosition,toPosition):
        return (fromPosition-toPosition).Module()
    @staticmethod
    def DotProduct(fromVector,toVector):
        return fromVector*toVector
    @staticmethod
    def CrossProduct(fromVector,toVector):
        return fromVector.x*toVector.y-fromVector.y*toVector.x
    @staticmethod
    def Angle2D(fromVector,toVector):
        return math.atan2(fromVector.x*toVector.y-fromVector.y*toVector.x,fromVector.x*toVector.x+fromVector.y*toVector.y)
