import rospy
import geometry_msgs.msg
from datetime import datetime
from threading import Thread
from Vector3 import *
StartPoseSaved:bool=False
CurrentPose:geometry_msgs.msg.Pose2D=geometry_msgs.msg.Pose2D()
#https://stackoverflow.com/questions/22753160/how-do-i-accept-input-from-arrow-keys-or-accept-directional-input/22755221#22755221
class _Getch:
   """Gets a single character from standard input.  Does not echo to the
screen."""
   def __init__(self):
      self.impl = _GetchUnix()
   def __call__(self):# return self.impl()
      charlist = []
      counter = 0
      for i in range(3):
         try:charlist.append(self.impl())
         except:pass
         if charlist[i] not in [chr(27),chr(91)]:#TODO sort out escape vs arrow duh use len()
            break
         if len(charlist) > 1:
            if charlist == [chr(27),chr(27)]:
               break
      if len(charlist) == 3:
         if charlist[2] == 'a' or charlist[2] == 'A':
            return 'u-arr'
         if charlist[2] == 'b' or charlist[2] == 'B':
            return 'd-arr'
         if charlist[2] == 'c' or charlist[2] == 'C':
            return 'r-arr'
         if charlist[2] == 'd' or charlist[2] == 'D':
            return 'l-arr'
      if(len(charlist)>0):
            return charlist[0]
      return ''
class _GetchUnix:
   def __init__(self):
      import tty, sys
   def __call__(self):
      import sys, tty, termios
      fd = sys.stdin.fileno()
      old_settings = termios.tcgetattr(fd)
      try:
         tty.setraw(sys.stdin.fileno())
         ch = sys.stdin.read(1)
      finally:
         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
      return ch
def getNowTime():
    now = datetime.now()
    return now.year*3.154e+13+now.month*2628336213698.6298828+now.day*86410958904.109588623+now.hour*3600456621.0045661926+now.minute*60007610.35007610172+now.second*1000126.8391679350752
def KeyReader():
    getch = _Getch()
    global key
    global keyTime
    while not(rospy.is_shutdown()):
        key=getch()
        keyTime=getNowTime()
MovePoints=[]
def CalculateDistanceForAllPoints(points):
    dist=0
    for id in range(1,len(points)):
        dist+=Vector3.DistanceVector3(points[id-1],points[id])
    return dist*2
def Worker():
    global twist
    global cmd_velPub
    global key
    global keyTime
    keyTime=0
    key=""
    twist=geometry_msgs.msg.Twist()
    while not(rospy.is_shutdown()):
        pose=CurrentPosition()
        if(pose.x!=0 or pose.y!=0 or pose.z!=0):
            if(len(MovePoints)==0):
                MovePoints.append(pose)
            else:
                if(Vector3.DistanceVector3(MovePoints[len(MovePoints)-1],pose)>0.01):
                    MovePoints.append(pose)
        d=CalculateDistanceForAllPoints(MovePoints)
        print(d)
        twist.linear.x=0
        twist.angular.z=0
        timeNow=getNowTime()
        linearSpeed=5
        if(timeNow-keyTime<0.1):
            if(key=='u-arr'):
                twist.linear.x=1
            elif(key=='d-arr'):
                twist.linear.x=-1
            elif(key=='l-arr'):
                twist.angular.z=1
            elif(key=='r-arr'):
                twist.angular.z=-1
        twist.linear.x*=linearSpeed
        cmd_velPub.publish(twist)
def subscriber_pose(pose:geometry_msgs.msg.Pose2D):
    global CurrentPose
    #Сохранение стартовой позиции
    global StartPoseSaved
    global StartPose
    if(not StartPoseSaved):
        StartPose=pose
        StartPoseSaved=True
    #Сохранение нынешней позиции
    CurrentPose=pose
    pass
def CurrentPosition():
    #z=CurrentPose.z
    global CurrentPose
    position=Vector3(x=CurrentPose.x,y=CurrentPose.y)
    return position
def PrepareWorkers():
    global cmd_velPub
    global poseSub
    global r
    global keyThread
    cmd_velPub=rospy.Publisher("/cmd_vel",geometry_msgs.msg.Twist,tcp_nodelay=True,queue_size=1)
    poseSub=rospy.Subscriber("pose",geometry_msgs.msg.Pose2D,tcp_nodelay=True,queue_size=1,callback=subscriber_pose)
    rospy.init_node("brain")
    keyThread = Thread(target=KeyReader)
    keyThread.start()
    r=rospy.Rate(60)
if __name__ == '__main__':
    try:
        PrepareWorkers()
        #Заготавливаем точки маршрута
        Worker()
    except rospy.ROSInterruptException:
        pass
