
import math
import rospy
import geometry_msgs.msg
import math

#Тестирование поворачивания робота 

if __name__ == '__main__':
    try:
        rospy.init_node("brain")
        r=rospy.Rate(60)#60hz
        cmd_velPub=rospy.Publisher("cmd_vel",geometry_msgs.msg.Twist,tcp_nodelay=True,queue_size=1)
        
        while not(rospy.is_shutdown()):
            twist=geometry_msgs.msg.Twist()
            twist.angular.z=2
            cmd_velPub.publish(twist)
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass