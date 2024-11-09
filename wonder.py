import math
import random

import rospy # needed for ROS
import numpy as np # for map arrays
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist      # ROS Twist message
from sensor_msgs.msg import LaserScan    # ROS laser msg




# ROS Topics
#
motionTopic='/cmd_vel' # turtlebot vel topic
laserTopic = '/scan'   # laser ranging topic

#global variable, to communicate with callbacks

gBumperLeft,gBumperRight= False, False # left/right close




# callback for the laser range data
# just sets the global Bump flags
#
def callback_laser(msg):
    '''Call back function for laser range data'''
    global gBumperLeft,gBumperRight
    
    gBumperLeft,gBumperRight=False,False
    numRays = len(msg.ranges) # total num readings
    radPerIndex = math.radians(360)/numRays
    
    width = int(numRays/6) # left/right bumper 'window'
    tooClose=0.5 # threshold for bumper to activate
    
    for i in range(0,len(msg.ranges)):
        #rule out bad readings first
        if not math.isnan( msg.ranges[i] ) and \
           not math.isinf( msg.ranges[i] ) and \
           msg.ranges[i]>0:
        
           # check for anything close left and right
           if msg.ranges[i]<tooClose:
               if i in range(0,width+1):
                   gBumperLeft=True
                   
               elif i in range(numRays-width,numRays+1):
                   gBumperRight=True
                   
    return


# wander_node - 
# Moves frandomly until it detects a close surface
# then avoids it

def wander_node():
    '''continually move forward until a close surface is detected'''
    global gBumper

    # all ROS 'nodes' (ie your program) have to do the following
    rospy.init_node('Wander', anonymous=True)

    # register as a ROS publisher for the velocity topic 
    vel_pub = rospy.Publisher(motionTopic, Twist, queue_size=10)
    # register as a subscribe for the laser scan topic
    scan_sub = rospy.Subscriber(laserTopic, LaserScan, callback_laser)

    # this is how frequently the loop below iterates
    rate = rospy.Rate(10) # Hz

    msg = Twist() # new velocity message
    msg.linear.x,msg.angular.z=0,0
    vel_pub.publish(msg) # stop all motors
    t = 0
    
    while not rospy.is_shutdown():
        # Simple OA strategy
        if t==0: # new velocities
            lvel = float(random.randint(0,5))/10.0
            avel = float(random.randint(-1,1))/10.0
            t = random.randint(1,40) 

        msg.linear.x,msg.angular.z=lvel,avel
         
        if gBumperLeft or gBumperRight: 
            msg.linear.x,msg.angular.z=-0.1,avel*10
            
        t = t-1
        vel_pub.publish(msg)
        rate.sleep()

    return

#
# This function is called by ROS when you stop ROS
# Here we use it to send a zero velocity to robot
# in case it was moving when you stopped ROS
#

def callback_shutdown():
    print("Shutting down")
    pub = rospy.Publisher(motionTopic, Twist, queue_size=10)
    msg = Twist()
    msg.angular.z=0.0
    msg.linear.x=0.0
    pub.publish(msg) 
    return



#-------------------------------MAIN  program----------------------
if __name__ == '__main__':
    try:
        rospy.on_shutdown(callback_shutdown)
        wander_node()
    except rospy.ROSInterruptException:
        pass