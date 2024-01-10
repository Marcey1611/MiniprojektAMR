#! /usr/bin/env python3

#from logging import shutdown
#from math import inf
from math import inf
import rospy
import actionlib
from me_223965_miniprj.msg import StartAction, StartGoal, StartFeedback, StartResult
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

twist = Twist()
laserData = 0
step=0
countdown = 1000

def getLaserInfos(data):
    global laserData 
    laserData = data

def callback(goal):
    global step
    r = rospy.Rate(20)
    if(goal.start_driving is not True):
        rospy.loginfo('goal.start isnt true!')
        return
    if(goal.start_driving is True):
        while not rospy.is_shutdown():
            if step==0:
                rospy.loginfo("step0")
                if(laserData.ranges[0]<=0.3):
                    twist.linear.x=0
                    step=step+1
                else:
                    twist.linear.x=0.1
                    feedback.current_state="driving to wall"
            elif step==1:
                rospy.loginfo("step1")
                if((round(laserData.ranges[265]+0.001,3)==round(laserData.ranges[275],3) or round(laserData.ranges[265],3)==round(laserData.ranges[275]+0.001,3))and np.mean(laserData.ranges[265:275])<=0.5):
                    twist.angular.z=0
                    step=step+1
                else:
                    twist.angular.z=0.1
                    feedback.current_state="turning around that I stand paralel to the wall"
            elif step==2:
                rospy.loginfo("step2")
                twist.linear.x=0.05
                feedback.current_state="following the wall"
                #if laserData.ranges[260] != 0.0 and laserData.ranges[260] != inf and laserData.ranges[280] != 0.0 and laserData.ranges[280] != inf:
                if laserData.ranges[280]<0.5:
                    if (round(laserData.ranges[260],3)<=round(laserData.ranges[280],3)):
                        twist.angular.z=-0.2
                    elif round(laserData.ranges[260],3)>=round(laserData.ranges[280],3):
                        twist.angular.z=0.2
                    else:
                        twist.angular.z=0
                elif laserData.ranges[280]>=0.5:
                    step=step+1
                    twist.angular.z=0
                    twist.linear.x=0
            elif step==3:
                twist.linear.x=0.05
                twist.angular.z=-0.1

            pubTwist.publish(twist)
            action_server.publish_feedback(feedback)
            r.sleep()                   

def shutdown():
    rospy.loginfo("SHUTDOWN/ABORTED")
    twist.linear.x=0
    twist.angular.z=0
    pubTwist.publish(twist)
    
rospy.init_node('follow_wall_as')
pubTwist = rospy.Publisher('cmd_vel', Twist, queue_size=10)
subLaser=rospy.Subscriber('/scan', LaserScan, getLaserInfos)
action_server = actionlib.SimpleActionServer('mini_project_action_server', StartAction, callback, auto_start = False)
action_server_name = 'Follow Wall Action Server'
action_server.start()

feedback = StartFeedback()
result = StartResult()

rospy.on_shutdown(shutdown)

rospy.spin()