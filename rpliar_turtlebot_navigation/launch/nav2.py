
#-*- coding: UTF-8 -*-
#!/usr/bin/env python
 
import roslib; roslib.load_manifest('rbx1_nav')
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
m = 1
class NavTest():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        
        
        self.rest_time = rospy.get_param("~rest_time", 10)
        
        
        self.fake_test = rospy.get_param("~fake_test", False)
        
       
        locations = dict()
        
        locations['hall_foyer'] = Pose(Point(4.02, -0.0372, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))
        locations['hall_foyer'] = Pose(Point( 4.01, 1.46, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))
        locations['hall_kitchen'] = Pose(Point(-0.0045, 1.45, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))
        locations['hall_bedroom'] = Pose(Point(0.0127, -0.00337, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))
        #locations['living_room_1'] = Pose(Point(4.720, 9.551, 2.704), Quaternion(0.000, 0.000, 0.786, 0.618))
        #locations['living_room_2'] = Pose(Point(3.343, -0.859, -0.864), Quaternion(0.000, 0.000, 0.480, 0.877))
       # locations['dining_room_1'] = Pose(Point(25.687,36.060, 2.990), Quaternion(0.000, 0.000, 0.899, -0.451))
        #locations['dining_room_2'] = Pose(Point(2.341,37.278, 3.309), Quaternion(0.000, 0.000, 0.892, -0.451))
        #locations['dining_room_3'] = Pose(Point(-3.237,30.083, -1.531), Quaternion(0.000, 0.000, 0.896, -0.451))
        #locations['dining_room_4'] = Pose(Point(-2.747,11.047, -1.803), Quaternion(0.000, 0.000, 0.895, -0.451))
        
        
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
       
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
       
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        
        
        initial_pose = PoseWithCovarianceStamped()
        
       
        n_locations = len(locations)    
        
       
        start_time = rospy.Time.now()
       
        
        
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
       
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
       
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
            
        rospy.loginfo("Starting navigation test")
        
        m='true'
        
        while m:
           
            if i == n_locations:
                i = 0
                m=0
                
               
              
           
            mm=locations
            
 
                        
           
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x - 
                                    locations[last_location].position.x, 2) +
                                pow(locations[location].position.y - 
                                    locations[last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(locations[location].position.x - 
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y - 
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""
            
           
            i += 1
            n_goals += 1
           
            for k in mm:
                self.goal = MoveBaseGoal()
                self.goal.target_pose.pose = mm[k]
                self.goal.target_pose.header.frame_id = 'map'
                self.goal.target_pose.header.stamp = rospy.Time.now()
                
               
                self.move_base.send_goal(self.goal)
                
                
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
                
              
                
                
                
                rospy.sleep(self.rest_time)
            
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
 
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
      
def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])
 
if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
