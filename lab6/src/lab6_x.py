#!/usr/bin/env python
'''
Lab6 Description:
In this lab, you will integrate the tasks that have been built in 
previous two labs for the robot into a comprehensive task. 
Specifically, in Part I (20%), you will first learn how ROS 
package Smach works as a way of expressing the solution to a 
robot problem in terms of a (finite) state machine. Study the 
Smach tutorial to become familiar with the Smach package. 
Then duplicate the example in this tutorial and show it to your TA. 
In the second part (80%), refer to the environment shown in the 
right figure for which you have built a map already. In addition 
to the static map, there are three poles placed near P2, P3, and P4, shown as red circles in the figure. Their 
locations are not known a priori. The overall task of your robot is to visit P1, P2, P3 and P4 in turn and 
stop at the three poles, as you did in Lab 4. For a visit to a pole to be considered successful, your robot must 
come to a full stop within 15cm of the pole for at least two seconds to indicate that it has parked. The 
complete task requires your robot to, starting from P1, visit the four location markers and three poles before 
coming back to P1. When you have completed and tested your solution to Part II, demonstrate it to the 
instructor and a TA. Be prepared to answer questions regarding your implementation of the solution.
Please note that we strongly encourage you to consider using Smach in designing the solution for Part II 
although this is not mandatory. Just for your reference, Lab 7 will be a repeat of Lab 6 but on a competitive 
basis where the time of completion in executing the navigation task will be used to determine the rank of a 
student group and subsequently the final mark of the lab. So, it is to your advantage to build a solution 
that is easy to understand, tune, and optimize.
'''

import rospy
import smach
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan

class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['P1', 'P2', 'P3', 'P4','P5', 'navigation_failed'])
        self.point = None
        self.counter = 0 
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Navigate')
        
        if self.counter == 0:
            self.point = 'P2'
        elif self.counter == 1:
            self.point = 'P3'
        elif self.counter == 2:
            self.point = 'P4'
        elif self.counter == 3:
            self.point = 'P5'
        elif self.counter == 4:
            self.point = 'P1'
        self.counter += 1
        
        rospy.loginfo(f"Navigating to {self.point}")
        
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  
        goal.target_pose.header.stamp = rospy.Time.now()

        if self.point == 'P2':
            goal.target_pose.pose.position.x = 1.237
            goal.target_pose.pose.position.y = 0.0402
            goal.target_pose.pose.orientation.w = 1.0
        elif self.point == 'P3':
            goal.target_pose.pose.position.x = 0.476
            goal.target_pose.pose.position.y = -2.466
            goal.target_pose.pose.orientation.w = 1.0
        elif self.point == 'P4':
            goal.target_pose.pose.position.x = 3.884
            goal.target_pose.pose.position.y = -2.617
            goal.target_pose.pose.orientation.w = 1.0
        elif self.point == 'P5':
            goal.target_pose.pose.position.x = 2.847
            goal.target_pose.pose.position.y = 0.2802
            goal.target_pose.pose.orientation.w = 1.0
        elif self.point == 'P1':
            goal.target_pose.pose.position.x = 4.1408
            goal.target_pose.pose.position.y = 1.057
            goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending goal to {self.point}")
        client.send_goal(goal)

        client.wait_for_result()

        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Successfully navigated to {self.point}")
            return self.point
        else:
            rospy.loginfo(f"Failed to navigate to {self.point} ")
            return 'navigation_failed'

class Find_Pillar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pillar_find', 'pillar_not_find'], output_keys=['pillar_position'])
        self.pillar_position = None

    def execute(self, userdata):
        rospy.loginfo('Executing state Find_Pillar')
        
        # Subscribe to LaserScan and find the pillar
        laser_data = rospy.wait_for_message("/scan", LaserScan)
        self.pillar_position = self.find_pillar(laser_data)
        
        if self.pillar_position is not None:
            rospy.loginfo(f"Pillar found at position: {self.pillar_position}")
            userdata.pillar_position = self.pillar_position
            return 'pillar_find'
        else:
            rospy.loginfo("No pillar detected.")
            return 'pillar_not_find'

    def find_pillar(self, msg):
        # fine tune the parameters here
        cluster_size_threshold = 10
        dist_threshold = 0.2
        width_threshold = 0.07
        curvature_high_threshold = 30.0
        curvature_low_threshold = 0.05
        min_dist = float('inf')
        min_index = -1

        for i in range(1, len(msg.ranges) - 1):
            dist = msg.ranges[i]
            if not math.isfinite(dist) or dist <= 0.0:
                continue

            prev_dist = msg.ranges[i - 1]
            next_dist = msg.ranges[i + 1]
            curvature = abs(prev_dist - 2 * dist + next_dist)

            if curvature_low_threshold < curvature < curvature_high_threshold:
                cluster_size = 0
                cluster_width = 0.0

                for j in range(i, len(msg.ranges)):
                    if abs(msg.ranges[j] - dist) < dist_threshold:
                        cluster_size += 1
                        cluster_width += msg.angle_increment * msg.ranges[j]
                    else:
                        break

                if cluster_size < cluster_size_threshold and cluster_width < width_threshold and dist < min_dist:
                    min_dist = dist
                    min_index = i

        if min_index >= 0:
            ang = msg.angle_min + msg.angle_increment * min_index
            x = (min_dist - 0.12) * math.cos(ang)
            y = (min_dist - 0.12) * math.sin(ang)
            return [x, y]
        else:
            return None
        
class Park_Pillar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['parked', 'parked_failed'], input_keys=['pillar_position'])
        self.pillar_position = None
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Park_Pillar')
        self.pillar_position = userdata.pillar_position
        
        if self.pillar_position is None:
            rospy.loginfo("Pillar position not found")
            return 'parked_failed'
        
        # park the pillar
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link" 
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration(0.03)

        # set the xy_goal_tolerance to 0.15 to make the robot park at the 15cm around the pillar
        goal.target_pose.pose.position.x = self.pillar_position[0]
        goal.target_pose.pose.position.y = self.pillar_position[1]
        goal.target_pose.pose.orientation.w =  1.0

        rospy.loginfo(f"Sending goal to park at position {self.pillar_position}")
        client.send_goal(goal)

        client.wait_for_result()

        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Successfully parked at the pillar location")
            # stay 2 seconds
            rospy.sleep(2)
            return 'parked'  
        else:
            rospy.loginfo("Failed to park at the pillar location")
            return 'parked_failed' 
        
def main():
    rospy.init_node('smach_homing_task')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['task_complete', 'task_incomplete'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Navigate', Navigate(), 
                               transitions={'P1':'task_complete',
                                            'P2':'Find_Pillar', 
                                            'P3':'Find_Pillar', 
                                            'P4':'Find_Pillar',
                                            'P5':'Find_Pillar',
                                            'navigation_failed':'task_incomplete'})
        
        smach.StateMachine.add('Find_Pillar', Find_Pillar(), 
                               transitions={'pillar_find':'Park_Pillar',
                                            'pillar_not_find':'task_incomplete'})
        
        
        smach.StateMachine.add('Park_Pillar', Park_Pillar(), 
                               transitions={'parked':'Navigate',
                                            'parked_failed':'task_incomplete'})
        
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.loginfo(f"State Machine finished with outcome: {outcome}")

if __name__ == '__main__':
    main()