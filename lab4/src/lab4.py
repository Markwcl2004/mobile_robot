#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
import math
import numpy as np
import time

class HuskyHighlevelController:
    def __init__(self, nh):
        self.nodeHandle = nh
        self.subscriber = None
        self.vel_pub = None
        self.viz_pub = None
        self.stop_srv = None
        self.marker = Marker()
        self.msg = Twist()
        self.ctrl_p = 0.1
        self.pillar_pos = np.zeros(2)
        self.temp=[]

        # Get param from config file
        #self.ctrl_p = rospy.get_param("controller_gain", 0.0)
        #topic = rospy.get_param("subscriber_topic", "")
        topic = "/scan"
        #queue_size = rospy.get_param("queue_size", 10)
        queue_size=10

        if not topic or queue_size is None:
            rospy.logerr("Could not find subscriber params!")
            rospy.signal_shutdown("Error in params")

        # Create subscriber
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.LaserCallback)

        # Create publishers
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.viz_pub = rospy.Publisher("visualization_marker", Marker, queue_size=0)

        # Service server
        self.stop_srv = rospy.Service("/start_stop", SetBool, self.start_stop)

        # Initialize pillar marker in RViz
        self.initPillarMarker()

        rospy.loginfo("Husky highlevel controller node launched!")

        # Init linear speed
        self.setVel(0.1, "forward")

    def setVel(self, vel, dof):
        if dof == "forward":
            self.msg.linear.x = vel
        elif dof == "ang":
            self.msg.angular.z = vel

    def DriveHusky(self):
        # Publish velocity to /cmd_vel
        self.vel_pub.publish(self.msg)
        rospy.loginfo(self.msg)

    def adjustHeading(self, ang):
        # Adjust heading using P control
        diff = -ang
        self.setVel(self.ctrl_p * diff, "ang")

    def vizPillar(self):
        # Visualize pillar in RViz
        self.marker.pose.position.x = self.pillar_pos[0]
        self.marker.pose.position.y = self.pillar_pos[1]
        self.marker.pose.position.z = -1.0
        self.viz_pub.publish(self.marker)

    def LaserCallback(self, msg):
    # 获取有效的激光扫描数据，过滤掉小于0.05或大于10的值
        valid_ranges = [r for r in msg.ranges if r > 0.05 and r < 10.0]
        print(type(msg.ranges))
    # 计算相邻点之间的差值
        diff_ranges = [abs(valid_ranges[i] - valid_ranges[i + 1]) for i in range(len(valid_ranges) - 1)]


        if len(diff_ranges)>1:
            sorted_indices = sorted(range(len(diff_ranges)), key=lambda k: diff_ranges[k], reverse=True)
            max_idx1, max_idx2 = sorted_indices[0], sorted_indices[1]
    
            # 映射回原始的 msg.ranges 中的索引
        # 注意 valid_ranges 和 msg.ranges 的索引有差异，所以我们需要调整
            original_idx1 = (msg.ranges).index(valid_ranges[max_idx1])  # 找到 valid_ranges 中最大值的原始索引
            original_idx2 = (msg.ranges).index(valid_ranges[max_idx2])  # 找到 valid_ranges 中第二大值的原始索引

        # 获取原始距离值
            dist1 = msg.ranges[original_idx1+1]
            dist2 = msg.ranges[original_idx2-1]

            avg_dist = msg.ranges[(int)((original_idx1 + original_idx2) / 2)]

        # 计算原始角度值
            ang1 = msg.angle_min + msg.angle_increment * original_idx1
            ang2 = msg.angle_min + msg.angle_increment * original_idx2
        # 计算这两个突变点的平均距离和平均角度
            # avg_dist = (dist1 + dist2) / 2
            avg_ang = (ang1 + ang2) / 2   
            avg_ang = avg_ang if avg_ang > math.pi else avg_ang - 2 * math.pi  
            rospy.loginfo("ang1: %.2f, ang2: %.2f degrees", ang1 * 180 / math.pi, ang2 * 180 / math.pi)
            #rospy.loginfo("Closest mutation points found:")
            #rospy.loginfo("Distance 1: %.2f, Angle 1: %.2f degrees", closest_dist1, closest_ang1 * 180.0 / math.pi)
            #rospy.loginfo("Distance 2: %.2f, Angle 2: %.2f degrees", closest_dist2, closest_ang2 * 180.0 / math.pi)
            rospy.loginfo("Average Distance: %.2f, Average Angle: %.2f degrees", avg_dist, avg_ang * 180.0 / math.pi)
        # 计算柱子相对于机器人的坐标
            self.pillar_pos[0] = avg_dist * math.cos(avg_ang)
            self.pillar_pos[1] = avg_dist * math.sin(avg_ang)
            rospy.loginfo("Pillar's coordinate to Husky is [%.2f, %.2f]", self.pillar_pos[0], self.pillar_pos[1])
        # 如果柱子距离机器人非常近，停止移动
            if avg_dist < 0.2:
                rospy.loginfo("Pillar is very close, stopping.")
                self.setVel(0.0, "forward")
                self.setVel(0.0, "ang")
            else:
            # 否则调整机器人朝向柱子
                self.adjustHeading(avg_ang)
                if (avg_ang * 180.0 / math.pi) > 30 :
                    self.setVel(0.1, "forward")  # 前进
                else:
                    self.setVel(0.2,"forword")
        else:
            rospy.loginfo("No closest mutation points found.")


    # 调整机器人的角度朝向柱子
        self.DriveHusky()

    # 可视化柱子
        self.vizPillar()



    def start_stop(self, request, response):
        if request.data:  # Start
            response.message = "Start Husky"
            self.setVel(3.0, "forward")
        else:  # Stop
            response.message = "Stop Husky"
            self.setVel(0.0, "forward")

        rospy.loginfo(response.message)
        response.success = True
        return response

    def initPillarMarker(self):
        # Initialize the pillar marker
        self.marker.header.frame_id = "base_laser"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "pillar"
        self.marker.id = 1
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = self.pillar_pos[0]
        self.marker.pose.position.y = self.pillar_pos[1]
        self.marker.pose.position.z = -1.0
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 2
        self.marker.color.a = 1.0  # Don't forget to set the alpha!
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0


if __name__ == "__main__":
    rospy.init_node("husky_highlevel_controller")

    controller = HuskyHighlevelController(rospy)

    rospy.spin()
