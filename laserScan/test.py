#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def visualize_coordinates():
    rospy.init_node('coordinate_visualizer', anonymous=True)

    # 创建一个发布器，用于发布可视化消息
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    rate = rospy.Rate(10)  # 发布频率为10Hz

    while not rospy.is_shutdown():
        # 创建一个球体可视化消息
        marker_msg = Marker()
        marker_msg.header.frame_id = "laser"
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.ns = "coordinates"
        marker_msg.action = Marker.ADD
        marker_msg.type = Marker.SPHERE_LIST
        marker_msg.scale.x = 0.1
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1
        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0

        # 添加两个坐标点
        points = [[4.225117408833038, -1.7934559605222702, 0.0], [3.4333139177211773, -1.1155513367720598, 0.0]]
        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            marker_msg.points.append(p)

        # 发布可视化消息
        marker_pub.publish(marker_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        visualize_coordinates()
    except rospy.ROSInterruptException:
        pass