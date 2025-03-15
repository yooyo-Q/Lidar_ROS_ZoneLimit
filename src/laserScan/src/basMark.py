# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA

def create_marker(header_frame_id, marker_type, scale, color):
    marker = Marker()
    marker.header.frame_id = header_frame_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.scale = scale
    marker.color = color
    marker.lifetime = rospy.Duration()  # 持久显示
    return marker

def create_text(header_frame_id,text):
    text_msg = Marker()
    text_msg.header.frame_id = header_frame_id
    text_msg.header.stamp = rospy.Time.now()
    text_msg.ns = "text_namespace"
    text_msg.id = 0
    text_msg.type = Marker.TEXT_VIEW_FACING
    text_msg.action = Marker.ADD

    text_msg.pose.position.x = -5.0
    text_msg.pose.position.y = 0.0
    text_msg.pose.position.z = 1.0

    text_msg.pose.orientation.x = 0.0
    text_msg.pose.orientation.y = 0.0
    text_msg.pose.orientation.z = 0.0
    text_msg.pose.orientation.w = 1.0

    text_msg.scale.x = 0.8
    text_msg.scale.y = 0.8
    text_msg.scale.z = 0.8

    text_msg.color.r = 1.0
    text_msg.color.g = 1.0
    text_msg.color.b = 0.0
    text_msg.color.a = 1.0

    text_msg.text = text

    text_msg.lifetime = rospy.Duration()  # 持久显示
    return text_msg

def create_spheres(points):
    marker_msg = Marker()
    marker_msg.header.frame_id = "laser"
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.ns = "coordinates"
    marker_msg.action = Marker.ADD
    marker_msg.type = Marker.SPHERE_LIST
    marker_msg.scale.x = 0.4
    marker_msg.scale.y = 0.4
    marker_msg.scale.z = 0.4
    marker_msg.color.r = 1.0
    marker_msg.color.g = 0.0
    marker_msg.color.b = 0.0
    marker_msg.color.a = 1.0

    # 添加两个坐标点
    # points = [[4.225117408833038, -1.7934559605222702, 0.0], [3.4333139177211773, -1.1155513367720598, 0.0]]
    for point in points:
        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = point[2]
        marker_msg.points.append(p)

    return marker_msg

