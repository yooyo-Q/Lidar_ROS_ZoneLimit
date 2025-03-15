# -*- coding: utf-8 -*-

import json
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from baseMark import create_marker

def create_marker(header_frame_id, marker_type, scale, color):
    marker = Marker()
    marker.header.frame_id = header_frame_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.scale = scale
    marker.color = color
    marker.lifetime = rospy.Duration()  # 持久显示
    return marker

def draw_lidar():
    lidar_marker = create_marker("laser", Marker.SPHERE, Vector3(0.2, 0.2, 0.2), ColorRGBA(1.0, 0.0, 0.0, 1.0))
    lidar_marker.pose.position = Point(0.0, 0.0, 0.0)
    lidar_marker.pose.orientation.w = 1.0  # 设置原点的方向
    return lidar_marker


def draw_arrow():
    arrow_marker = create_marker("laser", Marker.ARROW, Vector3(1, 0.1, 0.1), ColorRGBA(1.0, 0.0, 0.0, 1.0))
    arrow_marker.pose.position = Point(0, 0, 0)
    # 直接设置四元数分量，使箭头指向 Y 轴正方向
    arrow_marker.pose.orientation.x = 0.0
    arrow_marker.pose.orientation.y = 0.0
    arrow_marker.pose.orientation.z = 0.7071068
    arrow_marker.pose.orientation.w = 0.7071068

    return arrow_marker

def read_border_from_json(json_file):
    with open(json_file, 'r') as file:
        data = json.load(file)
        left_Border = data['left_Border']
        right_Border = data['right_Border']
        front_Border = data['front_Border']
        return left_Border, right_Border, front_Border

def draw_rectangle():
    geofence_marker_pub = rospy.Publisher('geofence_marker', Marker, queue_size=10)
    geofence_marker = create_marker("laser", Marker.LINE_STRIP, Vector3(0.1, 0.1, 0.1), ColorRGBA(0.0, 1.0, 0.0, 1.0))
    
    left_Border,right_Border,front_Border=read_border_from_json("border.json")

    p1=Point(x=left_Border, y=0, z=0.0)
    p2=Point(x=left_Border, y=front_Border, z=0.0)
    p3=Point(x=right_Border, y=front_Border, z=0.0)
    p4=Point(x=right_Border, y=0, z=0.0)


    # 添加矩形的四条边
    geofence_marker.points.extend([p1, p2, p3, p4, p1])  # 闭合矩形
    return geofence_marker


if __name__ == '__main__':
    rospy.init_node('geofence_lidar_visualizer', anonymous=True)
    rate = rospy.Rate(1)  # 发布频率为10Hz


    def shutdown_callback():
        # 停止发布消息
        lidar_marker_pub.unregister()
        arrow_marker_pub.unregister()
        geofence_marker_pub.unregister()


    rospy.on_shutdown(shutdown_callback)
    lidar_marker_pub = rospy.Publisher('lidar_marker', Marker, queue_size=10)
    arrow_marker_pub = rospy.Publisher('arrow_marker', Marker, queue_size=10)
    geofence_marker_pub = rospy.Publisher('geofence_marker', Marker, queue_size=10)

    try:
        while not rospy.is_shutdown():
            lidar_marker_pub.publish(draw_lidar())
            arrow_marker_pub.publish(draw_arrow())
            geofence_marker_pub.publish(draw_rectangle())
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
