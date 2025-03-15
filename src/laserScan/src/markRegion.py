# -*- coding: utf-8 -*-

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
    return arrow_marker

def draw_rectangle():
    geofence_marker_pub = rospy.Publisher('geofence_marker', Marker, queue_size=10)
    geofence_marker = create_marker("laser", Marker.LINE_STRIP, Vector3(0.1, 0.1, 0.1), ColorRGBA(0.0, 1.0, 0.0, 1.0))

    # 定义矩形的四个顶点
    p1 = Point(x=6.0, y=3.0, z=0.0)
    p2 = Point(x=6.0, y=-4.5, z=0.0)
    p3 = Point(x=0.0, y=-4.5, z=0.0)
    p4 = Point(x=0.0, y=3.0, z=0.0)

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
