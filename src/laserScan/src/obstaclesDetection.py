#coding=utf-8
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header, ColorRGBA,String
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
import math
import rospy
import multiprocessing
from baseMark import create_text,create_spheres


def classify_elements(array):
    categories = {}  # 用字典存储分类结果
    current_category = None  # 当前分类的标记
    for element in array:
        # 根据条件判断当前元素属于哪个分类
        if current_category is None or element - categories[current_category][-1] > 1:
            # 创建新的分类
            current_category = len(categories)
            categories[current_category] = []
        # 将元素添加到当前分类中
        categories[current_category].append(element)
    return categories


def publish_text(text):
    # rospy.init_node('text_publisher')
    text_pub = rospy.Publisher('text_marker', Marker, queue_size=10)

    text_msg = Marker()
    text_msg.header.frame_id = "laser"
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
    text_pub.publish(text_msg)


def scan_callback(scan_msg):
    # 获取激光扫描的角度范围和角度增量
    angle_min = (-60 / 180) * math.pi
    angle_increment = scan_msg.angle_increment
    # 获取激光扫描的测量值列表
    ranges = scan_msg.ranges

    # 进行噪点过滤和插值
    filtered_ranges = []
    for i in range(len(ranges)):
        if ranges[i] > 0.1:
            filtered_ranges.append(ranges[i])
        else:
            filtered_ranges.append(100)  # 插值时，将点替换为100
    ranges = filtered_ranges

    # 创建点云数据
    filter_pc2_header = Header()
    filter_pc2_header.stamp = rospy.Time.now()
    filter_pc2_header.frame_id = scan_msg.header.frame_id
    filter_pc2_points = []

    # 创建点云数据
    obstacle_pc2_header = Header()
    obstacle_pc2_header.stamp = rospy.Time.now()
    obstacle_pc2_header.frame_id = scan_msg.header.frame_id
    obstacle_pc2_points = []

    obstacles_index = []
    pc2_elements = []

    # 遍历测量值列表，将每个极坐标转换为笛卡尔坐标
    for i, range_value in enumerate(ranges):
        # 计算当前测量点的角度
        angle = angle_min + i * angle_increment
        x = range_value * math.cos(angle)
        y = range_value * math.sin(angle)

        # 判断激光点是否在矩形(geofence)内
        if x >= 0 and x <= 6 and y > -4.5 and y <= 3:
            # 在矩形内检测到障碍物，将障碍物的点添加到点云数据中
            obstacle_pc2_points.append([x, y, 0.0])
            obstacles_index.append(i)
        else:
            filter_pc2_points.append([x, y, 0.0])

        pc2_elements.append([x, y, 0.0])

    # 对连续元素进行分类
    categories = classify_elements(obstacles_index)
    # print(categories)
    # 打印分类结果
    obstacles = []
    text = []
    vdistance_list=[]
    hdistance_list=[]
    obstacles_mean_point=[]

    for category, elements in categories.items():
        if (len(elements) > 3):
            median_index = int(round(sum(elements) / float(len(elements))))
            obstacles_mean_point.append(pc2_elements[median_index])
            vdistance = round(pc2_elements[median_index][0], 2)
            hdistance = round(pc2_elements[median_index][1], 2)
            obstacles_str = "NO.{}: Y:{}m,X{}m\n".format(category, vdistance, hdistance)
            obstacles.append(category)
            text.append(obstacles_str)

            vdistance_list.append(vdistance)
            hdistance_list.append(hdistance)

    print(obstacles_mean_point)

    if obstacles_mean_point:
        mean_point_sphere_pub.publish(create_spheres(obstacles_mean_point))



    if(vdistance_list):
        min_vdistance=min(vdistance_list)
        # print(min_vdistance)
        min_vdistance_str="closest obstacle : {}m\n".format(min_vdistance)

        to_host_pub.publish(str(min_vdistance))
        obstacle_Info = create_text("laser", min_vdistance_str)
        textMark2_pub.publish(obstacle_Info)


    # 创建obstacle点云消息
    obstacle_pc2_msg = point_cloud2.create_cloud_xyz32(obstacle_pc2_header, obstacle_pc2_points)
    filter_pc2_msg = point_cloud2.create_cloud_xyz32(filter_pc2_header, filter_pc2_points)
    obstacle_point_cloud_pub.publish(obstacle_pc2_msg)
    filter_point_cloud_pub.publish(filter_pc2_msg)

    # 创建进程对象
    Info = create_text("laser", "".join(text))
    process1 = multiprocessing.Process(target=textMark1_pub.publish(Info))
    process1.start()


if __name__ == '__main__':
    rospy.init_node('scan_to_cartesian')


    def shutdown_callback():
        # 停止发布消息
        obstacle_point_cloud_pub.unregister()
        filter_point_cloud_pub.unregister()
        textMark1_pub.unregister()
        textMark2_pub.unregister()
        mean_point_sphere_pub.unregister()
        to_host_pub.unregister()


    rospy.on_shutdown(shutdown_callback)
    textMark1_pub = rospy.Publisher('text_marker', Marker, queue_size=10)
    textMark2_pub = rospy.Publisher('text_marker2', Marker, queue_size=10)
    obstacle_point_cloud_pub = rospy.Publisher('obstacle_point_cloud_topic', PointCloud2, queue_size=10)
    filter_point_cloud_pub = rospy.Publisher('filter_point_cloud_topic', PointCloud2, queue_size=10)

    mean_point_sphere_pub = rospy.Publisher('mean_point_topic', Marker, queue_size=10)
    to_host_pub = rospy.Publisher('obstacle_message_topic', String, queue_size=10)  # 创建发布者，指定消息类型为String

    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()
