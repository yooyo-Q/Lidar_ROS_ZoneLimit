/*
 * MXXX ROS.cpp
 * Author: Zhigang Wu
 * Date: 2018-01-12
*/

#include <csignal>
#include <cstdio>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "MXXX/MXXX.h"
#include <cstdlib>
#include <std_srvs/Trigger.h>


class mxxx_ros_node
{

private:
  ros::NodeHandle node_;
  ros::NodeHandle private_nh_;
  // Subscriber
  //ros::Subscriber topic_;

  // Publisher
  ros::Publisher laser_scan_pub_;

  // Service
  //ros::ServiceServer connect_laser_service_;
  ros::ServiceServer disconnect_laser_service_;
  ros::ServiceServer start_laser_service_;
  ros::ServiceServer stop_laser_service_;

  // Params
  double scan_range_min_;
  double scan_range_max_;
  double angle_min_;
  double angle_max_;
  int port_;
  int scan_seq_;

  std::string host_ip_;
  std::string frame_id_;
  std::string scan_topic_;

  // Laser status
  bool connect_status_;
  bool get_config_;
  bool scanParamInitialized_;
  MXXX laser;

  // Laser config
  ULDINI_Type uld_config_;
  // Laser message
  sensor_msgs::LaserScan scan_msg_;

  ros::Time HB_time;

public:

  mxxx_ros_node();
  ~mxxx_ros_node();

  void initialize_scan_param();
  bool connect();
  bool getConfig();
  void startMesure();
  void sendHB2Lidar();
  void check_valid_scan_data();
  bool connect_laser(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool disconnect_laser(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool stop_scan_msg(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool start_scan_msg(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  void update();

  void runsh();
};

mxxx_ros_node::mxxx_ros_node()
: scan_seq_(0)
, connect_status_(false)
, get_config_(true)
, scanParamInitialized_(false)
, private_nh_("~")
{
  // parameters

  private_nh_.param<double>("scan_range_min", scan_range_min_, 0.0);
  private_nh_.param<double>("scan_range_max", scan_range_max_, 100);

  private_nh_.param<double>("angle_min", angle_min_, -2.356);
  private_nh_.param<double>("angle_max", angle_max_, 2.356);

  private_nh_.param<std::string>("frame", frame_id_, "laser");
  private_nh_.param<std::string>("scan_topic", scan_topic_, "scan");

  private_nh_.param<std::string>("host_ip", host_ip_, "192.168.1.201");
  private_nh_.param<int>("port", port_, 2112);

  // subscribers

  // publishers
  laser_scan_pub_ = node_.advertise<sensor_msgs::LaserScan>(scan_topic_, 1);

  // Services
  //connect_laser_service_ = node_.advertiseService("/mxxx/connect_laser_srv", &mxxx_ros_node::connect_laser, this);
  disconnect_laser_service_ = node_.advertiseService("/mxxx/disconnect_laser_srv", &mxxx_ros_node::disconnect_laser, this);
  start_laser_service_ = node_.advertiseService("/mxxx/start_laser_srv", &mxxx_ros_node::start_scan_msg, this);
  stop_laser_service_ = node_.advertiseService("/mxxx/stop_laser_srv", &mxxx_ros_node::stop_scan_msg, this);
  
}

mxxx_ros_node::~mxxx_ros_node()
{
    laser.stopMeas();
    laser.disconnect();
}

void mxxx_ros_node::runsh()
{
  ROS_WARN("ros shutdown.");
  ros::shutdown();
}

// load from laser config
void mxxx_ros_node::initialize_scan_param()
{
    scan_msg_.header.frame_id = frame_id_;
    scan_msg_.range_min = 0;
    scan_msg_.range_max = uld_config_.nMR/ 100.;
    scan_msg_.angle_min = (uld_config_.nSA[0]-90) /(180.)*M_PI;
    scan_msg_.angle_max = (uld_config_.nSA[1]-90) /(180.)*M_PI;

    ROS_INFO("scan_range_min: %f", scan_msg_.range_min);
    ROS_INFO("scan_range_max: %f", scan_msg_.range_max);
    ROS_INFO("scan_angle_min: %f", scan_msg_.angle_min);
    ROS_INFO("scan_angle_max: %f", scan_msg_.angle_max);
    ROS_INFO("lase RPM: %d", uld_config_.nSAV);
    ROS_INFO("lase nSAP: %d", uld_config_.nSAP);  
    ROS_INFO("lase nPF: %d", uld_config_.nPF);  

    scanParamInitialized_ = true;
}

// Connect laser.
bool mxxx_ros_node::connect()
{
    ROS_INFO_STREAM("Connecting to laser at " << host_ip_);
    laser.connect(host_ip_, port_);
    if (!laser.isConnected())
    {
      ROS_WARN("Unable to connect, retrying.");
      ros::Duration(1).sleep();
      return false;
    }

    connect_status_ = true;
    ROS_INFO("Connected to laser.");
    return true;
}

// Get Laser config.
bool mxxx_ros_node::getConfig()
{
  return laser.getConfig();
}

// Start continous laser scan msgs
void mxxx_ros_node::startMesure()
{
  laser.startMeas();
}

// Send Heart Beat to laser.
void mxxx_ros_node::sendHB2Lidar()
{
  laser.sendHB();
}

// service: Connect laser.
/*bool mxxx_ros_node::connect_laser(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
    resp.success = true;
    resp.message = "Disconnect Laser.";
    connect();
    return resp.success;
}*/

// service: Disconnect laser.
bool mxxx_ros_node::disconnect_laser(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
    resp.success = true;
    resp.message = "Disconnect Laser.";
    laser.disconnect();
    return resp.success;
}

// service: start laser.
bool mxxx_ros_node::start_scan_msg(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
    resp.success = true;
    resp.message = "Start receiving LaserScan msg.";
    startMesure();
    return resp.success;
}

// service: Stop laser.
bool mxxx_ros_node::stop_scan_msg(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
    resp.success = true;
    resp.message = "Stop receiving LaserScan msg.";
    laser.stopMeas();
    return resp.success;
}

void mxxx_ros_node::update()
{
  if(!connect_status_)
  {
    connect();
    ros::Duration(1).sleep();
  }
  else
  {
    // Send laser configuration request.
    if(get_config_)
    {
      getConfig();
      ROS_INFO("SEND: Request laser config ...");
      get_config_ = false;
    }

    // if receive packet from tcp
    int ret = laser.packetDecodeExt(&uld_config_);
    switch (ret) 
    {
      case isOK:
        {
        // decode laser packet.
          ros::Time scan_time = ros::Time::now();
          if(scan_time.toSec() - HB_time.toSec() > 5)
          {
            // Send Heart beat to laser every 5 sec.
            sendHB2Lidar();
            HB_time = scan_time; 
          }
          while(laser.GetALim(&uld_config_, &scan_msg_))
          {     
            if(laser.initializedLaserConfig())
            {

              // if not intialized laser config.
              if(!scanParamInitialized_)
              {
                initialize_scan_param();
                startMesure();
                HB_time = ros::Time::now();
              }
          
              scan_msg_.header.stamp = scan_time;
              ++scan_msg_.header.seq;
          
              // publish sensor_msgs/LaserScan
              laser_scan_pub_.publish(scan_msg_);         
            }
          }
          break;
        }
      case isTIMEOUT:
        break;

      case isNOCONNECT:
        break;

      case isERROR:
        ros::shutdown();
  
      default:
        break;
    }
  }
}

void delay_ms(int milliseconds) {
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mxxx_ros_node");
  mxxx_ros_node node;

  ROS_INFO("M-XXX laser driver node.");

  //ros::Rate rate(30);

  while(ros::ok())
  {
    ros::spinOnce();
    node.update();
    delay_ms(5);
  }
  return 0;
}


