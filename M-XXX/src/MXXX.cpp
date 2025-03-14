/*
 * MXXX.cpp
 * Author: Zhigang Wu
 * Date: 2018-01-12
*/

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

#include "LIM/lim.h"
#include "MXXX/MXXX.h"
#include "console_bridge/console.h"

MXXX::MXXX()
: connected_(false)
, loaded_config_(false)
{
}

MXXX::~MXXX()
{
}

bool MXXX::connect(std::string host_ip, int port)
{
  if (!connected_)
  {
    std::cout <<"Creating non-blocking socket." << std::endl;
    socket_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (0 < socket_fd_)
    {
      struct sockaddr_in serverAddr;
      memset(&serverAddr, 0, sizeof(serverAddr));
      serverAddr.sin_family = AF_INET;
      serverAddr.sin_port = htons(port);
      inet_pton(AF_INET, host_ip.c_str(), &serverAddr.sin_addr);
      std::cout << "Connecting socket to laser."  << std::endl;
      int ret = ::connect(socket_fd_, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
      if (0 <= ret) 
      {
        connected_ = true;
        std::cout << "Connected succeeded." << std::endl;
        return true;
      }
      else // 0 > ret
      {
        return false;          
      }
    }
    else // 0 >= socket_fd
    {
      return false;
    }
  }
}

bool MXXX::initializedLaserConfig()
{
  return loaded_config_;
}

void MXXX::disconnect()
{
  if (connected_)
  {
    close(socket_fd_);
    connected_ = false;
  }
}

bool MXXX::isConnected()
{
  return connected_;
}

void MXXX::startMeas()
{
  int _cid = 1;
  LIM_HEAD * lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_START_LMD, NULL);

  write(socket_fd_, lim, lim->nLIMLen);

  LIM_Release(lim);
}

void MXXX::sendHB()
{
  int _cid = 1;
  LIM_HEAD * lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_HB, NULL);

  write(socket_fd_, lim, lim->nLIMLen);

  LIM_Release(lim);
}

bool MXXX::getConfig()
{
  int _cid = 1;
  LIM_HEAD *lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_GET_LDBCONFIG, NULL);

  write(socket_fd_, lim, lim->nLIMLen);
  LIM_Release(lim);
}

bool MXXX::LIM_CODE_LMD_Decoding(LIM_HEAD* lim, sensor_msgs::LaserScan *scan_data)
{
  if (lim->nCode != LIM_CODE_LMD)
    return false;
  LMD_INFO* lmd_info = LMD_Info(lim); 
  LMD_D_Type* lmd = LMD_D(lim);  

  scan_data->angle_increment = lmd_info->nAnglePrecision *M_PI / (1000.*180);
  scan_data->scan_time = 1.0 / float(lmd_info->nRPM / 60);
  scan_data->time_increment = scan_data->scan_time / (360000.0 / lmd_info->nAnglePrecision);
  scan_data->ranges.resize(lmd_info->nMDataNum);

  for(int i=0; i<lmd_info->nMDataNum; i++)
  {
    scan_data->ranges[i] = lmd[i]/100.;
	if(scan_data->ranges[i] > scan_data->range_max)
    {
      scan_data->ranges[i] = 0;
    }
  }
}

bool MXXX::LIM_CODE_LMD_HDRSSI_Decoding(LIM_HEAD* lim, sensor_msgs::LaserScan *scan_data)
{
  if (lim->nCode != LIM_CODE_LMD_HDRSSI)
  {
    return false;
  }
  LMD_INFO* lmd_info = LMD_Info(lim);
  LMD_HDRSSI_Type* lmdRSSI = LMD_D_HDRSSI(lim);

  scan_data->angle_increment = lmd_info->nAnglePrecision *M_PI / (1000.*180);
  scan_data->scan_time = 1.0 / float(lmd_info->nRPM / 60);
  scan_data->time_increment = scan_data->scan_time / (360000.0 / lmd_info->nAnglePrecision);
  scan_data->ranges.resize(lmd_info->nMDataNum);
  scan_data->intensities.resize(lmd_info->nMDataNum);

  for (int i = 0; i < lmd_info->nMDataNum; i++)
  {
    scan_data->ranges[i] = lmdRSSI[i].d_mm/1000.;
    scan_data->intensities[i] = lmdRSSI[i].rssi;

    if (scan_data->ranges[i] > scan_data->range_max || scan_data->intensities[i] > 1000)
    {
      scan_data->ranges[i] = 0;
      scan_data->intensities[i] = 0;
    }
  }
}

// LIM_CODE_LDBCONFIG LIM
bool MXXX::LIM_CODE_LDBCONFIG_Decoding(LIM_HEAD* lim, ULDINI_Type *uld)
{
  if (lim->nCode != LIM_CODE_LDBCONFIG)
  {
    std::cout << "Error: config packet decode failed.." << std::endl;
    return false;
  }

  *uld = *(ULDINI_Type*)LIM_ExData(lim);
  std::cout << "Config packet decoded...." << std::endl;
  return true;
}


STATUE_CONNECT MXXX::packetDecodeExt(ULDINI_Type *uld)
{
  int ret;
  static int cnt = 0;
  static bool bfile = true;
 
  while (1)
  {
    // Would be great to depend on linux's behaviour of updating the timeval, but unfortunately
      // that's non-POSIX (doesn't work on OS X, for example).
    fd_set rfds;    // file description set(fds)
    FD_ZERO(&rfds);   // empty fds
    FD_SET(socket_fd_, &rfds);  // add fd in fds

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    //(max file description value +1, read fds, write fds, error exit fds, time interval)
    int retval = select(socket_fd_ + 1, &rfds, NULL, NULL, &tv); // if receive fd return 1, else return 0

    if (0 < retval)
    {
      cnt = 0;
      int ret = buffer_.readFrom(socket_fd_);
      if(0 <= ret)
      {
        return isOK;
      }
      else if(RECV_STS_ERR == ret)
      {
        ROS_ERROR("error:socket read return:%d \r\nFILT:%s LINE:%d",ret,__FILE__,__LINE__);
        return isERROR;
      }
      else if(RECV_STS_TO == ret)
      {
        ROS_ERROR("error:socket recv return:%d \r\nFILT:%s LINE:%d",ret,__FILE__,__LINE__);
        return isERROR;
      }
    }
    else if(0 == retval)
    {
      if(MAX_SELECT_TIMEOUT_TIME < cnt++)
      {
        return isERROR;
      }
      ROS_INFO("[%02d-%03d]socket select TIMEOUT!",cnt,MAX_SELECT_TIMEOUT_TIME);
      return isTIMEOUT;
    }
    else if(0 > retval)
    {
        ROS_ERROR("error:socket select return:%d \r\nFILT:%s LINE:%d",ret,__FILE__,__LINE__);
        return isERROR;
    }
  }
}

bool MXXX::GetALim(ULDINI_Type *uld, sensor_msgs::LaserScan *scan_data)
{
  char* buffer_data = buffer_.getNextBuffer_LIM();

  if (buffer_data)
  {
    LIM_HEAD *lim = (LIM_HEAD*)buffer_data;
    unsigned int checksum = LIM_CheckSum(lim);
    if (checksum != lim->CheckSum)
    {
      printf("\tLIM checksum error!\n");
      return false;
    }

    if (LIM_CODE_LDBCONFIG == lim->nCode)
    {
      if(LIM_CODE_LDBCONFIG_Decoding(lim, uld))
      {
        loaded_config_ = true;
      }
    }

    if ((LIM_CODE_LMD == lim->nCode)&&(loaded_config_==true))
    {
      LIM_CODE_LMD_Decoding(lim, scan_data);
    }

    if ((LIM_CODE_LMD_HDRSSI==lim->nCode)&&(loaded_config_==true))
    {
      LIM_CODE_LMD_HDRSSI_Decoding(lim,scan_data);
    }

    buffer_.popLastBuffer();
    return true;
  }

  return false;
}

void MXXX::stopMeas()
{
  int _cid = 1;
  LIM_HEAD * lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_STOP_LMD, NULL);

  write(socket_fd_, lim, lim->nLIMLen);

  LIM_Release(lim);
}

/*
void MXXX::areaAlarm()
{
  int _cid = 1;
  LIM_HEAD * lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_FMSIG_QUERY,NULL);
  lim->Data[0] = 0;
  write(socket_fd_, lim, lim->nLIMLen);

  LIM_Release(lim);
}
*/
