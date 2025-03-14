/*
 * MXXX.h
*/

#ifndef MXXX_H_
#define MXXX_H_

#include <LIM/lim_buffer.h>
#include <string>
#include <stdint.h>
#include "sensor_msgs/LaserScan.h"

#define MAX_SELECT_TIMEOUT_TIME 50

typedef enum {
  isOK = 0,
  isTIMEOUT,
  isNOCONNECT,
  isERROR
}STATUE_CONNECT;

class MXXX
{
public:
  MXXX();
  virtual ~MXXX();

  // connect laser at certain port
  bool connect(std::string host_ip, int port);

  // disconnect laser
  void disconnect();

  // load laser config
  bool initializedLaserConfig();

  // check laser connection
  bool isConnected();

  // start continous laser scan message
  void startMeas();

  // send laser Heart Beat
  void sendHB();

  // get laser config
  bool getConfig();

  // stop continous laser scan message
  void stopMeas();

  // tcp package decoding
  STATUE_CONNECT packetDecodeExt(ULDINI_Type *uld);

  // laser lim struct: message decoding
  bool GetALim(ULDINI_Type *uld, sensor_msgs::LaserScan *scan_data);
  
  // laser lim struct: LMD laser scan message decode
  bool LIM_CODE_LMD_Decoding(LIM_HEAD* lim, sensor_msgs::LaserScan *scan_data);

  // laser lim struct: LMD_HDRSSI laser scan message decode
  bool LIM_CODE_LMD_HDRSSI_Decoding(LIM_HEAD* lim, sensor_msgs::LaserScan *scan_data);

  // laser lim struct: LDB laser config decode
  bool LIM_CODE_LDBCONFIG_Decoding(LIM_HEAD* lim, ULDINI_Type *uld);
protected:
  bool connected_;
  bool loaded_config_;
  LIMBuffer buffer_;
  int socket_fd_;
};

#endif

