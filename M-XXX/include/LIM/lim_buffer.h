/*
 * LIM_buffer.h
*/

#ifndef LIM_BUFFER_H_
#define LIM_BUFFER_H_

#include "console_bridge/console.h"
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "LIM/lim.h"

#include <sys/socket.h>
#include <sys/select.h>

#include <ros/ros.h>

#define RECV_STS_TO   -2
#define RECV_STS_ERR  -1

#define LIM_BUFFER_SIZE 60000
#define LIM_TAG0 0xa5
#define LIM_TAG1 0x96
#define LIM_TAG2 0xec
#define LIM_TAG3 0xf5
#define MAX_READ0_TIMEOUT_TIME 50

class LIMBuffer
{
public:
  LIMBuffer() : total_length_(0), end_of_first_message_(0)
  {
  }
  
  int readFrom(int fd)
  {
    static int re0cnt = 0;

    int ret = recv(fd, buffer_ + total_length_, sizeof(buffer_) - total_length_,0);

    if (ret > 0)
    {
      re0cnt = 0;
      total_length_ += ret;
      ROS_DEBUG("Read %d bytes from fd, total length is %d.", ret, total_length_);
    }
    else if(0 == ret)
    {
       if(MAX_READ0_TIMEOUT_TIME < re0cnt++)
      {
        ROS_WARN("read TIMEOUT ret:%d-%d ",ret,RECV_STS_TO);
        return RECV_STS_TO;
      }     
    }
    else if(0 > ret)
    {
      ROS_ERROR("read error ret:%d ",ret);
      return RECV_STS_ERR;
    }
    return ret;
  }
  // Find Lim pack start.
  char* findLIMPktStart()
  {
    unsigned char *dbuffer_ = (unsigned char*)buffer_;
    for( std::size_t i=0; i<total_length_-4; i++)
    {
        if(dbuffer_[i] == LIM_TAG0
           && (dbuffer_[i+1]) == LIM_TAG1
           && (dbuffer_[i+2]) == LIM_TAG2
           && (dbuffer_[i+3]) == LIM_TAG3 )
        {
            return buffer_+i;
        }
    }
    return NULL;
  }
  
  // Find Lim pack end.
  char* findLIMPktEnd() 
  {
    if( total_length_< 40 )
        return NULL;
    
    LIM_HEAD *lim = (LIM_HEAD*)buffer_;
    if( total_length_ < lim->nLIMLen )
        return NULL;

    return buffer_ + lim->nLIMLen -1;
  }
  
  char* getNextBuffer_LIM()
  {
    if (total_length_ == 0)
    {
      // Buffer is empty, no scan data present.
      ROS_DEBUG("Empty buffer, nothing to return.");
      return NULL;
    }
  
    char* start_of_message = findLIMPktStart();
    if (start_of_message == NULL)
    {
      // Not foundt LIM TAG, reset buffer.
      ROS_WARN("No STX found, dropping %d bytes from buffer.", total_length_);
      total_length_ = 0;
    }
    else if (buffer_ != start_of_message)
    {
      // Shift buffer if start found.
      ROS_WARN("Shifting buffer, dropping %d bytes, %d bytes remain.",
              (start_of_message - buffer_), total_length_ - (start_of_message - buffer_));
      shiftBuffer(start_of_message);
    }

    // Find Lim pack end.
    end_of_first_message_ = findLIMPktEnd();
    if (end_of_first_message_ == NULL)
    {
      // Return null if no end.
      ROS_DEBUG("No ETX found, nothing to return.");
      return NULL;
    }

    return buffer_;
  }

 
  void popLastBuffer()
  {
    if (end_of_first_message_)
    {
      shiftBuffer(end_of_first_message_ + 1);
      end_of_first_message_ = NULL;
    }
  }

private:
  void shiftBuffer(char* new_start)
  {
    // Shift buffer.
    uint16_t remaining_length = total_length_ - (new_start - buffer_);

    if (remaining_length > 0)
    {
      memmove(buffer_, new_start, remaining_length);
    }
    total_length_ = remaining_length;
  }

  char buffer_[LIM_BUFFER_SIZE];
  uint16_t total_length_;

  char* end_of_first_message_;
};

#endif  // LIM_BUFFER_H_
