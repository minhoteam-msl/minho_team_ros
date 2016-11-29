#ifndef _ROS_minho_team_ros_imageConfig_h
#define _ROS_minho_team_ros_imageConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace minho_team_ros
{

  class imageConfig : public ros::Msg
  {
    public:
      uint16_t center_x;
      uint16_t center_y;
      uint16_t tilt;

    imageConfig():
      center_x(0),
      center_y(0),
      tilt(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->center_x >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->center_x >> (8 * 1)) & 0xFF;
      offset += sizeof(this->center_x);
      *(outbuffer + offset + 0) = (this->center_y >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->center_y >> (8 * 1)) & 0xFF;
      offset += sizeof(this->center_y);
      *(outbuffer + offset + 0) = (this->tilt >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tilt >> (8 * 1)) & 0xFF;
      offset += sizeof(this->tilt);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->center_x =  ((uint16_t) (*(inbuffer + offset)));
      this->center_x |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->center_x);
      this->center_y =  ((uint16_t) (*(inbuffer + offset)));
      this->center_y |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->center_y);
      this->tilt =  ((uint16_t) (*(inbuffer + offset)));
      this->tilt |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->tilt);
     return offset;
    }

    const char * getType(){ return "minho_team_ros/imageConfig"; };
    const char * getMD5(){ return "4be8e7dcbe61cab7b88ec929e0e5cfee"; };

  };

}
#endif