#ifndef _ROS_minho_team_ros_imgRequest_h
#define _ROS_minho_team_ros_imgRequest_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace minho_team_ros
{

  class imgRequest : public ros::Msg
  {
    public:
      bool is_multiple;
      uint8_t frequency;
      uint8_t type;

    imgRequest():
      is_multiple(0),
      frequency(0),
      type(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_multiple;
      u_is_multiple.real = this->is_multiple;
      *(outbuffer + offset + 0) = (u_is_multiple.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_multiple);
      *(outbuffer + offset + 0) = (this->frequency >> (8 * 0)) & 0xFF;
      offset += sizeof(this->frequency);
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_multiple;
      u_is_multiple.base = 0;
      u_is_multiple.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_multiple = u_is_multiple.real;
      offset += sizeof(this->is_multiple);
      this->frequency =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->frequency);
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
     return offset;
    }

    const char * getType(){ return "minho_team_ros/imgRequest"; };
    const char * getMD5(){ return "c6caee47f7970edb7512b46856a943d3"; };

  };

}
#endif