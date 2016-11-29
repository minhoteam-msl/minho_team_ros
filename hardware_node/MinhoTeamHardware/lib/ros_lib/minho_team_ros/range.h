#ifndef _ROS_minho_team_ros_range_h
#define _ROS_minho_team_ros_range_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace minho_team_ros
{

  class range : public ros::Msg
  {
    public:
      uint8_t min;
      uint8_t max;

    range():
      min(0),
      max(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->min >> (8 * 0)) & 0xFF;
      offset += sizeof(this->min);
      *(outbuffer + offset + 0) = (this->max >> (8 * 0)) & 0xFF;
      offset += sizeof(this->max);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->min =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->min);
      this->max =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->max);
     return offset;
    }

    const char * getType(){ return "minho_team_ros/range"; };
    const char * getMD5(){ return "fbb30d0003352cb8e4e931cefd163d1a"; };

  };

}
#endif