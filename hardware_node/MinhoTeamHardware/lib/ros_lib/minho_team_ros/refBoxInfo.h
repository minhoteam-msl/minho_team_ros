#ifndef _ROS_minho_team_ros_refBoxInfo_h
#define _ROS_minho_team_ros_refBoxInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace minho_team_ros
{

  class refBoxInfo : public ros::Msg
  {
    public:
      uint8_t state;

    refBoxInfo():
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "minho_team_ros/refBoxInfo"; };
    const char * getMD5(){ return "800f34bc468def1d86e2d42bea5648c0"; };

  };

}
#endif