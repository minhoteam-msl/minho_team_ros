#ifndef _ROS_minho_team_ros_teleop_h
#define _ROS_minho_team_ros_teleop_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace minho_team_ros
{

  class teleop : public ros::Msg
  {
    public:
      bool set_teleop;

    teleop():
      set_teleop(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_set_teleop;
      u_set_teleop.real = this->set_teleop;
      *(outbuffer + offset + 0) = (u_set_teleop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->set_teleop);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_set_teleop;
      u_set_teleop.base = 0;
      u_set_teleop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->set_teleop = u_set_teleop.real;
      offset += sizeof(this->set_teleop);
     return offset;
    }

    const char * getType(){ return "minho_team_ros/teleop"; };
    const char * getMD5(){ return "bc881cc517e374ee4ce02901069fabbf"; };

  };

}
#endif