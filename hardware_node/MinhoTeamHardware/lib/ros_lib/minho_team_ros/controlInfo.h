#ifndef _ROS_minho_team_ros_controlInfo_h
#define _ROS_minho_team_ros_controlInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace minho_team_ros
{

  class controlInfo : public ros::Msg
  {
    public:
      uint8_t linear_velocity;
      int8_t angular_velocity;
      uint16_t movement_direction;
      bool dribbler_on;
      bool is_teleop;

    controlInfo():
      linear_velocity(0),
      angular_velocity(0),
      movement_direction(0),
      dribbler_on(0),
      is_teleop(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->linear_velocity >> (8 * 0)) & 0xFF;
      offset += sizeof(this->linear_velocity);
      union {
        int8_t real;
        uint8_t base;
      } u_angular_velocity;
      u_angular_velocity.real = this->angular_velocity;
      *(outbuffer + offset + 0) = (u_angular_velocity.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angular_velocity);
      *(outbuffer + offset + 0) = (this->movement_direction >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->movement_direction >> (8 * 1)) & 0xFF;
      offset += sizeof(this->movement_direction);
      union {
        bool real;
        uint8_t base;
      } u_dribbler_on;
      u_dribbler_on.real = this->dribbler_on;
      *(outbuffer + offset + 0) = (u_dribbler_on.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dribbler_on);
      union {
        bool real;
        uint8_t base;
      } u_is_teleop;
      u_is_teleop.real = this->is_teleop;
      *(outbuffer + offset + 0) = (u_is_teleop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_teleop);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->linear_velocity =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->linear_velocity);
      union {
        int8_t real;
        uint8_t base;
      } u_angular_velocity;
      u_angular_velocity.base = 0;
      u_angular_velocity.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->angular_velocity = u_angular_velocity.real;
      offset += sizeof(this->angular_velocity);
      this->movement_direction =  ((uint16_t) (*(inbuffer + offset)));
      this->movement_direction |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->movement_direction);
      union {
        bool real;
        uint8_t base;
      } u_dribbler_on;
      u_dribbler_on.base = 0;
      u_dribbler_on.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dribbler_on = u_dribbler_on.real;
      offset += sizeof(this->dribbler_on);
      union {
        bool real;
        uint8_t base;
      } u_is_teleop;
      u_is_teleop.base = 0;
      u_is_teleop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_teleop = u_is_teleop.real;
      offset += sizeof(this->is_teleop);
     return offset;
    }

    const char * getType(){ return "minho_team_ros/controlInfo"; };
    const char * getMD5(){ return "38bbfed00b67b1a671e86724bc746854"; };

  };

}
#endif