#ifndef _ROS_SERVICE_requestKick_h
#define _ROS_SERVICE_requestKick_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace minho_team_ros
{

static const char REQUESTKICK[] = "minho_team_ros/requestKick";

  class requestKickRequest : public ros::Msg
  {
    public:
      uint8_t kick_strength;
      uint8_t kick_direction;
      bool kick_is_pass;

    requestKickRequest():
      kick_strength(0),
      kick_direction(0),
      kick_is_pass(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->kick_strength >> (8 * 0)) & 0xFF;
      offset += sizeof(this->kick_strength);
      *(outbuffer + offset + 0) = (this->kick_direction >> (8 * 0)) & 0xFF;
      offset += sizeof(this->kick_direction);
      union {
        bool real;
        uint8_t base;
      } u_kick_is_pass;
      u_kick_is_pass.real = this->kick_is_pass;
      *(outbuffer + offset + 0) = (u_kick_is_pass.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->kick_is_pass);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->kick_strength =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->kick_strength);
      this->kick_direction =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->kick_direction);
      union {
        bool real;
        uint8_t base;
      } u_kick_is_pass;
      u_kick_is_pass.base = 0;
      u_kick_is_pass.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->kick_is_pass = u_kick_is_pass.real;
      offset += sizeof(this->kick_is_pass);
     return offset;
    }

    const char * getType(){ return REQUESTKICK; };
    const char * getMD5(){ return "45d262cd0fa80dfe4763cc89da9cfc87"; };

  };

  class requestKickResponse : public ros::Msg
  {
    public:
      bool kicked;

    requestKickResponse():
      kicked(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_kicked;
      u_kicked.real = this->kicked;
      *(outbuffer + offset + 0) = (u_kicked.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->kicked);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_kicked;
      u_kicked.base = 0;
      u_kicked.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->kicked = u_kicked.real;
      offset += sizeof(this->kicked);
     return offset;
    }

    const char * getType(){ return REQUESTKICK; };
    const char * getMD5(){ return "a716d79ab51d1a2bef728eb98d7deddb"; };

  };

  class requestKick {
    public:
    typedef requestKickRequest Request;
    typedef requestKickResponse Response;
  };

}
#endif
