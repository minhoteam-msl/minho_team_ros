#ifndef _ROS_SERVICE_requestResetEncoders_h
#define _ROS_SERVICE_requestResetEncoders_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace minho_team_ros
{

static const char REQUESTRESETENCODERS[] = "minho_team_ros/requestResetEncoders";

  class requestResetEncodersRequest : public ros::Msg
  {
    public:
      bool dummy_var;

    requestResetEncodersRequest():
      dummy_var(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_dummy_var;
      u_dummy_var.real = this->dummy_var;
      *(outbuffer + offset + 0) = (u_dummy_var.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dummy_var);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_dummy_var;
      u_dummy_var.base = 0;
      u_dummy_var.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dummy_var = u_dummy_var.real;
      offset += sizeof(this->dummy_var);
     return offset;
    }

    const char * getType(){ return REQUESTRESETENCODERS; };
    const char * getMD5(){ return "d8c65f453dac023a8ef7ccee4f3a0dfc"; };

  };

  class requestResetEncodersResponse : public ros::Msg
  {
    public:
      bool dummy_var;

    requestResetEncodersResponse():
      dummy_var(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_dummy_var;
      u_dummy_var.real = this->dummy_var;
      *(outbuffer + offset + 0) = (u_dummy_var.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dummy_var);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_dummy_var;
      u_dummy_var.base = 0;
      u_dummy_var.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dummy_var = u_dummy_var.real;
      offset += sizeof(this->dummy_var);
     return offset;
    }

    const char * getType(){ return REQUESTRESETENCODERS; };
    const char * getMD5(){ return "d8c65f453dac023a8ef7ccee4f3a0dfc"; };

  };

  class requestResetEncoders {
    public:
    typedef requestResetEncodersRequest Request;
    typedef requestResetEncodersResponse Response;
  };

}
#endif
