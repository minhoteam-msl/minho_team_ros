#ifndef _ROS_SERVICE_requestResetIMU_h
#define _ROS_SERVICE_requestResetIMU_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace minho_team_ros
{

static const char REQUESTRESETIMU[] = "minho_team_ros/requestResetIMU";

  class requestResetIMURequest : public ros::Msg
  {
    public:
      bool dummy_var;

    requestResetIMURequest():
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

    const char * getType(){ return REQUESTRESETIMU; };
    const char * getMD5(){ return "d8c65f453dac023a8ef7ccee4f3a0dfc"; };

  };

  class requestResetIMUResponse : public ros::Msg
  {
    public:
      bool dummy_var;

    requestResetIMUResponse():
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

    const char * getType(){ return REQUESTRESETIMU; };
    const char * getMD5(){ return "d8c65f453dac023a8ef7ccee4f3a0dfc"; };

  };

  class requestResetIMU {
    public:
    typedef requestResetIMURequest Request;
    typedef requestResetIMUResponse Response;
  };

}
#endif
