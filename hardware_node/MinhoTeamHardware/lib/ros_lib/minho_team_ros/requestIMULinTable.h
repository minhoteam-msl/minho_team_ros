#ifndef _ROS_SERVICE_requestIMULinTable_h
#define _ROS_SERVICE_requestIMULinTable_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "minho_team_ros/imuConfig.h"

namespace minho_team_ros
{

static const char REQUESTIMULINTABLE[] = "minho_team_ros/requestIMULinTable";

  class requestIMULinTableRequest : public ros::Msg
  {
    public:
      minho_team_ros::imuConfig imuConf;
      bool is_set;

    requestIMULinTableRequest():
      imuConf(),
      is_set(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->imuConf.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_is_set;
      u_is_set.real = this->is_set;
      *(outbuffer + offset + 0) = (u_is_set.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_set);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->imuConf.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_is_set;
      u_is_set.base = 0;
      u_is_set.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_set = u_is_set.real;
      offset += sizeof(this->is_set);
     return offset;
    }

    const char * getType(){ return REQUESTIMULINTABLE; };
    const char * getMD5(){ return "cd39589adbb3cfcaaae8ebd2332b8319"; };

  };

  class requestIMULinTableResponse : public ros::Msg
  {
    public:
      minho_team_ros::imuConfig imuConf;

    requestIMULinTableResponse():
      imuConf()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->imuConf.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->imuConf.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return REQUESTIMULINTABLE; };
    const char * getMD5(){ return "9e6482de230fd27484d1771670a9c37a"; };

  };

  class requestIMULinTable {
    public:
    typedef requestIMULinTableRequest Request;
    typedef requestIMULinTableResponse Response;
  };

}
#endif
