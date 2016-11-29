#ifndef _ROS_SERVICE_requestOmniProps_h
#define _ROS_SERVICE_requestOmniProps_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "minho_team_ros/omniConfig.h"

namespace minho_team_ros
{

static const char REQUESTOMNIPROPS[] = "minho_team_ros/requestOmniProps";

  class requestOmniPropsRequest : public ros::Msg
  {
    public:
      minho_team_ros::omniConfig omniConf;
      bool is_set;

    requestOmniPropsRequest():
      omniConf(),
      is_set(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->omniConf.serialize(outbuffer + offset);
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
      offset += this->omniConf.deserialize(inbuffer + offset);
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

    const char * getType(){ return REQUESTOMNIPROPS; };
    const char * getMD5(){ return "36e4272eae35aed58f4c6eff93940225"; };

  };

  class requestOmniPropsResponse : public ros::Msg
  {
    public:
      minho_team_ros::omniConfig omniConf;

    requestOmniPropsResponse():
      omniConf()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->omniConf.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->omniConf.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return REQUESTOMNIPROPS; };
    const char * getMD5(){ return "294d2a8d409cb2af8b414e22d13dfaf6"; };

  };

  class requestOmniProps {
    public:
    typedef requestOmniPropsRequest Request;
    typedef requestOmniPropsResponse Response;
  };

}
#endif
