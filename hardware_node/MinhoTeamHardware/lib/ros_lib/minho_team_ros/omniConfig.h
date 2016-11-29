#ifndef _ROS_minho_team_ros_omniConfig_h
#define _ROS_minho_team_ros_omniConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace minho_team_ros
{

  class omniConfig : public ros::Msg
  {
    public:
      uint16_t P;
      uint16_t I;
      uint16_t D;
      uint16_t B;
      uint16_t N;
      uint16_t M;

    omniConfig():
      P(0),
      I(0),
      D(0),
      B(0),
      N(0),
      M(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->P >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->P >> (8 * 1)) & 0xFF;
      offset += sizeof(this->P);
      *(outbuffer + offset + 0) = (this->I >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->I >> (8 * 1)) & 0xFF;
      offset += sizeof(this->I);
      *(outbuffer + offset + 0) = (this->D >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->D >> (8 * 1)) & 0xFF;
      offset += sizeof(this->D);
      *(outbuffer + offset + 0) = (this->B >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->B >> (8 * 1)) & 0xFF;
      offset += sizeof(this->B);
      *(outbuffer + offset + 0) = (this->N >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->N >> (8 * 1)) & 0xFF;
      offset += sizeof(this->N);
      *(outbuffer + offset + 0) = (this->M >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->M >> (8 * 1)) & 0xFF;
      offset += sizeof(this->M);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->P =  ((uint16_t) (*(inbuffer + offset)));
      this->P |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->P);
      this->I =  ((uint16_t) (*(inbuffer + offset)));
      this->I |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->I);
      this->D =  ((uint16_t) (*(inbuffer + offset)));
      this->D |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->D);
      this->B =  ((uint16_t) (*(inbuffer + offset)));
      this->B |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->B);
      this->N =  ((uint16_t) (*(inbuffer + offset)));
      this->N |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->N);
      this->M =  ((uint16_t) (*(inbuffer + offset)));
      this->M |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->M);
     return offset;
    }

    const char * getType(){ return "minho_team_ros/omniConfig"; };
    const char * getMD5(){ return "65e2f433e6ddf3820fa6cab01cfd20e1"; };

  };

}
#endif