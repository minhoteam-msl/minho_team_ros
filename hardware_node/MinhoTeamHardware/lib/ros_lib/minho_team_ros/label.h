#ifndef _ROS_minho_team_ros_label_h
#define _ROS_minho_team_ros_label_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "minho_team_ros/range.h"

namespace minho_team_ros
{

  class label : public ros::Msg
  {
    public:
      minho_team_ros::range H;
      minho_team_ros::range S;
      minho_team_ros::range V;
      uint8_t classifier;

    label():
      H(),
      S(),
      V(),
      classifier(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->H.serialize(outbuffer + offset);
      offset += this->S.serialize(outbuffer + offset);
      offset += this->V.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->classifier >> (8 * 0)) & 0xFF;
      offset += sizeof(this->classifier);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->H.deserialize(inbuffer + offset);
      offset += this->S.deserialize(inbuffer + offset);
      offset += this->V.deserialize(inbuffer + offset);
      this->classifier =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->classifier);
     return offset;
    }

    const char * getType(){ return "minho_team_ros/label"; };
    const char * getMD5(){ return "f8d11e25bb27658154ba79be8012bd83"; };

  };

}
#endif