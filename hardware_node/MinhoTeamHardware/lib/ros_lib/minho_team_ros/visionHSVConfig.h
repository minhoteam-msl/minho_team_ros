#ifndef _ROS_minho_team_ros_visionHSVConfig_h
#define _ROS_minho_team_ros_visionHSVConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "minho_team_ros/label.h"

namespace minho_team_ros
{

  class visionHSVConfig : public ros::Msg
  {
    public:
      minho_team_ros::label field;
      minho_team_ros::label line;
      minho_team_ros::label ball;
      minho_team_ros::label obstacle;
      uint32_t other_labels_length;
      minho_team_ros::label st_other_labels;
      minho_team_ros::label * other_labels;

    visionHSVConfig():
      field(),
      line(),
      ball(),
      obstacle(),
      other_labels_length(0), other_labels(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->field.serialize(outbuffer + offset);
      offset += this->line.serialize(outbuffer + offset);
      offset += this->ball.serialize(outbuffer + offset);
      offset += this->obstacle.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->other_labels_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->other_labels_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->other_labels_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->other_labels_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->other_labels_length);
      for( uint32_t i = 0; i < other_labels_length; i++){
      offset += this->other_labels[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->field.deserialize(inbuffer + offset);
      offset += this->line.deserialize(inbuffer + offset);
      offset += this->ball.deserialize(inbuffer + offset);
      offset += this->obstacle.deserialize(inbuffer + offset);
      uint32_t other_labels_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      other_labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      other_labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      other_labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->other_labels_length);
      if(other_labels_lengthT > other_labels_length)
        this->other_labels = (minho_team_ros::label*)realloc(this->other_labels, other_labels_lengthT * sizeof(minho_team_ros::label));
      other_labels_length = other_labels_lengthT;
      for( uint32_t i = 0; i < other_labels_length; i++){
      offset += this->st_other_labels.deserialize(inbuffer + offset);
        memcpy( &(this->other_labels[i]), &(this->st_other_labels), sizeof(minho_team_ros::label));
      }
     return offset;
    }

    const char * getType(){ return "minho_team_ros/visionHSVConfig"; };
    const char * getMD5(){ return "7854b40a308443a08e5857dbcdd5aac1"; };

  };

}
#endif