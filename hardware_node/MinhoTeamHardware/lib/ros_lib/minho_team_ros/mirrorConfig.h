#ifndef _ROS_minho_team_ros_mirrorConfig_h
#define _ROS_minho_team_ros_mirrorConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace minho_team_ros
{

  class mirrorConfig : public ros::Msg
  {
    public:
      float max_distance;
      float step;
      uint32_t pixel_distances_length;
      uint16_t st_pixel_distances;
      uint16_t * pixel_distances;

    mirrorConfig():
      max_distance(0),
      step(0),
      pixel_distances_length(0), pixel_distances(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_max_distance;
      u_max_distance.real = this->max_distance;
      *(outbuffer + offset + 0) = (u_max_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_distance);
      union {
        float real;
        uint32_t base;
      } u_step;
      u_step.real = this->step;
      *(outbuffer + offset + 0) = (u_step.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_step.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_step.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_step.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->step);
      *(outbuffer + offset + 0) = (this->pixel_distances_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pixel_distances_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pixel_distances_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pixel_distances_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pixel_distances_length);
      for( uint32_t i = 0; i < pixel_distances_length; i++){
      *(outbuffer + offset + 0) = (this->pixel_distances[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pixel_distances[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pixel_distances[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_max_distance;
      u_max_distance.base = 0;
      u_max_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_distance = u_max_distance.real;
      offset += sizeof(this->max_distance);
      union {
        float real;
        uint32_t base;
      } u_step;
      u_step.base = 0;
      u_step.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_step.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_step.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_step.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->step = u_step.real;
      offset += sizeof(this->step);
      uint32_t pixel_distances_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pixel_distances_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pixel_distances_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pixel_distances_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pixel_distances_length);
      if(pixel_distances_lengthT > pixel_distances_length)
        this->pixel_distances = (uint16_t*)realloc(this->pixel_distances, pixel_distances_lengthT * sizeof(uint16_t));
      pixel_distances_length = pixel_distances_lengthT;
      for( uint32_t i = 0; i < pixel_distances_length; i++){
      this->st_pixel_distances =  ((uint16_t) (*(inbuffer + offset)));
      this->st_pixel_distances |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_pixel_distances);
        memcpy( &(this->pixel_distances[i]), &(this->st_pixel_distances), sizeof(uint16_t));
      }
     return offset;
    }

    const char * getType(){ return "minho_team_ros/mirrorConfig"; };
    const char * getMD5(){ return "40d195ed19d0d59ed71c7c6e402471a8"; };

  };

}
#endif