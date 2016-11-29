#ifndef _ROS_minho_team_ros_imuConfig_h
#define _ROS_minho_team_ros_imuConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace minho_team_ros
{

  class imuConfig : public ros::Msg
  {
    public:
      uint8_t step;
      uint32_t imu_values_length;
      uint16_t st_imu_values;
      uint16_t * imu_values;

    imuConfig():
      step(0),
      imu_values_length(0), imu_values(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->step >> (8 * 0)) & 0xFF;
      offset += sizeof(this->step);
      *(outbuffer + offset + 0) = (this->imu_values_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->imu_values_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->imu_values_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->imu_values_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->imu_values_length);
      for( uint32_t i = 0; i < imu_values_length; i++){
      *(outbuffer + offset + 0) = (this->imu_values[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->imu_values[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->imu_values[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->step =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->step);
      uint32_t imu_values_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      imu_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      imu_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      imu_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->imu_values_length);
      if(imu_values_lengthT > imu_values_length)
        this->imu_values = (uint16_t*)realloc(this->imu_values, imu_values_lengthT * sizeof(uint16_t));
      imu_values_length = imu_values_lengthT;
      for( uint32_t i = 0; i < imu_values_length; i++){
      this->st_imu_values =  ((uint16_t) (*(inbuffer + offset)));
      this->st_imu_values |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_imu_values);
        memcpy( &(this->imu_values[i]), &(this->st_imu_values), sizeof(uint16_t));
      }
     return offset;
    }

    const char * getType(){ return "minho_team_ros/imuConfig"; };
    const char * getMD5(){ return "99267622e2c033bc950d0df6e2012da9"; };

  };

}
#endif