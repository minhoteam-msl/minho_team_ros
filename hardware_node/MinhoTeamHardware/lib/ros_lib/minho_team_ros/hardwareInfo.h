#ifndef _ROS_minho_team_ros_hardwareInfo_h
#define _ROS_minho_team_ros_hardwareInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace minho_team_ros
{

  class hardwareInfo : public ros::Msg
  {
    public:
      int16_t imu_value;
      bool free_wheel_activated;
      uint8_t ball_sensor;
      int16_t encoder_1;
      int16_t encoder_2;
      int16_t encoder_3;
      float battery_pc;
      float battery_camera;
      float battery_main;

    hardwareInfo():
      imu_value(0),
      free_wheel_activated(0),
      ball_sensor(0),
      encoder_1(0),
      encoder_2(0),
      encoder_3(0),
      battery_pc(0),
      battery_camera(0),
      battery_main(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_imu_value;
      u_imu_value.real = this->imu_value;
      *(outbuffer + offset + 0) = (u_imu_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_imu_value.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->imu_value);
      union {
        bool real;
        uint8_t base;
      } u_free_wheel_activated;
      u_free_wheel_activated.real = this->free_wheel_activated;
      *(outbuffer + offset + 0) = (u_free_wheel_activated.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->free_wheel_activated);
      *(outbuffer + offset + 0) = (this->ball_sensor >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ball_sensor);
      union {
        int16_t real;
        uint16_t base;
      } u_encoder_1;
      u_encoder_1.real = this->encoder_1;
      *(outbuffer + offset + 0) = (u_encoder_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder_1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoder_1);
      union {
        int16_t real;
        uint16_t base;
      } u_encoder_2;
      u_encoder_2.real = this->encoder_2;
      *(outbuffer + offset + 0) = (u_encoder_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder_2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoder_2);
      union {
        int16_t real;
        uint16_t base;
      } u_encoder_3;
      u_encoder_3.real = this->encoder_3;
      *(outbuffer + offset + 0) = (u_encoder_3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder_3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoder_3);
      union {
        float real;
        uint32_t base;
      } u_battery_pc;
      u_battery_pc.real = this->battery_pc;
      *(outbuffer + offset + 0) = (u_battery_pc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_pc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_pc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_pc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_pc);
      union {
        float real;
        uint32_t base;
      } u_battery_camera;
      u_battery_camera.real = this->battery_camera;
      *(outbuffer + offset + 0) = (u_battery_camera.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_camera.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_camera.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_camera.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_camera);
      union {
        float real;
        uint32_t base;
      } u_battery_main;
      u_battery_main.real = this->battery_main;
      *(outbuffer + offset + 0) = (u_battery_main.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_main.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_main.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_main.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_main);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_imu_value;
      u_imu_value.base = 0;
      u_imu_value.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_imu_value.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->imu_value = u_imu_value.real;
      offset += sizeof(this->imu_value);
      union {
        bool real;
        uint8_t base;
      } u_free_wheel_activated;
      u_free_wheel_activated.base = 0;
      u_free_wheel_activated.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->free_wheel_activated = u_free_wheel_activated.real;
      offset += sizeof(this->free_wheel_activated);
      this->ball_sensor =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ball_sensor);
      union {
        int16_t real;
        uint16_t base;
      } u_encoder_1;
      u_encoder_1.base = 0;
      u_encoder_1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder_1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoder_1 = u_encoder_1.real;
      offset += sizeof(this->encoder_1);
      union {
        int16_t real;
        uint16_t base;
      } u_encoder_2;
      u_encoder_2.base = 0;
      u_encoder_2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder_2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoder_2 = u_encoder_2.real;
      offset += sizeof(this->encoder_2);
      union {
        int16_t real;
        uint16_t base;
      } u_encoder_3;
      u_encoder_3.base = 0;
      u_encoder_3.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder_3.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoder_3 = u_encoder_3.real;
      offset += sizeof(this->encoder_3);
      union {
        float real;
        uint32_t base;
      } u_battery_pc;
      u_battery_pc.base = 0;
      u_battery_pc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_pc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_pc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_pc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_pc = u_battery_pc.real;
      offset += sizeof(this->battery_pc);
      union {
        float real;
        uint32_t base;
      } u_battery_camera;
      u_battery_camera.base = 0;
      u_battery_camera.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_camera.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_camera.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_camera.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_camera = u_battery_camera.real;
      offset += sizeof(this->battery_camera);
      union {
        float real;
        uint32_t base;
      } u_battery_main;
      u_battery_main.base = 0;
      u_battery_main.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_main.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_main.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_main.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_main = u_battery_main.real;
      offset += sizeof(this->battery_main);
     return offset;
    }

    const char * getType(){ return "minho_team_ros/hardwareInfo"; };
    const char * getMD5(){ return "7ddd104ef92fbe37ae7db672b1b7876a"; };

  };

}
#endif