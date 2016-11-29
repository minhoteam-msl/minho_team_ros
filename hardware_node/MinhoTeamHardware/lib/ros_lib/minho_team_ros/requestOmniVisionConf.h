#ifndef _ROS_SERVICE_requestOmniVisionConf_h
#define _ROS_SERVICE_requestOmniVisionConf_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "minho_team_ros/imageConfig.h"
#include "minho_team_ros/mirrorConfig.h"
#include "minho_team_ros/visionHSVConfig.h"

namespace minho_team_ros
{

static const char REQUESTOMNIVISIONCONF[] = "minho_team_ros/requestOmniVisionConf";

  class requestOmniVisionConfRequest : public ros::Msg
  {
    public:
      const char* request_node_name;

    requestOmniVisionConfRequest():
      request_node_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_request_node_name = strlen(this->request_node_name);
      memcpy(outbuffer + offset, &length_request_node_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->request_node_name, length_request_node_name);
      offset += length_request_node_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_request_node_name;
      memcpy(&length_request_node_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_request_node_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_request_node_name-1]=0;
      this->request_node_name = (char *)(inbuffer + offset-1);
      offset += length_request_node_name;
     return offset;
    }

    const char * getType(){ return REQUESTOMNIVISIONCONF; };
    const char * getMD5(){ return "5b1b49211fc70bf533d687f4e30b7df9"; };

  };

  class requestOmniVisionConfResponse : public ros::Msg
  {
    public:
      minho_team_ros::mirrorConfig mirrorConf;
      minho_team_ros::visionHSVConfig visionConf;
      minho_team_ros::imageConfig imageConf;

    requestOmniVisionConfResponse():
      mirrorConf(),
      visionConf(),
      imageConf()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->mirrorConf.serialize(outbuffer + offset);
      offset += this->visionConf.serialize(outbuffer + offset);
      offset += this->imageConf.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->mirrorConf.deserialize(inbuffer + offset);
      offset += this->visionConf.deserialize(inbuffer + offset);
      offset += this->imageConf.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return REQUESTOMNIVISIONCONF; };
    const char * getMD5(){ return "a8f4cbd67be931555de72951f620a764"; };

  };

  class requestOmniVisionConf {
    public:
    typedef requestOmniVisionConfRequest Request;
    typedef requestOmniVisionConfResponse Response;
  };

}
#endif
