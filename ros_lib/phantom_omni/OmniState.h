#ifndef _ROS_phantom_omni_OmniState_h
#define _ROS_phantom_omni_OmniState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace phantom_omni
{

  class OmniState : public ros::Msg
  {
    public:
      float transform[16];
      typedef bool _button1_type;
      _button1_type button1;
      typedef bool _button2_type;
      _button2_type button2;

    OmniState():
      transform(),
      button1(0),
      button2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 16; i++){
      union {
        float real;
        uint32_t base;
      } u_transformi;
      u_transformi.real = this->transform[i];
      *(outbuffer + offset + 0) = (u_transformi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_transformi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_transformi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_transformi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->transform[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_button1;
      u_button1.real = this->button1;
      *(outbuffer + offset + 0) = (u_button1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->button1);
      union {
        bool real;
        uint8_t base;
      } u_button2;
      u_button2.real = this->button2;
      *(outbuffer + offset + 0) = (u_button2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->button2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 16; i++){
      union {
        float real;
        uint32_t base;
      } u_transformi;
      u_transformi.base = 0;
      u_transformi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_transformi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_transformi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_transformi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->transform[i] = u_transformi.real;
      offset += sizeof(this->transform[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_button1;
      u_button1.base = 0;
      u_button1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->button1 = u_button1.real;
      offset += sizeof(this->button1);
      union {
        bool real;
        uint8_t base;
      } u_button2;
      u_button2.base = 0;
      u_button2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->button2 = u_button2.real;
      offset += sizeof(this->button2);
     return offset;
    }

    const char * getType(){ return "phantom_omni/OmniState"; };
    const char * getMD5(){ return "47368dbb1bbc2b8aaa2e727f084de664"; };

  };

}
#endif