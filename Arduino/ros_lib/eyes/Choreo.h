#ifndef _ROS_eyes_Choreo_h
#define _ROS_eyes_Choreo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace eyes
{

  class Choreo : public ros::Msg
  {
    public:
      typedef bool _timed_type;
      _timed_type timed;
      typedef int32_t _duration_type;
      _duration_type duration;
      typedef bool _left_forward_type;
      _left_forward_type left_forward;
      typedef bool _right_forward_type;
      _right_forward_type right_forward;
      typedef int16_t _left_speed_type;
      _left_speed_type left_speed;
      typedef int16_t _right_speed_type;
      _right_speed_type right_speed;

    Choreo():
      timed(0),
      duration(0),
      left_forward(0),
      right_forward(0),
      left_speed(0),
      right_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_timed;
      u_timed.real = this->timed;
      *(outbuffer + offset + 0) = (u_timed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->timed);
      union {
        int32_t real;
        uint32_t base;
      } u_duration;
      u_duration.real = this->duration;
      *(outbuffer + offset + 0) = (u_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_duration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duration);
      union {
        bool real;
        uint8_t base;
      } u_left_forward;
      u_left_forward.real = this->left_forward;
      *(outbuffer + offset + 0) = (u_left_forward.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left_forward);
      union {
        bool real;
        uint8_t base;
      } u_right_forward;
      u_right_forward.real = this->right_forward;
      *(outbuffer + offset + 0) = (u_right_forward.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right_forward);
      union {
        int16_t real;
        uint16_t base;
      } u_left_speed;
      u_left_speed.real = this->left_speed;
      *(outbuffer + offset + 0) = (u_left_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_speed.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->left_speed);
      union {
        int16_t real;
        uint16_t base;
      } u_right_speed;
      u_right_speed.real = this->right_speed;
      *(outbuffer + offset + 0) = (u_right_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_speed.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->right_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_timed;
      u_timed.base = 0;
      u_timed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->timed = u_timed.real;
      offset += sizeof(this->timed);
      union {
        int32_t real;
        uint32_t base;
      } u_duration;
      u_duration.base = 0;
      u_duration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_duration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_duration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_duration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->duration = u_duration.real;
      offset += sizeof(this->duration);
      union {
        bool real;
        uint8_t base;
      } u_left_forward;
      u_left_forward.base = 0;
      u_left_forward.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->left_forward = u_left_forward.real;
      offset += sizeof(this->left_forward);
      union {
        bool real;
        uint8_t base;
      } u_right_forward;
      u_right_forward.base = 0;
      u_right_forward.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->right_forward = u_right_forward.real;
      offset += sizeof(this->right_forward);
      union {
        int16_t real;
        uint16_t base;
      } u_left_speed;
      u_left_speed.base = 0;
      u_left_speed.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_speed.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->left_speed = u_left_speed.real;
      offset += sizeof(this->left_speed);
      union {
        int16_t real;
        uint16_t base;
      } u_right_speed;
      u_right_speed.base = 0;
      u_right_speed.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_speed.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->right_speed = u_right_speed.real;
      offset += sizeof(this->right_speed);
     return offset;
    }

    const char * getType(){ return "eyes/Choreo"; };
    const char * getMD5(){ return "cc893b48a04f4c0dd26849bbdbb03ffa"; };

  };

}
#endif
