#ifndef _ROS_eyes_Generic_h
#define _ROS_eyes_Generic_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace eyes
{

  class Generic : public ros::Msg
  {
    public:
      typedef uint8_t _identifier_type;
      _identifier_type identifier;
      typedef bool _left_forward_type;
      _left_forward_type left_forward;
      typedef bool _right_forward_type;
      _right_forward_type right_forward;
      typedef uint8_t _left_speed_type;
      _left_speed_type left_speed;
      typedef uint8_t _right_speed_type;
      _right_speed_type right_speed;
      typedef bool _timed_type;
      _timed_type timed;
      typedef uint32_t _duration_type;
      _duration_type duration;

    Generic():
      identifier(0),
      left_forward(0),
      right_forward(0),
      left_speed(0),
      right_speed(0),
      timed(0),
      duration(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->identifier >> (8 * 0)) & 0xFF;
      offset += sizeof(this->identifier);
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
      *(outbuffer + offset + 0) = (this->left_speed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left_speed);
      *(outbuffer + offset + 0) = (this->right_speed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right_speed);
      union {
        bool real;
        uint8_t base;
      } u_timed;
      u_timed.real = this->timed;
      *(outbuffer + offset + 0) = (u_timed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->timed);
      *(outbuffer + offset + 0) = (this->duration >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->duration >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->duration >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->duration >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->identifier =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->identifier);
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
      this->left_speed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->left_speed);
      this->right_speed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->right_speed);
      union {
        bool real;
        uint8_t base;
      } u_timed;
      u_timed.base = 0;
      u_timed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->timed = u_timed.real;
      offset += sizeof(this->timed);
      this->duration =  ((uint32_t) (*(inbuffer + offset)));
      this->duration |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->duration |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->duration |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->duration);
     return offset;
    }

    const char * getType(){ return "eyes/Generic"; };
    const char * getMD5(){ return "7a33c669a022f7fea29ccba33d517b1f"; };

  };

}
#endif
