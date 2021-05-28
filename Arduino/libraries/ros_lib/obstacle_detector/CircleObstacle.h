#ifndef _ROS_obstacle_detector_CircleObstacle_h
#define _ROS_obstacle_detector_CircleObstacle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

namespace obstacle_detector
{

  class CircleObstacle : public ros::Msg
  {
    public:
      typedef geometry_msgs::Point _center_type;
      _center_type center;
      typedef geometry_msgs::Vector3 _velocity_type;
      _velocity_type velocity;
      typedef float _radius_type;
      _radius_type radius;
      typedef float _true_radius_type;
      _true_radius_type true_radius;

    CircleObstacle():
      center(),
      velocity(),
      radius(0),
      true_radius(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->center.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->radius);
      offset += serializeAvrFloat64(outbuffer + offset, this->true_radius);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->center.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->radius));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->true_radius));
     return offset;
    }

    const char * getType(){ return "obstacle_detector/CircleObstacle"; };
    const char * getMD5(){ return "d23cb7e768ed09971078d4cccc3808a9"; };

  };

}
#endif
