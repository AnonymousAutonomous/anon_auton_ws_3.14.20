// Generated by gencpp from file eyes/Choreo.msg
// DO NOT EDIT!


#ifndef EYES_MESSAGE_CHOREO_H
#define EYES_MESSAGE_CHOREO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace eyes
{
template <class ContainerAllocator>
struct Choreo_
{
  typedef Choreo_<ContainerAllocator> Type;

  Choreo_()
    : timed(false)
    , duration(0)
    , left_forward(false)
    , right_forward(false)
    , left_speed(0)
    , right_speed(0)  {
    }
  Choreo_(const ContainerAllocator& _alloc)
    : timed(false)
    , duration(0)
    , left_forward(false)
    , right_forward(false)
    , left_speed(0)
    , right_speed(0)  {
  (void)_alloc;
    }



   typedef uint8_t _timed_type;
  _timed_type timed;

   typedef int32_t _duration_type;
  _duration_type duration;

   typedef uint8_t _left_forward_type;
  _left_forward_type left_forward;

   typedef uint8_t _right_forward_type;
  _right_forward_type right_forward;

   typedef int16_t _left_speed_type;
  _left_speed_type left_speed;

   typedef int16_t _right_speed_type;
  _right_speed_type right_speed;





  typedef boost::shared_ptr< ::eyes::Choreo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::eyes::Choreo_<ContainerAllocator> const> ConstPtr;

}; // struct Choreo_

typedef ::eyes::Choreo_<std::allocator<void> > Choreo;

typedef boost::shared_ptr< ::eyes::Choreo > ChoreoPtr;
typedef boost::shared_ptr< ::eyes::Choreo const> ChoreoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::eyes::Choreo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::eyes::Choreo_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace eyes

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'eyes': ['/home/anonymous3/anon_auton_ws/src/eyes/msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::eyes::Choreo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::eyes::Choreo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::eyes::Choreo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::eyes::Choreo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::eyes::Choreo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::eyes::Choreo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::eyes::Choreo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cc893b48a04f4c0dd26849bbdbb03ffa";
  }

  static const char* value(const ::eyes::Choreo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcc893b48a04f4c0dULL;
  static const uint64_t static_value2 = 0xd26849bbdbb03ffaULL;
};

template<class ContainerAllocator>
struct DataType< ::eyes::Choreo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eyes/Choreo";
  }

  static const char* value(const ::eyes::Choreo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::eyes::Choreo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool timed\n"
"int32 duration\n"
"bool left_forward\n"
"bool right_forward\n"
"int16 left_speed\n"
"int16 right_speed\n"
;
  }

  static const char* value(const ::eyes::Choreo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::eyes::Choreo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timed);
      stream.next(m.duration);
      stream.next(m.left_forward);
      stream.next(m.right_forward);
      stream.next(m.left_speed);
      stream.next(m.right_speed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Choreo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::eyes::Choreo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::eyes::Choreo_<ContainerAllocator>& v)
  {
    s << indent << "timed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.timed);
    s << indent << "duration: ";
    Printer<int32_t>::stream(s, indent + "  ", v.duration);
    s << indent << "left_forward: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.left_forward);
    s << indent << "right_forward: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.right_forward);
    s << indent << "left_speed: ";
    Printer<int16_t>::stream(s, indent + "  ", v.left_speed);
    s << indent << "right_speed: ";
    Printer<int16_t>::stream(s, indent + "  ", v.right_speed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // EYES_MESSAGE_CHOREO_H