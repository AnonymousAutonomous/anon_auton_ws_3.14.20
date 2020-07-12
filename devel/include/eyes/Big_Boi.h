// Generated by gencpp from file eyes/Big_Boi.msg
// DO NOT EDIT!


#ifndef EYES_MESSAGE_BIG_BOI_H
#define EYES_MESSAGE_BIG_BOI_H


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
struct Big_Boi_
{
  typedef Big_Boi_<ContainerAllocator> Type;

  Big_Boi_()
    : timmy(false)
    , tommy(false)
    , tammy(false)  {
    }
  Big_Boi_(const ContainerAllocator& _alloc)
    : timmy(false)
    , tommy(false)
    , tammy(false)  {
  (void)_alloc;
    }



   typedef uint8_t _timmy_type;
  _timmy_type timmy;

   typedef uint8_t _tommy_type;
  _tommy_type tommy;

   typedef uint8_t _tammy_type;
  _tammy_type tammy;





  typedef boost::shared_ptr< ::eyes::Big_Boi_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::eyes::Big_Boi_<ContainerAllocator> const> ConstPtr;

}; // struct Big_Boi_

typedef ::eyes::Big_Boi_<std::allocator<void> > Big_Boi;

typedef boost::shared_ptr< ::eyes::Big_Boi > Big_BoiPtr;
typedef boost::shared_ptr< ::eyes::Big_Boi const> Big_BoiConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::eyes::Big_Boi_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::eyes::Big_Boi_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::eyes::Big_Boi_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::eyes::Big_Boi_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::eyes::Big_Boi_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::eyes::Big_Boi_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::eyes::Big_Boi_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::eyes::Big_Boi_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::eyes::Big_Boi_<ContainerAllocator> >
{
  static const char* value()
  {
    return "14735f0ab9bd89e9e2ae5c4a9db3ab4c";
  }

  static const char* value(const ::eyes::Big_Boi_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x14735f0ab9bd89e9ULL;
  static const uint64_t static_value2 = 0xe2ae5c4a9db3ab4cULL;
};

template<class ContainerAllocator>
struct DataType< ::eyes::Big_Boi_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eyes/Big_Boi";
  }

  static const char* value(const ::eyes::Big_Boi_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::eyes::Big_Boi_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool timmy\n"
"bool tommy\n"
"bool tammy\n"
;
  }

  static const char* value(const ::eyes::Big_Boi_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::eyes::Big_Boi_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timmy);
      stream.next(m.tommy);
      stream.next(m.tammy);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Big_Boi_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::eyes::Big_Boi_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::eyes::Big_Boi_<ContainerAllocator>& v)
  {
    s << indent << "timmy: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.timmy);
    s << indent << "tommy: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.tommy);
    s << indent << "tammy: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.tammy);
  }
};

} // namespace message_operations
} // namespace ros

#endif // EYES_MESSAGE_BIG_BOI_H
