// Generated by gencpp from file robot_control/tactile.msg
// DO NOT EDIT!


#ifndef ROBOT_CONTROL_MESSAGE_TACTILE_H
#define ROBOT_CONTROL_MESSAGE_TACTILE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robot_control
{
template <class ContainerAllocator>
struct tactile_
{
  typedef tactile_<ContainerAllocator> Type;

  tactile_()
    : first(0.0)
    , second(0.0)
    , third(0.0)
    , fourth(0.0)
    , fifth(0.0)  {
    }
  tactile_(const ContainerAllocator& _alloc)
    : first(0.0)
    , second(0.0)
    , third(0.0)
    , fourth(0.0)
    , fifth(0.0)  {
  (void)_alloc;
    }



   typedef double _first_type;
  _first_type first;

   typedef double _second_type;
  _second_type second;

   typedef double _third_type;
  _third_type third;

   typedef double _fourth_type;
  _fourth_type fourth;

   typedef double _fifth_type;
  _fifth_type fifth;





  typedef boost::shared_ptr< ::robot_control::tactile_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_control::tactile_<ContainerAllocator> const> ConstPtr;

}; // struct tactile_

typedef ::robot_control::tactile_<std::allocator<void> > tactile;

typedef boost::shared_ptr< ::robot_control::tactile > tactilePtr;
typedef boost::shared_ptr< ::robot_control::tactile const> tactileConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_control::tactile_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_control::tactile_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robot_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'robot_control': ['/home/nektiria/rl_ws/src/robot_control/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robot_control::tactile_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_control::tactile_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_control::tactile_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_control::tactile_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_control::tactile_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_control::tactile_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_control::tactile_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b92696e452fb86182cb1f3c9a1668690";
  }

  static const char* value(const ::robot_control::tactile_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb92696e452fb8618ULL;
  static const uint64_t static_value2 = 0x2cb1f3c9a1668690ULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_control::tactile_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_control/tactile";
  }

  static const char* value(const ::robot_control::tactile_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_control::tactile_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 first\n\
float64 second\n\
float64 third\n\
float64 fourth\n\
float64 fifth\n\
";
  }

  static const char* value(const ::robot_control::tactile_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_control::tactile_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.first);
      stream.next(m.second);
      stream.next(m.third);
      stream.next(m.fourth);
      stream.next(m.fifth);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct tactile_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_control::tactile_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_control::tactile_<ContainerAllocator>& v)
  {
    s << indent << "first: ";
    Printer<double>::stream(s, indent + "  ", v.first);
    s << indent << "second: ";
    Printer<double>::stream(s, indent + "  ", v.second);
    s << indent << "third: ";
    Printer<double>::stream(s, indent + "  ", v.third);
    s << indent << "fourth: ";
    Printer<double>::stream(s, indent + "  ", v.fourth);
    s << indent << "fifth: ";
    Printer<double>::stream(s, indent + "  ", v.fifth);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_CONTROL_MESSAGE_TACTILE_H
