// Generated by gencpp from file vision_unit/transformed_scan.msg
// DO NOT EDIT!


#ifndef VISION_UNIT_MESSAGE_TRANSFORMED_SCAN_H
#define VISION_UNIT_MESSAGE_TRANSFORMED_SCAN_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace vision_unit
{
template <class ContainerAllocator>
struct transformed_scan_
{
  typedef transformed_scan_<ContainerAllocator> Type;

  transformed_scan_()
    : dist()
    , angle()  {
    }
  transformed_scan_(const ContainerAllocator& _alloc)
    : dist(_alloc)
    , angle(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _dist_type;
  _dist_type dist;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _angle_type;
  _angle_type angle;





  typedef boost::shared_ptr< ::vision_unit::transformed_scan_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vision_unit::transformed_scan_<ContainerAllocator> const> ConstPtr;

}; // struct transformed_scan_

typedef ::vision_unit::transformed_scan_<std::allocator<void> > transformed_scan;

typedef boost::shared_ptr< ::vision_unit::transformed_scan > transformed_scanPtr;
typedef boost::shared_ptr< ::vision_unit::transformed_scan const> transformed_scanConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vision_unit::transformed_scan_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vision_unit::transformed_scan_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace vision_unit

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'move_base_msgs': ['/opt/ros/kinetic/share/move_base_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'vision_unit': ['/home/linux/autocar_ws/src/vision_unit/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::vision_unit::transformed_scan_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vision_unit::transformed_scan_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vision_unit::transformed_scan_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vision_unit::transformed_scan_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision_unit::transformed_scan_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision_unit::transformed_scan_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vision_unit::transformed_scan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d04f633fb6d830034a14d1a39c7655c7";
  }

  static const char* value(const ::vision_unit::transformed_scan_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd04f633fb6d83003ULL;
  static const uint64_t static_value2 = 0x4a14d1a39c7655c7ULL;
};

template<class ContainerAllocator>
struct DataType< ::vision_unit::transformed_scan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vision_unit/transformed_scan";
  }

  static const char* value(const ::vision_unit::transformed_scan_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vision_unit::transformed_scan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[] dist\n\
float32[] angle\n\
";
  }

  static const char* value(const ::vision_unit::transformed_scan_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vision_unit::transformed_scan_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.dist);
      stream.next(m.angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct transformed_scan_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vision_unit::transformed_scan_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vision_unit::transformed_scan_<ContainerAllocator>& v)
  {
    s << indent << "dist[]" << std::endl;
    for (size_t i = 0; i < v.dist.size(); ++i)
    {
      s << indent << "  dist[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.dist[i]);
    }
    s << indent << "angle[]" << std::endl;
    for (size_t i = 0; i < v.angle.size(); ++i)
    {
      s << indent << "  angle[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.angle[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISION_UNIT_MESSAGE_TRANSFORMED_SCAN_H