// Generated by gencpp from file qr_robot_localization/LocationRequest.msg
// DO NOT EDIT!


#ifndef QR_ROBOT_LOCALIZATION_MESSAGE_LOCATIONREQUEST_H
#define QR_ROBOT_LOCALIZATION_MESSAGE_LOCATIONREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace qr_robot_localization
{
template <class ContainerAllocator>
struct LocationRequest_
{
  typedef LocationRequest_<ContainerAllocator> Type;

  LocationRequest_()
    {
    }
  LocationRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::qr_robot_localization::LocationRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::qr_robot_localization::LocationRequest_<ContainerAllocator> const> ConstPtr;

}; // struct LocationRequest_

typedef ::qr_robot_localization::LocationRequest_<std::allocator<void> > LocationRequest;

typedef boost::shared_ptr< ::qr_robot_localization::LocationRequest > LocationRequestPtr;
typedef boost::shared_ptr< ::qr_robot_localization::LocationRequest const> LocationRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::qr_robot_localization::LocationRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::qr_robot_localization::LocationRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace qr_robot_localization

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::qr_robot_localization::LocationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::qr_robot_localization::LocationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qr_robot_localization::LocationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qr_robot_localization::LocationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qr_robot_localization::LocationRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qr_robot_localization::LocationRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::qr_robot_localization::LocationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::qr_robot_localization::LocationRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::qr_robot_localization::LocationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "qr_robot_localization/LocationRequest";
  }

  static const char* value(const ::qr_robot_localization::LocationRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::qr_robot_localization::LocationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
";
  }

  static const char* value(const ::qr_robot_localization::LocationRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::qr_robot_localization::LocationRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LocationRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::qr_robot_localization::LocationRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::qr_robot_localization::LocationRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // QR_ROBOT_LOCALIZATION_MESSAGE_LOCATIONREQUEST_H
