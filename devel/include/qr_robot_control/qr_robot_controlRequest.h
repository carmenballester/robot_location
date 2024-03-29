// Generated by gencpp from file qr_robot_control/qr_robot_controlRequest.msg
// DO NOT EDIT!


#ifndef QR_ROBOT_CONTROL_MESSAGE_QR_ROBOT_CONTROLREQUEST_H
#define QR_ROBOT_CONTROL_MESSAGE_QR_ROBOT_CONTROLREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace qr_robot_control
{
template <class ContainerAllocator>
struct qr_robot_controlRequest_
{
  typedef qr_robot_controlRequest_<ContainerAllocator> Type;

  qr_robot_controlRequest_()
    : a(0)
    , b(0)
    , c(0)  {
    }
  qr_robot_controlRequest_(const ContainerAllocator& _alloc)
    : a(0)
    , b(0)
    , c(0)  {
  (void)_alloc;
    }



   typedef int64_t _a_type;
  _a_type a;

   typedef int64_t _b_type;
  _b_type b;

   typedef int64_t _c_type;
  _c_type c;





  typedef boost::shared_ptr< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> const> ConstPtr;

}; // struct qr_robot_controlRequest_

typedef ::qr_robot_control::qr_robot_controlRequest_<std::allocator<void> > qr_robot_controlRequest;

typedef boost::shared_ptr< ::qr_robot_control::qr_robot_controlRequest > qr_robot_controlRequestPtr;
typedef boost::shared_ptr< ::qr_robot_control::qr_robot_controlRequest const> qr_robot_controlRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace qr_robot_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c8559b52d432bccd240703f31aeca517";
  }

  static const char* value(const ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc8559b52d432bccdULL;
  static const uint64_t static_value2 = 0x240703f31aeca517ULL;
};

template<class ContainerAllocator>
struct DataType< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "qr_robot_control/qr_robot_controlRequest";
  }

  static const char* value(const ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 a\n\
int64 b\n\
int64 c\n\
";
  }

  static const char* value(const ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
      stream.next(m.b);
      stream.next(m.c);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct qr_robot_controlRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::qr_robot_control::qr_robot_controlRequest_<ContainerAllocator>& v)
  {
    s << indent << "a: ";
    Printer<int64_t>::stream(s, indent + "  ", v.a);
    s << indent << "b: ";
    Printer<int64_t>::stream(s, indent + "  ", v.b);
    s << indent << "c: ";
    Printer<int64_t>::stream(s, indent + "  ", v.c);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QR_ROBOT_CONTROL_MESSAGE_QR_ROBOT_CONTROLREQUEST_H
