// Generated by gencpp from file qr_robot_control/qr_robot_controlResponse.msg
// DO NOT EDIT!


#ifndef QR_ROBOT_CONTROL_MESSAGE_QR_ROBOT_CONTROLRESPONSE_H
#define QR_ROBOT_CONTROL_MESSAGE_QR_ROBOT_CONTROLRESPONSE_H


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
struct qr_robot_controlResponse_
{
  typedef qr_robot_controlResponse_<ContainerAllocator> Type;

  qr_robot_controlResponse_()
    : sum(0)  {
    }
  qr_robot_controlResponse_(const ContainerAllocator& _alloc)
    : sum(0)  {
  (void)_alloc;
    }



   typedef int64_t _sum_type;
  _sum_type sum;





  typedef boost::shared_ptr< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> const> ConstPtr;

}; // struct qr_robot_controlResponse_

typedef ::qr_robot_control::qr_robot_controlResponse_<std::allocator<void> > qr_robot_controlResponse;

typedef boost::shared_ptr< ::qr_robot_control::qr_robot_controlResponse > qr_robot_controlResponsePtr;
typedef boost::shared_ptr< ::qr_robot_control::qr_robot_controlResponse const> qr_robot_controlResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b88405221c77b1878a3cbbfff53428d7";
  }

  static const char* value(const ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb88405221c77b187ULL;
  static const uint64_t static_value2 = 0x8a3cbbfff53428d7ULL;
};

template<class ContainerAllocator>
struct DataType< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "qr_robot_control/qr_robot_controlResponse";
  }

  static const char* value(const ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 sum\n\
\n\
";
  }

  static const char* value(const ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sum);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct qr_robot_controlResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::qr_robot_control::qr_robot_controlResponse_<ContainerAllocator>& v)
  {
    s << indent << "sum: ";
    Printer<int64_t>::stream(s, indent + "  ", v.sum);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QR_ROBOT_CONTROL_MESSAGE_QR_ROBOT_CONTROLRESPONSE_H
