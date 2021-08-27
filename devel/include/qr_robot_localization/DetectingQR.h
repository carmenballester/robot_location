// Generated by gencpp from file qr_robot_localization/DetectingQR.msg
// DO NOT EDIT!


#ifndef QR_ROBOT_LOCALIZATION_MESSAGE_DETECTINGQR_H
#define QR_ROBOT_LOCALIZATION_MESSAGE_DETECTINGQR_H

#include <ros/service_traits.h>


#include <qr_robot_localization/DetectingQRRequest.h>
#include <qr_robot_localization/DetectingQRResponse.h>


namespace qr_robot_localization
{

struct DetectingQR
{

typedef DetectingQRRequest Request;
typedef DetectingQRResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct DetectingQR
} // namespace qr_robot_localization


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::qr_robot_localization::DetectingQR > {
  static const char* value()
  {
    return "a94c40e70a4b82863e6e52ec16732447";
  }

  static const char* value(const ::qr_robot_localization::DetectingQR&) { return value(); }
};

template<>
struct DataType< ::qr_robot_localization::DetectingQR > {
  static const char* value()
  {
    return "qr_robot_localization/DetectingQR";
  }

  static const char* value(const ::qr_robot_localization::DetectingQR&) { return value(); }
};


// service_traits::MD5Sum< ::qr_robot_localization::DetectingQRRequest> should match 
// service_traits::MD5Sum< ::qr_robot_localization::DetectingQR > 
template<>
struct MD5Sum< ::qr_robot_localization::DetectingQRRequest>
{
  static const char* value()
  {
    return MD5Sum< ::qr_robot_localization::DetectingQR >::value();
  }
  static const char* value(const ::qr_robot_localization::DetectingQRRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::qr_robot_localization::DetectingQRRequest> should match 
// service_traits::DataType< ::qr_robot_localization::DetectingQR > 
template<>
struct DataType< ::qr_robot_localization::DetectingQRRequest>
{
  static const char* value()
  {
    return DataType< ::qr_robot_localization::DetectingQR >::value();
  }
  static const char* value(const ::qr_robot_localization::DetectingQRRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::qr_robot_localization::DetectingQRResponse> should match 
// service_traits::MD5Sum< ::qr_robot_localization::DetectingQR > 
template<>
struct MD5Sum< ::qr_robot_localization::DetectingQRResponse>
{
  static const char* value()
  {
    return MD5Sum< ::qr_robot_localization::DetectingQR >::value();
  }
  static const char* value(const ::qr_robot_localization::DetectingQRResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::qr_robot_localization::DetectingQRResponse> should match 
// service_traits::DataType< ::qr_robot_localization::DetectingQR > 
template<>
struct DataType< ::qr_robot_localization::DetectingQRResponse>
{
  static const char* value()
  {
    return DataType< ::qr_robot_localization::DetectingQR >::value();
  }
  static const char* value(const ::qr_robot_localization::DetectingQRResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // QR_ROBOT_LOCALIZATION_MESSAGE_DETECTINGQR_H
