// Generated by gencpp from file ur_dashboard_msgs/IsInRemoteControlResponse.msg
// DO NOT EDIT!


#ifndef UR_DASHBOARD_MSGS_MESSAGE_ISINREMOTECONTROLRESPONSE_H
#define UR_DASHBOARD_MSGS_MESSAGE_ISINREMOTECONTROLRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ur_dashboard_msgs
{
template <class ContainerAllocator>
struct IsInRemoteControlResponse_
{
  typedef IsInRemoteControlResponse_<ContainerAllocator> Type;

  IsInRemoteControlResponse_()
    : answer()
    , in_remote_control(false)
    , success(false)  {
    }
  IsInRemoteControlResponse_(const ContainerAllocator& _alloc)
    : answer(_alloc)
    , in_remote_control(false)
    , success(false)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _answer_type;
  _answer_type answer;

   typedef uint8_t _in_remote_control_type;
  _in_remote_control_type in_remote_control;

   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> const> ConstPtr;

}; // struct IsInRemoteControlResponse_

typedef ::ur_dashboard_msgs::IsInRemoteControlResponse_<std::allocator<void> > IsInRemoteControlResponse;

typedef boost::shared_ptr< ::ur_dashboard_msgs::IsInRemoteControlResponse > IsInRemoteControlResponsePtr;
typedef boost::shared_ptr< ::ur_dashboard_msgs::IsInRemoteControlResponse const> IsInRemoteControlResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator1> & lhs, const ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator2> & rhs)
{
  return lhs.answer == rhs.answer &&
    lhs.in_remote_control == rhs.in_remote_control &&
    lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator1> & lhs, const ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ur_dashboard_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "032fdd19f824627299e7ba024ba3c0bc";
  }

  static const char* value(const ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x032fdd19f8246272ULL;
  static const uint64_t static_value2 = 0x99e7ba024ba3c0bcULL;
};

template<class ContainerAllocator>
struct DataType< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur_dashboard_msgs/IsInRemoteControlResponse";
  }

  static const char* value(const ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string answer\n"
"bool in_remote_control # is the robot currently in remote control mode?\n"
"bool success # Did the dashboard server call succeed?\n"
"\n"
;
  }

  static const char* value(const ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.answer);
      stream.next(m.in_remote_control);
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IsInRemoteControlResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur_dashboard_msgs::IsInRemoteControlResponse_<ContainerAllocator>& v)
  {
    s << indent << "answer: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.answer);
    s << indent << "in_remote_control: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.in_remote_control);
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR_DASHBOARD_MSGS_MESSAGE_ISINREMOTECONTROLRESPONSE_H