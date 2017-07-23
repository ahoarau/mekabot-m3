/* Auto-generated by genmsg_cpp for file /home/meka/mekabot/m3meka/ros/shm_led_x2xn/msg/LEDX2XNCmd.msg */
#ifndef SHM_LED_X2XN_MESSAGE_LEDX2XNCMD_H
#define SHM_LED_X2XN_MESSAGE_LEDX2XNCMD_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "shm_led_x2xn/LEDX2XNRGB.h"
#include "shm_led_x2xn/LEDX2XNRGB.h"

namespace shm_led_x2xn
{
template <class ContainerAllocator>
struct LEDX2XNCmd_ {
  typedef LEDX2XNCmd_<ContainerAllocator> Type;

  LEDX2XNCmd_()
  : header()
  , enable_a(0)
  , enable_b(0)
  , branch_a()
  , branch_b()
  {
  }

  LEDX2XNCmd_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , enable_a(0)
  , enable_b(0)
  , branch_a(_alloc)
  , branch_b(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef uint8_t _enable_a_type;
  uint8_t enable_a;

  typedef uint8_t _enable_b_type;
  uint8_t enable_b;

  typedef std::vector< ::shm_led_x2xn::LEDX2XNRGB_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::shm_led_x2xn::LEDX2XNRGB_<ContainerAllocator> >::other >  _branch_a_type;
  std::vector< ::shm_led_x2xn::LEDX2XNRGB_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::shm_led_x2xn::LEDX2XNRGB_<ContainerAllocator> >::other >  branch_a;

  typedef std::vector< ::shm_led_x2xn::LEDX2XNRGB_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::shm_led_x2xn::LEDX2XNRGB_<ContainerAllocator> >::other >  _branch_b_type;
  std::vector< ::shm_led_x2xn::LEDX2XNRGB_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::shm_led_x2xn::LEDX2XNRGB_<ContainerAllocator> >::other >  branch_b;


  typedef boost::shared_ptr< ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct LEDX2XNCmd
typedef  ::shm_led_x2xn::LEDX2XNCmd_<std::allocator<void> > LEDX2XNCmd;

typedef boost::shared_ptr< ::shm_led_x2xn::LEDX2XNCmd> LEDX2XNCmdPtr;
typedef boost::shared_ptr< ::shm_led_x2xn::LEDX2XNCmd const> LEDX2XNCmdConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace shm_led_x2xn

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d1c07f75260256e0d40356eab79bc22f";
  }

  static const char* value(const  ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd1c07f75260256e0ULL;
  static const uint64_t static_value2 = 0xd40356eab79bc22fULL;
};

template<class ContainerAllocator>
struct DataType< ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "shm_led_x2xn/LEDX2XNCmd";
  }

  static const char* value(const  ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
uint8 enable_a\n\
uint8 enable_b\n\
LEDX2XNRGB[] branch_a\n\
LEDX2XNRGB[] branch_b\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: shm_led_x2xn/LEDX2XNRGB\n\
uint32 r\n\
uint32 g\n\
uint32 b\n\
";
  }

  static const char* value(const  ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.enable_a);
    stream.next(m.enable_b);
    stream.next(m.branch_a);
    stream.next(m.branch_b);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct LEDX2XNCmd_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::shm_led_x2xn::LEDX2XNCmd_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "enable_a: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.enable_a);
    s << indent << "enable_b: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.enable_b);
    s << indent << "branch_a[]" << std::endl;
    for (size_t i = 0; i < v.branch_a.size(); ++i)
    {
      s << indent << "  branch_a[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::shm_led_x2xn::LEDX2XNRGB_<ContainerAllocator> >::stream(s, indent + "    ", v.branch_a[i]);
    }
    s << indent << "branch_b[]" << std::endl;
    for (size_t i = 0; i < v.branch_b.size(); ++i)
    {
      s << indent << "  branch_b[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::shm_led_x2xn::LEDX2XNRGB_<ContainerAllocator> >::stream(s, indent + "    ", v.branch_b[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // SHM_LED_X2XN_MESSAGE_LEDX2XNCMD_H

