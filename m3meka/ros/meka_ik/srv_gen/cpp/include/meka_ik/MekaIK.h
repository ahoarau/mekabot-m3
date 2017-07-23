/* Auto-generated by genmsg_cpp for file /home/meka/mekabot/m3meka/ros/meka_ik/srv/MekaIK.srv */
#ifndef MEKA_IK_SERVICE_MEKAIK_H
#define MEKA_IK_SERVICE_MEKAIK_H
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

#include "ros/service_traits.h"




namespace meka_ik
{
template <class ContainerAllocator>
struct MekaIKRequest_ {
  typedef MekaIKRequest_<ContainerAllocator> Type;

  MekaIKRequest_()
  : arm_name()
  , end_position()
  , end_rpy()
  , angles_current()
  {
    end_position.assign(0.0);
    end_rpy.assign(0.0);
    angles_current.assign(0.0);
  }

  MekaIKRequest_(const ContainerAllocator& _alloc)
  : arm_name(_alloc)
  , end_position()
  , end_rpy()
  , angles_current()
  {
    end_position.assign(0.0);
    end_rpy.assign(0.0);
    angles_current.assign(0.0);
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _arm_name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  arm_name;

  typedef boost::array<float, 3>  _end_position_type;
  boost::array<float, 3>  end_position;

  typedef boost::array<float, 3>  _end_rpy_type;
  boost::array<float, 3>  end_rpy;

  typedef boost::array<float, 7>  _angles_current_type;
  boost::array<float, 7>  angles_current;


  typedef boost::shared_ptr< ::meka_ik::MekaIKRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::meka_ik::MekaIKRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MekaIKRequest
typedef  ::meka_ik::MekaIKRequest_<std::allocator<void> > MekaIKRequest;

typedef boost::shared_ptr< ::meka_ik::MekaIKRequest> MekaIKRequestPtr;
typedef boost::shared_ptr< ::meka_ik::MekaIKRequest const> MekaIKRequestConstPtr;


template <class ContainerAllocator>
struct MekaIKResponse_ {
  typedef MekaIKResponse_<ContainerAllocator> Type;

  MekaIKResponse_()
  : success(false)
  , angles_solution()
  {
    angles_solution.assign(0.0);
  }

  MekaIKResponse_(const ContainerAllocator& _alloc)
  : success(false)
  , angles_solution()
  {
    angles_solution.assign(0.0);
  }

  typedef uint8_t _success_type;
  uint8_t success;

  typedef boost::array<float, 7>  _angles_solution_type;
  boost::array<float, 7>  angles_solution;


  typedef boost::shared_ptr< ::meka_ik::MekaIKResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::meka_ik::MekaIKResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MekaIKResponse
typedef  ::meka_ik::MekaIKResponse_<std::allocator<void> > MekaIKResponse;

typedef boost::shared_ptr< ::meka_ik::MekaIKResponse> MekaIKResponsePtr;
typedef boost::shared_ptr< ::meka_ik::MekaIKResponse const> MekaIKResponseConstPtr;

struct MekaIK
{

typedef MekaIKRequest Request;
typedef MekaIKResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct MekaIK
} // namespace meka_ik

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::meka_ik::MekaIKRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::meka_ik::MekaIKRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::meka_ik::MekaIKRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "27d4170986f50b4c204c4a40684420d7";
  }

  static const char* value(const  ::meka_ik::MekaIKRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x27d4170986f50b4cULL;
  static const uint64_t static_value2 = 0x204c4a40684420d7ULL;
};

template<class ContainerAllocator>
struct DataType< ::meka_ik::MekaIKRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "meka_ik/MekaIKRequest";
  }

  static const char* value(const  ::meka_ik::MekaIKRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::meka_ik::MekaIKRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string arm_name\n\
float32[3] end_position\n\
float32[3] end_rpy\n\
float32[7] angles_current\n\
\n\
";
  }

  static const char* value(const  ::meka_ik::MekaIKRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::meka_ik::MekaIKResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::meka_ik::MekaIKResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::meka_ik::MekaIKResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f2d3205e4d921a993b3ae1d8ef840cf2";
  }

  static const char* value(const  ::meka_ik::MekaIKResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf2d3205e4d921a99ULL;
  static const uint64_t static_value2 = 0x3b3ae1d8ef840cf2ULL;
};

template<class ContainerAllocator>
struct DataType< ::meka_ik::MekaIKResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "meka_ik/MekaIKResponse";
  }

  static const char* value(const  ::meka_ik::MekaIKResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::meka_ik::MekaIKResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool success\n\
float32[7] angles_solution\n\
\n\
";
  }

  static const char* value(const  ::meka_ik::MekaIKResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::meka_ik::MekaIKResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::meka_ik::MekaIKRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.arm_name);
    stream.next(m.end_position);
    stream.next(m.end_rpy);
    stream.next(m.angles_current);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MekaIKRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::meka_ik::MekaIKResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.success);
    stream.next(m.angles_solution);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MekaIKResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<meka_ik::MekaIK> {
  static const char* value() 
  {
    return "ad8c64be4db919e70fe6ca282436c423";
  }

  static const char* value(const meka_ik::MekaIK&) { return value(); } 
};

template<>
struct DataType<meka_ik::MekaIK> {
  static const char* value() 
  {
    return "meka_ik/MekaIK";
  }

  static const char* value(const meka_ik::MekaIK&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<meka_ik::MekaIKRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ad8c64be4db919e70fe6ca282436c423";
  }

  static const char* value(const meka_ik::MekaIKRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<meka_ik::MekaIKRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "meka_ik/MekaIK";
  }

  static const char* value(const meka_ik::MekaIKRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<meka_ik::MekaIKResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ad8c64be4db919e70fe6ca282436c423";
  }

  static const char* value(const meka_ik::MekaIKResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<meka_ik::MekaIKResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "meka_ik/MekaIK";
  }

  static const char* value(const meka_ik::MekaIKResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // MEKA_IK_SERVICE_MEKAIK_H

