/* Auto-generated by genmsg_cpp for file /home/gmanfred/devel/ros/packs/vision/reco/reco_3d/msg/OBB.msg */
#ifndef RECO_3D_MESSAGE_OBB_H
#define RECO_3D_MESSAGE_OBB_H
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

#include "geometry_msgs/PoseStamped.h"

namespace reco_3d
{
template <class ContainerAllocator>
struct OBB_ {
  typedef OBB_<ContainerAllocator> Type;

  OBB_()
  : width(0.0)
  , height(0.0)
  , depth(0.0)
  , transform()
  {
  }

  OBB_(const ContainerAllocator& _alloc)
  : width(0.0)
  , height(0.0)
  , depth(0.0)
  , transform(_alloc)
  {
  }

  typedef float _width_type;
  float width;

  typedef float _height_type;
  float height;

  typedef float _depth_type;
  float depth;

  typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _transform_type;
   ::geometry_msgs::PoseStamped_<ContainerAllocator>  transform;


  typedef boost::shared_ptr< ::reco_3d::OBB_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::reco_3d::OBB_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct OBB
typedef  ::reco_3d::OBB_<std::allocator<void> > OBB;

typedef boost::shared_ptr< ::reco_3d::OBB> OBBPtr;
typedef boost::shared_ptr< ::reco_3d::OBB const> OBBConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::reco_3d::OBB_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::reco_3d::OBB_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace reco_3d

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::reco_3d::OBB_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::reco_3d::OBB_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::reco_3d::OBB_<ContainerAllocator> > {
  static const char* value() 
  {
    return "06791cf24424e92552191ddad658185b";
  }

  static const char* value(const  ::reco_3d::OBB_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x06791cf24424e925ULL;
  static const uint64_t static_value2 = 0x52191ddad658185bULL;
};

template<class ContainerAllocator>
struct DataType< ::reco_3d::OBB_<ContainerAllocator> > {
  static const char* value() 
  {
    return "reco_3d/OBB";
  }

  static const char* value(const  ::reco_3d::OBB_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::reco_3d::OBB_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 width\n\
float32 height\n\
float32 depth\n\
geometry_msgs/PoseStamped transform\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
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
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::reco_3d::OBB_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::reco_3d::OBB_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.width);
    stream.next(m.height);
    stream.next(m.depth);
    stream.next(m.transform);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct OBB_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::reco_3d::OBB_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::reco_3d::OBB_<ContainerAllocator> & v) 
  {
    s << indent << "width: ";
    Printer<float>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<float>::stream(s, indent + "  ", v.height);
    s << indent << "depth: ";
    Printer<float>::stream(s, indent + "  ", v.depth);
    s << indent << "transform: ";
s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.transform);
  }
};


} // namespace message_operations
} // namespace ros

#endif // RECO_3D_MESSAGE_OBB_H

