/* Auto-generated by genmsg_cpp for file /home/gmanfred/devel/ros/icra2013/segment_plans_objects/msg/ImageArray.msg */
#ifndef SEGMENT_PLANS_OBJECTS_MESSAGE_IMAGEARRAY_H
#define SEGMENT_PLANS_OBJECTS_MESSAGE_IMAGEARRAY_H
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

#include "sensor_msgs/Image.h"

namespace segment_plans_objects
{
template <class ContainerAllocator>
struct ImageArray_ {
  typedef ImageArray_<ContainerAllocator> Type;

  ImageArray_()
  : array()
  {
  }

  ImageArray_(const ContainerAllocator& _alloc)
  : array(_alloc)
  {
  }

  typedef std::vector< ::sensor_msgs::Image_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::Image_<ContainerAllocator> >::other >  _array_type;
  std::vector< ::sensor_msgs::Image_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::Image_<ContainerAllocator> >::other >  array;


  typedef boost::shared_ptr< ::segment_plans_objects::ImageArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::segment_plans_objects::ImageArray_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ImageArray
typedef  ::segment_plans_objects::ImageArray_<std::allocator<void> > ImageArray;

typedef boost::shared_ptr< ::segment_plans_objects::ImageArray> ImageArrayPtr;
typedef boost::shared_ptr< ::segment_plans_objects::ImageArray const> ImageArrayConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::segment_plans_objects::ImageArray_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::segment_plans_objects::ImageArray_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace segment_plans_objects

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::segment_plans_objects::ImageArray_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::segment_plans_objects::ImageArray_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::segment_plans_objects::ImageArray_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2d45007bb1ef655f17889f52265cf557";
  }

  static const char* value(const  ::segment_plans_objects::ImageArray_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x2d45007bb1ef655fULL;
  static const uint64_t static_value2 = 0x17889f52265cf557ULL;
};

template<class ContainerAllocator>
struct DataType< ::segment_plans_objects::ImageArray_<ContainerAllocator> > {
  static const char* value() 
  {
    return "segment_plans_objects/ImageArray";
  }

  static const char* value(const  ::segment_plans_objects::ImageArray_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::segment_plans_objects::ImageArray_<ContainerAllocator> > {
  static const char* value() 
  {
    return "sensor_msgs/Image[] array\n\
\n\
================================================================================\n\
MSG: sensor_msgs/Image\n\
# This message contains an uncompressed image\n\
# (0, 0) is at top-left corner of image\n\
#\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of cameara\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
                     # If the frame_id here and the frame_id of the CameraInfo\n\
                     # message associated with the image conflict\n\
                     # the behavior is undefined\n\
\n\
uint32 height         # image height, that is, number of rows\n\
uint32 width          # image width, that is, number of columns\n\
\n\
# The legal values for encoding are in file src/image_encodings.cpp\n\
# If you want to standardize a new string format, join\n\
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\
\n\
string encoding       # Encoding of pixels -- channel meaning, ordering, size\n\
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\
\n\
uint8 is_bigendian    # is this data bigendian?\n\
uint32 step           # Full row length in bytes\n\
uint8[] data          # actual matrix data, size is (step * rows)\n\
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
";
  }

  static const char* value(const  ::segment_plans_objects::ImageArray_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::segment_plans_objects::ImageArray_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.array);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ImageArray_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::segment_plans_objects::ImageArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::segment_plans_objects::ImageArray_<ContainerAllocator> & v) 
  {
    s << indent << "array[]" << std::endl;
    for (size_t i = 0; i < v.array.size(); ++i)
    {
      s << indent << "  array[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "    ", v.array[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // SEGMENT_PLANS_OBJECTS_MESSAGE_IMAGEARRAY_H
