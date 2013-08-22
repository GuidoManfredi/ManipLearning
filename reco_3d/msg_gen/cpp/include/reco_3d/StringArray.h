/* Auto-generated by genmsg_cpp for file /home/gmanfred/devel/ros/packs/vision/reco/reco_3d/msg/StringArray.msg */
#ifndef RECO_3D_MESSAGE_STRINGARRAY_H
#define RECO_3D_MESSAGE_STRINGARRAY_H
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

#include "std_msgs/String.h"

namespace reco_3d
{
template <class ContainerAllocator>
struct StringArray_ {
  typedef StringArray_<ContainerAllocator> Type;

  StringArray_()
  : array()
  {
  }

  StringArray_(const ContainerAllocator& _alloc)
  : array(_alloc)
  {
  }

  typedef std::vector< ::std_msgs::String_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::std_msgs::String_<ContainerAllocator> >::other >  _array_type;
  std::vector< ::std_msgs::String_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::std_msgs::String_<ContainerAllocator> >::other >  array;


  typedef boost::shared_ptr< ::reco_3d::StringArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::reco_3d::StringArray_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct StringArray
typedef  ::reco_3d::StringArray_<std::allocator<void> > StringArray;

typedef boost::shared_ptr< ::reco_3d::StringArray> StringArrayPtr;
typedef boost::shared_ptr< ::reco_3d::StringArray const> StringArrayConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::reco_3d::StringArray_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::reco_3d::StringArray_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace reco_3d

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::reco_3d::StringArray_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::reco_3d::StringArray_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::reco_3d::StringArray_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5971fc61eaffe5ed44eaa15ad61177ae";
  }

  static const char* value(const  ::reco_3d::StringArray_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5971fc61eaffe5edULL;
  static const uint64_t static_value2 = 0x44eaa15ad61177aeULL;
};

template<class ContainerAllocator>
struct DataType< ::reco_3d::StringArray_<ContainerAllocator> > {
  static const char* value() 
  {
    return "reco_3d/StringArray";
  }

  static const char* value(const  ::reco_3d::StringArray_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::reco_3d::StringArray_<ContainerAllocator> > {
  static const char* value() 
  {
    return "std_msgs/String[] array\n\
\n\
================================================================================\n\
MSG: std_msgs/String\n\
string data\n\
\n\
";
  }

  static const char* value(const  ::reco_3d::StringArray_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::reco_3d::StringArray_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.array);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct StringArray_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::reco_3d::StringArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::reco_3d::StringArray_<ContainerAllocator> & v) 
  {
    s << indent << "array[]" << std::endl;
    for (size_t i = 0; i < v.array.size(); ++i)
    {
      s << indent << "  array[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "    ", v.array[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // RECO_3D_MESSAGE_STRINGARRAY_H

