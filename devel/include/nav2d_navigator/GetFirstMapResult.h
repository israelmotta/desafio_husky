// Generated by gencpp from file nav2d_navigator/GetFirstMapResult.msg
// DO NOT EDIT!


#ifndef NAV2D_NAVIGATOR_MESSAGE_GETFIRSTMAPRESULT_H
#define NAV2D_NAVIGATOR_MESSAGE_GETFIRSTMAPRESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace nav2d_navigator
{
template <class ContainerAllocator>
struct GetFirstMapResult_
{
  typedef GetFirstMapResult_<ContainerAllocator> Type;

  GetFirstMapResult_()
    {
    }
  GetFirstMapResult_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> const> ConstPtr;

}; // struct GetFirstMapResult_

typedef ::nav2d_navigator::GetFirstMapResult_<std::allocator<void> > GetFirstMapResult;

typedef boost::shared_ptr< ::nav2d_navigator::GetFirstMapResult > GetFirstMapResultPtr;
typedef boost::shared_ptr< ::nav2d_navigator::GetFirstMapResult const> GetFirstMapResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace nav2d_navigator

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav2d_navigator': ['/home/israel/desafio_husky/devel/share/nav2d_navigator/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nav2d_navigator/GetFirstMapResult";
  }

  static const char* value(const ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
;
  }

  static const char* value(const ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetFirstMapResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::nav2d_navigator::GetFirstMapResult_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // NAV2D_NAVIGATOR_MESSAGE_GETFIRSTMAPRESULT_H
