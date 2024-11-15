// Generated by gencpp from file navigation_pkg/Coord2d.msg
// DO NOT EDIT!


#ifndef NAVIGATION_PKG_MESSAGE_COORD2D_H
#define NAVIGATION_PKG_MESSAGE_COORD2D_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace navigation_pkg
{
template <class ContainerAllocator>
struct Coord2d_
{
  typedef Coord2d_<ContainerAllocator> Type;

  Coord2d_()
    : x(0.0)
    , y(0.0)  {
    }
  Coord2d_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::navigation_pkg::Coord2d_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::navigation_pkg::Coord2d_<ContainerAllocator> const> ConstPtr;

}; // struct Coord2d_

typedef ::navigation_pkg::Coord2d_<std::allocator<void> > Coord2d;

typedef boost::shared_ptr< ::navigation_pkg::Coord2d > Coord2dPtr;
typedef boost::shared_ptr< ::navigation_pkg::Coord2d const> Coord2dConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::navigation_pkg::Coord2d_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::navigation_pkg::Coord2d_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::navigation_pkg::Coord2d_<ContainerAllocator1> & lhs, const ::navigation_pkg::Coord2d_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::navigation_pkg::Coord2d_<ContainerAllocator1> & lhs, const ::navigation_pkg::Coord2d_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace navigation_pkg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::navigation_pkg::Coord2d_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::navigation_pkg::Coord2d_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navigation_pkg::Coord2d_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navigation_pkg::Coord2d_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation_pkg::Coord2d_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation_pkg::Coord2d_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::navigation_pkg::Coord2d_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff8d7d66dd3e4b731ef14a45d38888b6";
  }

  static const char* value(const ::navigation_pkg::Coord2d_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff8d7d66dd3e4b73ULL;
  static const uint64_t static_value2 = 0x1ef14a45d38888b6ULL;
};

template<class ContainerAllocator>
struct DataType< ::navigation_pkg::Coord2d_<ContainerAllocator> >
{
  static const char* value()
  {
    return "navigation_pkg/Coord2d";
  }

  static const char* value(const ::navigation_pkg::Coord2d_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::navigation_pkg::Coord2d_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
;
  }

  static const char* value(const ::navigation_pkg::Coord2d_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::navigation_pkg::Coord2d_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Coord2d_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::navigation_pkg::Coord2d_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::navigation_pkg::Coord2d_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAVIGATION_PKG_MESSAGE_COORD2D_H
