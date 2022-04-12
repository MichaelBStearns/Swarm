// Generated by gencpp from file swarm/column.msg
// DO NOT EDIT!


#ifndef SWARM_MESSAGE_COLUMN_H
#define SWARM_MESSAGE_COLUMN_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <swarm/square.h>

namespace swarm
{
template <class ContainerAllocator>
struct column_
{
  typedef column_<ContainerAllocator> Type;

  column_()
    : row()  {
    }
  column_(const ContainerAllocator& _alloc)
    : row()  {
  (void)_alloc;
      row.assign( ::swarm::square_<ContainerAllocator> (_alloc));
  }



   typedef boost::array< ::swarm::square_<ContainerAllocator> , 100>  _row_type;
  _row_type row;





  typedef boost::shared_ptr< ::swarm::column_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::swarm::column_<ContainerAllocator> const> ConstPtr;

}; // struct column_

typedef ::swarm::column_<std::allocator<void> > column;

typedef boost::shared_ptr< ::swarm::column > columnPtr;
typedef boost::shared_ptr< ::swarm::column const> columnConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::swarm::column_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::swarm::column_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::swarm::column_<ContainerAllocator1> & lhs, const ::swarm::column_<ContainerAllocator2> & rhs)
{
  return lhs.row == rhs.row;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::swarm::column_<ContainerAllocator1> & lhs, const ::swarm::column_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace swarm

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::swarm::column_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::swarm::column_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::swarm::column_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::swarm::column_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::swarm::column_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::swarm::column_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::swarm::column_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3606c5dc7d55d80c61e5a77e52f55b69";
  }

  static const char* value(const ::swarm::column_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3606c5dc7d55d80cULL;
  static const uint64_t static_value2 = 0x61e5a77e52f55b69ULL;
};

template<class ContainerAllocator>
struct DataType< ::swarm::column_<ContainerAllocator> >
{
  static const char* value()
  {
    return "swarm/column";
  }

  static const char* value(const ::swarm::column_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::swarm::column_<ContainerAllocator> >
{
  static const char* value()
  {
    return "square[100] row\n"
"================================================================================\n"
"MSG: swarm/square\n"
"char[5] pheromones\n"
;
  }

  static const char* value(const ::swarm::column_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::swarm::column_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.row);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct column_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::swarm::column_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::swarm::column_<ContainerAllocator>& v)
  {
    s << indent << "row[]" << std::endl;
    for (size_t i = 0; i < v.row.size(); ++i)
    {
      s << indent << "  row[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::swarm::square_<ContainerAllocator> >::stream(s, indent + "    ", v.row[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SWARM_MESSAGE_COLUMN_H
