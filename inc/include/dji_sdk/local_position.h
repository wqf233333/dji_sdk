/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/iarc/spline_ws/src/dji_sdk/msg/local_position.msg
 *
 */


#ifndef DJI_SDK_MESSAGE_LOCAL_POSITION_H
#define DJI_SDK_MESSAGE_LOCAL_POSITION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dji_sdk
{
template <class ContainerAllocator>
struct local_position_
{
  typedef local_position_<ContainerAllocator> Type;

  local_position_()
    : ts(0)
    , x(0.0)
    , y(0.0)
    , height(0.0)  {
    }
  local_position_(const ContainerAllocator& _alloc)
    : ts(0)
    , x(0.0)
    , y(0.0)
    , height(0.0)  {
    }



   typedef int32_t _ts_type;
  _ts_type ts;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _height_type;
  _height_type height;




  typedef boost::shared_ptr< ::dji_sdk::local_position_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::local_position_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct local_position_

typedef ::dji_sdk::local_position_<std::allocator<void> > local_position;

typedef boost::shared_ptr< ::dji_sdk::local_position > local_positionPtr;
typedef boost::shared_ptr< ::dji_sdk::local_position const> local_positionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::local_position_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::local_position_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dji_sdk

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'dji_sdk': ['/home/iarc/spline_ws/src/dji_sdk/msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::local_position_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::local_position_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::local_position_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::local_position_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::local_position_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::local_position_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::local_position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8b845cd1de8cf0e094afada22e702b2e";
  }

  static const char* value(const ::dji_sdk::local_position_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8b845cd1de8cf0e0ULL;
  static const uint64_t static_value2 = 0x94afada22e702b2eULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::local_position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/local_position";
  }

  static const char* value(const ::dji_sdk::local_position_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::local_position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 ts\n\
float32 x\n\
float32 y\n\
float32 height\n\
";
  }

  static const char* value(const ::dji_sdk::local_position_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::local_position_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ts);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.height);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct local_position_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::local_position_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::local_position_<ContainerAllocator>& v)
  {
    s << indent << "ts: ";
    Printer<int32_t>::stream(s, indent + "  ", v.ts);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "height: ";
    Printer<float>::stream(s, indent + "  ", v.height);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_LOCAL_POSITION_H