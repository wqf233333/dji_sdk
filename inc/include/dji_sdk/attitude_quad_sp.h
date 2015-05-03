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
 * Auto-generated by genmsg_cpp from file /home/iarc/spline_ws/src/dji_sdk/msg/attitude_quad_sp.msg
 *
 */


#ifndef DJI_SDK_MESSAGE_ATTITUDE_QUAD_SP_H
#define DJI_SDK_MESSAGE_ATTITUDE_QUAD_SP_H


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
struct attitude_quad_sp_
{
  typedef attitude_quad_sp_<ContainerAllocator> Type;

  attitude_quad_sp_()
    : q0(0.0)
    , q1(0.0)
    , q2(0.0)
    , q3(0.0)  {
    }
  attitude_quad_sp_(const ContainerAllocator& _alloc)
    : q0(0.0)
    , q1(0.0)
    , q2(0.0)
    , q3(0.0)  {
    }



   typedef float _q0_type;
  _q0_type q0;

   typedef float _q1_type;
  _q1_type q1;

   typedef float _q2_type;
  _q2_type q2;

   typedef float _q3_type;
  _q3_type q3;




  typedef boost::shared_ptr< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct attitude_quad_sp_

typedef ::dji_sdk::attitude_quad_sp_<std::allocator<void> > attitude_quad_sp;

typedef boost::shared_ptr< ::dji_sdk::attitude_quad_sp > attitude_quad_spPtr;
typedef boost::shared_ptr< ::dji_sdk::attitude_quad_sp const> attitude_quad_spConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::attitude_quad_sp_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "63ca77594d1bad6ed3441f10f9332674";
  }

  static const char* value(const ::dji_sdk::attitude_quad_sp_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x63ca77594d1bad6eULL;
  static const uint64_t static_value2 = 0xd3441f10f9332674ULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/attitude_quad_sp";
  }

  static const char* value(const ::dji_sdk::attitude_quad_sp_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 q0\n\
float32 q1\n\
float32 q2\n\
float32 q3\n\
";
  }

  static const char* value(const ::dji_sdk::attitude_quad_sp_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.q0);
      stream.next(m.q1);
      stream.next(m.q2);
      stream.next(m.q3);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct attitude_quad_sp_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::attitude_quad_sp_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::attitude_quad_sp_<ContainerAllocator>& v)
  {
    s << indent << "q0: ";
    Printer<float>::stream(s, indent + "  ", v.q0);
    s << indent << "q1: ";
    Printer<float>::stream(s, indent + "  ", v.q1);
    s << indent << "q2: ";
    Printer<float>::stream(s, indent + "  ", v.q2);
    s << indent << "q3: ";
    Printer<float>::stream(s, indent + "  ", v.q3);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_ATTITUDE_QUAD_SP_H