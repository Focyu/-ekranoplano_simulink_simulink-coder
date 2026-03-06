#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include "rclcpp/rclcpp.hpp"
#include <gazebo_msgs/msg/entity_state.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include "pid_control_V1_types.h"
#include "slros_msgconvert_utils.h"


[[maybe_unused]] static void convertFromBus(gazebo_msgs::msg::EntityState& msgPtr, SL_Bus_gazebo_msgs_EntityState const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_gazebo_msgs_EntityState* busPtr, const gazebo_msgs::msg::EntityState& msgPtr);

[[maybe_unused]] static void convertFromBus(gazebo_msgs::srv::SetEntityState::Request& msgPtr, SL_Bus_gazebo_msgs_SetEntityStateRequest const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_gazebo_msgs_SetEntityStateRequest* busPtr, const gazebo_msgs::srv::SetEntityState::Request& msgPtr);

[[maybe_unused]] static void convertFromBus(gazebo_msgs::srv::SetEntityState::Response& msgPtr, SL_Bus_gazebo_msgs_SetEntityStateResponse const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_gazebo_msgs_SetEntityStateResponse* busPtr, const gazebo_msgs::srv::SetEntityState::Response& msgPtr);

[[maybe_unused]] static void convertFromBus(geometry_msgs::msg::Point& msgPtr, SL_Bus_geometry_msgs_Point const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_geometry_msgs_Point* busPtr, const geometry_msgs::msg::Point& msgPtr);

[[maybe_unused]] static void convertFromBus(geometry_msgs::msg::Pose& msgPtr, SL_Bus_geometry_msgs_Pose const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_geometry_msgs_Pose* busPtr, const geometry_msgs::msg::Pose& msgPtr);

[[maybe_unused]] static void convertFromBus(geometry_msgs::msg::Quaternion& msgPtr, SL_Bus_geometry_msgs_Quaternion const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_geometry_msgs_Quaternion* busPtr, const geometry_msgs::msg::Quaternion& msgPtr);

[[maybe_unused]] static void convertFromBus(geometry_msgs::msg::Twist& msgPtr, SL_Bus_geometry_msgs_Twist const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_geometry_msgs_Twist* busPtr, const geometry_msgs::msg::Twist& msgPtr);

[[maybe_unused]] static void convertFromBus(geometry_msgs::msg::Vector3& msgPtr, SL_Bus_geometry_msgs_Vector3 const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_geometry_msgs_Vector3* busPtr, const geometry_msgs::msg::Vector3& msgPtr);

[[maybe_unused]] static void convertFromBus(std_msgs::msg::Bool& msgPtr, SL_Bus_std_msgs_Bool const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_Bool* busPtr, const std_msgs::msg::Bool& msgPtr);

[[maybe_unused]] static void convertFromBus(std_msgs::msg::Float64& msgPtr, SL_Bus_std_msgs_Float64 const* busPtr);
[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_Float64* busPtr, const std_msgs::msg::Float64& msgPtr);



// Conversions between SL_Bus_gazebo_msgs_EntityState and gazebo_msgs::msg::EntityState

[[maybe_unused]] static void convertFromBus(gazebo_msgs::msg::EntityState& msgPtr, SL_Bus_gazebo_msgs_EntityState const* busPtr)
{
  const std::string rosMessageType("gazebo_msgs/EntityState");

  convertFromBusVariablePrimitiveArray(msgPtr.name, busPtr->name, busPtr->name_SL_Info);
  convertFromBus(msgPtr.pose, &busPtr->pose);
  convertFromBusVariablePrimitiveArray(msgPtr.reference_frame, busPtr->reference_frame, busPtr->reference_frame_SL_Info);
  convertFromBus(msgPtr.twist, &busPtr->twist);
}

[[maybe_unused]] static void convertToBus(SL_Bus_gazebo_msgs_EntityState* busPtr, const gazebo_msgs::msg::EntityState& msgPtr)
{
  const std::string rosMessageType("gazebo_msgs/EntityState");

  convertToBusVariablePrimitiveArray(busPtr->name, busPtr->name_SL_Info, msgPtr.name, slros::EnabledWarning(rosMessageType, "name"));
  convertToBus(&busPtr->pose, msgPtr.pose);
  convertToBusVariablePrimitiveArray(busPtr->reference_frame, busPtr->reference_frame_SL_Info, msgPtr.reference_frame, slros::EnabledWarning(rosMessageType, "reference_frame"));
  convertToBus(&busPtr->twist, msgPtr.twist);
}


// Conversions between SL_Bus_gazebo_msgs_SetEntityStateRequest and gazebo_msgs::srv::SetEntityState::Request

[[maybe_unused]] static void convertFromBus(gazebo_msgs::srv::SetEntityState::Request& msgPtr, SL_Bus_gazebo_msgs_SetEntityStateRequest const* busPtr)
{
  const std::string rosMessageType("gazebo_msgs/SetEntityStateRequest");

  convertFromBus(msgPtr.state, &busPtr->state);
}

[[maybe_unused]] static void convertToBus(SL_Bus_gazebo_msgs_SetEntityStateRequest* busPtr, const gazebo_msgs::srv::SetEntityState::Request& msgPtr)
{
  const std::string rosMessageType("gazebo_msgs/SetEntityStateRequest");

  convertToBus(&busPtr->state, msgPtr.state);
}


// Conversions between SL_Bus_gazebo_msgs_SetEntityStateResponse and gazebo_msgs::srv::SetEntityState::Response

[[maybe_unused]] static void convertFromBus(gazebo_msgs::srv::SetEntityState::Response& msgPtr, SL_Bus_gazebo_msgs_SetEntityStateResponse const* busPtr)
{
  const std::string rosMessageType("gazebo_msgs/SetEntityStateResponse");

  msgPtr.success =  busPtr->success;
}

[[maybe_unused]] static void convertToBus(SL_Bus_gazebo_msgs_SetEntityStateResponse* busPtr, const gazebo_msgs::srv::SetEntityState::Response& msgPtr)
{
  const std::string rosMessageType("gazebo_msgs/SetEntityStateResponse");

  busPtr->success =  msgPtr.success;
}


// Conversions between SL_Bus_geometry_msgs_Point and geometry_msgs::msg::Point

[[maybe_unused]] static void convertFromBus(geometry_msgs::msg::Point& msgPtr, SL_Bus_geometry_msgs_Point const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr.x =  busPtr->x;
  msgPtr.y =  busPtr->y;
  msgPtr.z =  busPtr->z;
}

[[maybe_unused]] static void convertToBus(SL_Bus_geometry_msgs_Point* busPtr, const geometry_msgs::msg::Point& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  busPtr->x =  msgPtr.x;
  busPtr->y =  msgPtr.y;
  busPtr->z =  msgPtr.z;
}


// Conversions between SL_Bus_geometry_msgs_Pose and geometry_msgs::msg::Pose

[[maybe_unused]] static void convertFromBus(geometry_msgs::msg::Pose& msgPtr, SL_Bus_geometry_msgs_Pose const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertFromBus(msgPtr.orientation, &busPtr->orientation);
  convertFromBus(msgPtr.position, &busPtr->position);
}

[[maybe_unused]] static void convertToBus(SL_Bus_geometry_msgs_Pose* busPtr, const geometry_msgs::msg::Pose& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertToBus(&busPtr->orientation, msgPtr.orientation);
  convertToBus(&busPtr->position, msgPtr.position);
}


// Conversions between SL_Bus_geometry_msgs_Quaternion and geometry_msgs::msg::Quaternion

[[maybe_unused]] static void convertFromBus(geometry_msgs::msg::Quaternion& msgPtr, SL_Bus_geometry_msgs_Quaternion const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  msgPtr.w =  busPtr->w;
  msgPtr.x =  busPtr->x;
  msgPtr.y =  busPtr->y;
  msgPtr.z =  busPtr->z;
}

[[maybe_unused]] static void convertToBus(SL_Bus_geometry_msgs_Quaternion* busPtr, const geometry_msgs::msg::Quaternion& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  busPtr->w =  msgPtr.w;
  busPtr->x =  msgPtr.x;
  busPtr->y =  msgPtr.y;
  busPtr->z =  msgPtr.z;
}


// Conversions between SL_Bus_geometry_msgs_Twist and geometry_msgs::msg::Twist

[[maybe_unused]] static void convertFromBus(geometry_msgs::msg::Twist& msgPtr, SL_Bus_geometry_msgs_Twist const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  convertFromBus(msgPtr.angular, &busPtr->angular);
  convertFromBus(msgPtr.linear, &busPtr->linear);
}

[[maybe_unused]] static void convertToBus(SL_Bus_geometry_msgs_Twist* busPtr, const geometry_msgs::msg::Twist& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  convertToBus(&busPtr->angular, msgPtr.angular);
  convertToBus(&busPtr->linear, msgPtr.linear);
}


// Conversions between SL_Bus_geometry_msgs_Vector3 and geometry_msgs::msg::Vector3

[[maybe_unused]] static void convertFromBus(geometry_msgs::msg::Vector3& msgPtr, SL_Bus_geometry_msgs_Vector3 const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  msgPtr.x =  busPtr->x;
  msgPtr.y =  busPtr->y;
  msgPtr.z =  busPtr->z;
}

[[maybe_unused]] static void convertToBus(SL_Bus_geometry_msgs_Vector3* busPtr, const geometry_msgs::msg::Vector3& msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  busPtr->x =  msgPtr.x;
  busPtr->y =  msgPtr.y;
  busPtr->z =  msgPtr.z;
}


// Conversions between SL_Bus_std_msgs_Bool and std_msgs::msg::Bool

[[maybe_unused]] static void convertFromBus(std_msgs::msg::Bool& msgPtr, SL_Bus_std_msgs_Bool const* busPtr)
{
  const std::string rosMessageType("std_msgs/Bool");

  msgPtr.data =  busPtr->data;
}

[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_Bool* busPtr, const std_msgs::msg::Bool& msgPtr)
{
  const std::string rosMessageType("std_msgs/Bool");

  busPtr->data =  msgPtr.data;
}


// Conversions between SL_Bus_std_msgs_Float64 and std_msgs::msg::Float64

[[maybe_unused]] static void convertFromBus(std_msgs::msg::Float64& msgPtr, SL_Bus_std_msgs_Float64 const* busPtr)
{
  const std::string rosMessageType("std_msgs/Float64");

  msgPtr.data =  busPtr->data;
}

[[maybe_unused]] static void convertToBus(SL_Bus_std_msgs_Float64* busPtr, const std_msgs::msg::Float64& msgPtr)
{
  const std::string rosMessageType("std_msgs/Float64");

  busPtr->data =  msgPtr.data;
}



#endif
