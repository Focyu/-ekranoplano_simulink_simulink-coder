// Copyright 2022-2024 The MathWorks, Inc.
// Generated 06-Mar-2026 11:19:12
#include "slros2_initialize.h"
const std::string SLROSNodeName("pid_control_V1");
// pid_control_V1/Subscribe
SimulinkSubscriber<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Sub_pid_control_V1_366;
// pid_control_V1/Subscribe-PITCH
SimulinkSubscriber<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Sub_pid_control_V1_370;
// pid_control_V1/Subscribe-YAW
SimulinkSubscriber<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Sub_pid_control_V1_377;
// pid_control_V1/Subsystem/Subscribe
SimulinkSubscriber<std_msgs::msg::Bool,SL_Bus_std_msgs_Bool> Sub_pid_control_V1_417;
// pid_control_V1/Subsystem/Subscribe1
SimulinkSubscriber<std_msgs::msg::Bool,SL_Bus_std_msgs_Bool> Sub_pid_control_V1_423;
// pid_control_V1/Call Service
SimulinkServiceCaller<gazebo_msgs::srv::SetEntityState,SL_Bus_gazebo_msgs_SetEntityStateRequest,SL_Bus_gazebo_msgs_SetEntityStateResponse> ServCall_pid_control_V1_326;
