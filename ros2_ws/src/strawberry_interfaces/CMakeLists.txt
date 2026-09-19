# Copyright 2026 Strawberry URP Team
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(strawberry_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StrawberryDetection.msg"
  "msg/StrawberryDetectionArray.msg"
  "msg/TargetPose.msg"
  "msg/TrackedTarget.msg"
  "msg/TrackedTargetArray.msg"
  "msg/ObservationPlan.msg"
  "srv/MoveToObservation.srv"
  "srv/EvaluateTarget.srv"
  "action/PickAndPlace.action"
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(interface_contract_test
    test/test_interface_contract.py
    TIMEOUT 30
  )
endif()

ament_package()
