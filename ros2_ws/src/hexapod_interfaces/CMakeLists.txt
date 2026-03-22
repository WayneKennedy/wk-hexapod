cmake_minimum_required(VERSION 3.8)
project(hexapod_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Face recognition messages
  "msg/Face.msg"
  "msg/FaceArray.msg"
  "msg/FaceEvent.msg"
  # Autonomy messages
  "msg/AutonomyState.msg"
  "msg/LocalizationStatus.msg"
  # Face recognition services
  "srv/CaptureTrainingImage.srv"
  "srv/TrainModel.srv"
  "srv/ListKnownFaces.srv"
  # Autonomy services
  "srv/GetAutonomyState.srv"
  "srv/StartMission.srv"
  "srv/StopMission.srv"
  # Actions
  "action/MoveDistance.action"
  "action/LookAround.action"
  "action/ExploreFrontiers.action"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
