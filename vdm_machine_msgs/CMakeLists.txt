cmake_minimum_required(VERSION 3.8)
project(vdm_machine_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/MachineData.msg"
  "msg/MachineDataStamped.msg"
  "msg/MachineLog.msg"
  "msg/MachineLogStamped.msg"
  "msg/MachineProducePlan.msg"
  "msg/MachinesStateStamped.msg"
  "msg/MachineState.msg"
  "msg/MachineStateArray.msg"
  "msg/OverralMachine.msg"
  "msg/ResetMachinePLC.msg"
  "msg/ValueSetting.msg"
)

set(srv_files
  "srv/CreateMachine.srv"
  "srv/DeleteMachine.srv"
  "srv/GetAllMachineName.srv"
  "srv/GetAllStageData.srv"
  "srv/GetMachineData.srv"
  "srv/GetMachineLogs.srv"
  "srv/GetMinMaxDate.srv"
  "srv/GetProducePlan.srv"
  "srv/GetStageData.srv"
  "srv/GetStagesLogs.srv"
  "srv/ResetMachine.srv"
  "srv/ResetMachinePLC.srv"
  "srv/UpdateMachine.srv"
  "srv/UpdateProducePlan.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
