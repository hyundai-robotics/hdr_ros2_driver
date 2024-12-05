cmake_minimum_required(VERSION 3.8)
project(hdr_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  
  "srv/IoRequest.srv"
  "srv/OpCnd.srv"
  
  "srv/FilePath.srv"
  "srv/FileRename.srv"
  "srv/FileSend.srv"
  "srv/FileList.srv"

  "srv/LogManager.srv"
  "srv/Number.srv"

  "srv/ProgramCnt.srv"
  "srv/ProgramVar.srv"

  "srv/DateTime.srv"

  "srv/PoseCur.srv"

  "srv/IoplcGet.srv"
  "srv/IoplcPost.srv"

  "srv/Emergency.srv"
  "srv/ExecuteMove.srv"
  "srv/ExecuteCmd.srv"
  

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
