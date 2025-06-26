cmake_minimum_required(VERSION 3.5)
project(origincar_competition)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hbm_img_msgs REQUIRED)
add_definitions(-DBUILD_HBMEM_MSG)

# find ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(origincar_msg REQUIRED)

# 添加库搜索路径（关键修正）
link_directories(/usr/hobot/lib)

add_executable(origincar_competition src/origincar_competition.cpp)

# 链接库（确保 hbmem、ion、alog 正确链接）
target_link_libraries(origincar_competition
  hbmem
  alog
  bp
  ltp
  ici
  cgr
)

# 链接 ROS2 依赖库
ament_target_dependencies(
  origincar_competition
  rclcpp
  sensor_msgs
  hbm_img_msgs
  OpenCV 
  ai_msgs
  cv_bridge
  origincar_msg
)

target_include_directories(origincar_competition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 安装 launch 文件
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装可执行文件
install(
  TARGETS origincar_competition
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()