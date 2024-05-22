cmake_minimum_required(VERSION 3.8)
project(camera_faker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_localization REQUIRED)
find_package(glfw3 REQUIRED)
find_package(assimp REQUIRED)
find_package(Freetype REQUIRED)

add_executable(zed_faker include/external/glad/glad.c src/zed_faker.cpp)

target_include_directories(zed_faker PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/external>
    $<INSTALL_INTERFACE:include>
    ${YAML_CPP_INCLUDEDIR}
    ${FREETYPE_INCLUDE_DIRS})

ament_target_dependencies(zed_faker 
  Eigen3 
  rclcpp 
  urdf
  tf2
  tf2_ros
  std_msgs
  riptide_msgs2
  robot_localization
  visualization_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  yaml-cpp
  glfw3
  assimp
  Freetype)

target_link_libraries(zed_faker 
  ${ASSIMP_LIBRARIES}
  ${FREETYPE_LIBRARIES}
  glfw
)

install(TARGETS zed_faker
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch textures shaders models
  DESTINATION share/${PROJECT_NAME}
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
