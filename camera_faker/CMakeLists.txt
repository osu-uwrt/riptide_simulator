cmake_minimum_required(VERSION 3.8)
project(camera_faker)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(glfw3 REQUIRED)
find_package(assimp REQUIRED)
find_package(OpenCV REQUIRED)

set_source_files_properties(include/external/glad/glad.c PROPERTIES COMPILE_OPTIONS "-Wno-pedantic")

add_library(pool_imgui STATIC
  vendor/imgui/imgui.cpp vendor/imgui/imgui_draw.cpp
  vendor/imgui/imgui_tables.cpp vendor/imgui/imgui_widgets.cpp
  vendor/imgui/backends/imgui_impl_glfw.cpp
  vendor/imgui/backends/imgui_impl_opengl3.cpp)
target_include_directories(pool_imgui PUBLIC vendor/imgui)
target_link_libraries(pool_imgui glfw ${CMAKE_DL_LIBS})

add_executable(pool_viewer include/external/glad/glad.c
  src/pool_viewer/main.cpp src/pool_viewer/renderer.cpp)
target_compile_features(pool_viewer PRIVATE cxx_std_17)
target_include_directories(pool_viewer PRIVATE include include/external ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(pool_viewer rclcpp geometry_msgs sensor_msgs std_msgs std_srvs visualization_msgs riptide_msgs2 tf2 tf2_ros)
target_link_libraries(pool_viewer pool_imgui glfw assimp yaml-cpp ${OpenCV_LIBS})

install(TARGETS pool_viewer
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ../config.yaml DESTINATION share/${PROJECT_NAME}/config RENAME simulation.yaml)
install(FILES ../scene_info.yaml DESTINATION share/${PROJECT_NAME}/config)
install(FILES ../c_simulator/config/talos_tasks.yaml DESTINATION share/${PROJECT_NAME}/config)
install(FILES vendor/imgui/LICENSE.txt DESTINATION share/${PROJECT_NAME}/licenses RENAME imgui-MIT.txt)

install(
  DIRECTORY launch textures shaders models config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(pool_detection_pose_test test/detection_pose.cpp)
  target_compile_features(pool_detection_pose_test PRIVATE cxx_std_17)
  target_compile_options(pool_detection_pose_test PRIVATE -UNDEBUG)
  target_include_directories(pool_detection_pose_test PRIVATE include include/external)
  ament_target_dependencies(pool_detection_pose_test rclcpp tf2 tf2_ros visualization_msgs)
  add_test(NAME pool_detection_pose COMMAND pool_detection_pose_test)
  add_executable(pool_tf_tree_test test/tf_tree.cpp)
  target_compile_features(pool_tf_tree_test PRIVATE cxx_std_17)
  target_compile_options(pool_tf_tree_test PRIVATE -UNDEBUG)
  target_include_directories(pool_tf_tree_test PRIVATE include)
  add_test(NAME pool_tf_tree COMMAND pool_tf_tree_test)
  add_executable(pool_frustum_test test/frustum.cpp)
  target_compile_features(pool_frustum_test PRIVATE cxx_std_17)
  target_include_directories(pool_frustum_test PRIVATE include include/external)
  add_test(NAME pool_frustum COMMAND pool_frustum_test)
  add_executable(pool_depth_noise_test test/depth_noise.cpp)
  target_compile_features(pool_depth_noise_test PRIVATE cxx_std_17)
  target_compile_options(pool_depth_noise_test PRIVATE -UNDEBUG)
  target_include_directories(pool_depth_noise_test PRIVATE include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(pool_depth_noise_test ${OpenCV_LIBS})
  add_test(NAME pool_depth_noise COMMAND pool_depth_noise_test)
  add_executable(pool_camera_geometry_test test/camera_geometry.cpp)
  target_compile_features(pool_camera_geometry_test PRIVATE cxx_std_17)
  target_include_directories(pool_camera_geometry_test PRIVATE include include/external ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(pool_camera_geometry_test yaml-cpp ${OpenCV_LIBS})
  add_test(NAME pool_camera_geometry COMMAND pool_camera_geometry_test
    ${CMAKE_CURRENT_SOURCE_DIR}/test)
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
