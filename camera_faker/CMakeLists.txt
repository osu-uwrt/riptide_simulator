cmake_minimum_required(VERSION 3.18)
project(camera_faker)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("$<$<COMPILE_LANGUAGE:C,CXX>:-Wall;-Wextra;-Wpedantic>")
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(chameleon_tf_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(glfw3 REQUIRED)
find_package(assimp REQUIRED)
find_package(OpenCV REQUIRED)

# CUDA is optional at build time and probed again at runtime. CPU-only hosts
# need neither the toolkit nor an NVIDIA driver.
option(POOL_ENABLE_NVJPEG "Use nvJPEG for compressed camera images when available" ON)
option(POOL_ENABLE_CUDA "Build CUDA camera processing when a toolkit is available" ON)
if(POOL_ENABLE_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    set(CMAKE_CUDA_RUNTIME_LIBRARY Static)
    find_package(CUDAToolkit REQUIRED)
  endif()
endif()

add_library(pool_camera_processing STATIC src/pool_viewer/camera_processor.cpp)
target_compile_features(pool_camera_processing PUBLIC cxx_std_17)
target_include_directories(pool_camera_processing PUBLIC include ${OpenCV_INCLUDE_DIRS} PRIVATE include/external)
target_link_libraries(pool_camera_processing PUBLIC ${OpenCV_LIBS} PRIVATE yaml-cpp)
if(POOL_ENABLE_CUDA AND CMAKE_CUDA_COMPILER)
  target_sources(pool_camera_processing PRIVATE src/pool_viewer/camera_cuda.cu)
  target_compile_definitions(pool_camera_processing PRIVATE POOL_HAS_CUDA=1)
  set_target_properties(pool_camera_processing PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON)
  message(STATUS "Camera processing: CUDA with CPU fallback")
  # Static linkage keeps CUDA-enabled executables usable on CPU-only machines.
  if(POOL_ENABLE_NVJPEG AND TARGET CUDA::nvjpeg_static AND TARGET CUDA::culibos)
    target_compile_definitions(pool_camera_processing PRIVATE POOL_HAS_NVJPEG=1)
    target_link_libraries(pool_camera_processing PRIVATE CUDA::nvjpeg_static CUDA::culibos)
    message(STATUS "Camera JPEG encoding: nvJPEG with CPU fallback")
  else()
    message(STATUS "Camera JPEG encoding: CPU (static nvJPEG unavailable or disabled)")
  endif()
else()
  message(STATUS "Camera processing: CPU (CUDA unavailable or disabled)")
endif()

set_source_files_properties(include/external/glad/glad.c PROPERTIES COMPILE_OPTIONS "-Wno-pedantic")

add_library(pool_imgui STATIC
  vendor/imgui/imgui.cpp vendor/imgui/imgui_draw.cpp
  vendor/imgui/imgui_tables.cpp vendor/imgui/imgui_widgets.cpp
  vendor/imgui/backends/imgui_impl_glfw.cpp
  vendor/imgui/backends/imgui_impl_opengl3.cpp)
target_include_directories(pool_imgui PUBLIC vendor/imgui)
target_link_libraries(pool_imgui glfw ${CMAKE_DL_LIBS})

# Presentation and contracts do not depend on ROS or custom robot messages.
add_library(pool_panel_core STATIC
  src/pool_viewer/panels/panel_registry.cpp src/pool_viewer/panels/composition.cpp
  src/pool_viewer/panels/motion_panel.cpp
  src/pool_viewer/panels/mapping_panel.cpp src/pool_viewer/panels/actuator_panel.cpp src/pool_viewer/panels/run_panel.cpp src/pool_viewer/panels/simulation_panel.cpp
  src/pool_viewer/panels/autonomy_panel.cpp src/pool_viewer/panels/pose_gizmo.cpp)
target_compile_features(pool_panel_core PUBLIC cxx_std_17)
target_include_directories(pool_panel_core PUBLIC include include/external)
target_link_libraries(pool_panel_core PUBLIC pool_imgui yaml-cpp)

add_library(pool_panel_ros STATIC
  src/pool_viewer/panels/ros_runtime.cpp src/pool_viewer/panels/uwrt_motion.cpp
  src/pool_viewer/panels/standard_motion.cpp src/pool_viewer/panels/uwrt_autonomy.cpp
  src/pool_viewer/panels/uwrt_mapping.cpp src/pool_viewer/panels/uwrt_actuators.cpp src/pool_viewer/panels/sim_run.cpp src/pool_viewer/panels/simulation_rate.cpp)
target_link_libraries(pool_panel_ros pool_panel_core)
ament_target_dependencies(pool_panel_ros rclcpp rclcpp_action action_msgs rcl_interfaces
  geometry_msgs std_msgs std_srvs riptide_msgs2 chameleon_tf_msgs visualization_msgs tf2 tf2_ros)

add_executable(pool_viewer include/external/glad/glad.c
  src/pool_viewer/main.cpp src/pool_viewer/renderer.cpp)
target_compile_features(pool_viewer PRIVATE cxx_std_17)
target_include_directories(pool_viewer PRIVATE include include/external ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(pool_viewer rclcpp geometry_msgs sensor_msgs std_msgs std_srvs visualization_msgs riptide_msgs2 tf2 tf2_ros)
target_link_libraries(pool_viewer pool_camera_processing pool_panel_ros pool_imgui glfw assimp yaml-cpp ${OpenCV_LIBS})

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
  add_executable(pool_gizmo_input_test test/gizmo_input.cpp)
  target_link_libraries(pool_gizmo_input_test pool_panel_core)
  target_compile_options(pool_gizmo_input_test PRIVATE -UNDEBUG)
  add_test(NAME pool_gizmo_input COMMAND pool_gizmo_input_test)
  add_executable(pool_viewer_input_test test/viewer_input.cpp)
  target_compile_features(pool_viewer_input_test PRIVATE cxx_std_17)
  target_compile_options(pool_viewer_input_test PRIVATE -UNDEBUG)
  target_include_directories(pool_viewer_input_test PRIVATE include include/external)
  add_test(NAME pool_viewer_input COMMAND pool_viewer_input_test)
  add_executable(pool_panel_composition_test test/panel_composition.cpp)
  target_link_libraries(pool_panel_composition_test pool_panel_core)
  target_compile_options(pool_panel_composition_test PRIVATE -UNDEBUG)
  add_test(NAME pool_panel_composition COMMAND pool_panel_composition_test)
  add_executable(pool_panel_ros_test test/panel_ros.cpp)
  target_link_libraries(pool_panel_ros_test pool_panel_ros)
  target_compile_options(pool_panel_ros_test PRIVATE -UNDEBUG)
  add_test(NAME pool_panel_ros COMMAND pool_panel_ros_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../c_simulator/robots/talos/config/viewer.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/config/standard_panels.example.yaml)
  set_tests_properties(pool_panel_ros PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
  add_executable(pool_operator_panels_test test/operator_panels.cpp)
  target_link_libraries(pool_operator_panels_test pool_panel_ros)
  target_compile_options(pool_operator_panels_test PRIVATE -UNDEBUG)
  add_test(NAME pool_operator_panels COMMAND pool_operator_panels_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../c_simulator/robots/talos/config/viewer.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/config/viewer_tools.yaml)
  set_tests_properties(pool_operator_panels PROPERTIES TIMEOUT 45 RUN_SERIAL TRUE)
  add_executable(pool_thruster_visuals_test test/thruster_visuals.cpp)
  target_compile_features(pool_thruster_visuals_test PRIVATE cxx_std_17)
  target_compile_options(pool_thruster_visuals_test PRIVATE -UNDEBUG)
  target_include_directories(pool_thruster_visuals_test PRIVATE include include/external ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(pool_thruster_visuals_test yaml-cpp ${OpenCV_LIBS})
  add_test(NAME pool_thruster_visuals COMMAND pool_thruster_visuals_test
    ${CMAKE_CURRENT_SOURCE_DIR}/models/talos3/thrusters.yaml)
  add_executable(pool_status_lights_test test/status_lights.cpp)
  target_compile_features(pool_status_lights_test PRIVATE cxx_std_17)
  target_compile_options(pool_status_lights_test PRIVATE -UNDEBUG)
  target_include_directories(pool_status_lights_test PRIVATE include include/external ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(pool_status_lights_test yaml-cpp ${OpenCV_LIBS})
  add_test(NAME pool_status_lights COMMAND pool_status_lights_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../c_simulator/robots/talos/config/status_lights.yaml)
  add_executable(pool_camera_processor_test test/camera_processor.cpp)
  target_link_libraries(pool_camera_processor_test pool_camera_processing)
  target_compile_options(pool_camera_processor_test PRIVATE -UNDEBUG)
  target_include_directories(pool_camera_processor_test PRIVATE include/external)
  add_test(NAME pool_camera_processor COMMAND pool_camera_processor_test)
  add_test(NAME pool_camera_cuda COMMAND pool_camera_processor_test --cuda)
  set_tests_properties(pool_camera_cuda PROPERTIES SKIP_RETURN_CODE 77)
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
