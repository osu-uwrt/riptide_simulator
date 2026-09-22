cmake_minimum_required(VERSION 3.8)
project(c_simulator)

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
find_package(nav_msgs REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_localization REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(marine_dynamics SHARED src/marine_dynamics.cpp src/thruster_dynamics.cpp)
target_compile_features(marine_dynamics PUBLIC cxx_std_17)
target_include_directories(marine_dynamics PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(marine_dynamics Eigen3)
add_library(task_contacts src/task_contacts.cpp)
target_compile_features(task_contacts PUBLIC cxx_std_17)
target_include_directories(task_contacts PUBLIC include ${BULLET_INCLUDE_DIRS})
target_link_libraries(task_contacts ${BULLET_LIBRARIES} yaml-cpp Eigen3::Eigen)

add_executable(physics_simulator src/physics_simulator.cpp src/robot_class.cpp
  src/collisionBox_class.cpp)
target_compile_features(physics_simulator PRIVATE cxx_std_17)

target_include_directories(physics_simulator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${YAML_CPP_INCLUDEDIR})

ament_target_dependencies(physics_simulator
  Eigen3
  rclcpp
  rosgraph_msgs
  std_srvs
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
  yaml-cpp)

target_link_libraries(physics_simulator
  ${YAML_CPP_LIBRARIES} marine_dynamics task_contacts)

install(TARGETS physics_simulator
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS marine_dynamics EXPORT marine_dynamics_targets LIBRARY DESTINATION lib)
install(DIRECTORY include/c_simulator DESTINATION include)
ament_export_targets(marine_dynamics_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
install(DIRECTORY launch collision_files config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES ../config.yaml DESTINATION share/${PROJECT_NAME}/config RENAME simulation.yaml)
install(FILES ../scene_info.yaml DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/task_simulator.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/payload_model.py scripts/claw_world.py scripts/magnet_light_model.py scripts/run_score.py DESTINATION lib/${PROJECT_NAME})
install(FILES requirements.txt DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_task_contacts test/test_task_contacts.cpp)
  target_link_libraries(test_task_contacts task_contacts)
  target_compile_definitions(test_task_contacts PRIVATE TASK_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_gtest(test_marine_dynamics test/test_marine_dynamics.cpp)
  target_link_libraries(test_marine_dynamics marine_dynamics)
  target_include_directories(test_marine_dynamics PUBLIC include)
  ament_target_dependencies(test_marine_dynamics Eigen3)
  ament_add_gtest(test_collision_box test/test_collision_box.cpp src/collisionBox_class.cpp)
  target_compile_features(test_collision_box PRIVATE cxx_std_17)
  target_include_directories(test_collision_box PRIVATE include)
  ament_target_dependencies(test_collision_box Eigen3)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME hydro_prior COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_hydro_prior.py)
  add_test(NAME payload_model COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_payload_model.py)
  add_test(NAME magnet_lights COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_magnet_lights.py)
  add_test(NAME run_score COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_run_score.py)
  add_test(NAME claw_contacts COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_claw_world.py)

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
