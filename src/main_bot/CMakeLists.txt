cmake_minimum_required(VERSION 3.8)
project(main_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── C++ nodes ─────────────────────────────────────────────────────────────────
add_executable(ackermann_transform_node
  src/ackermann_transform_node.cpp
)
ament_target_dependencies(ackermann_transform_node
  rclcpp
  geometry_msgs
)
install(TARGETS ackermann_transform_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Launch / config / assets ───────────────────────────────────────────────────
install(
  DIRECTORY launch config description worlds maps
  DESTINATION share/${PROJECT_NAME}
)

# ── Python scripts (giữ odom_tf_relay) ────────────────────────────────────────
install(
  PROGRAMS
    scripts/odom_tf_relay.py
    scripts/dock_to_charge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
