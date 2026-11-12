cmake_minimum_required(VERSION 3.22)
project(moveit_config_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(franka_description REQUIRED)   # provides action/PickAndPlace
find_package(linkattacher_msgs REQUIRED)    # only if your C++ code uses it
# find_package(nlohmann_json REQUIRED)      # uncomment if the node parses the JSON with it

# ---- C++ pick and place action server ----
add_executable(pick_and_place_node src/pick_place_gazebo.cpp)
ament_target_dependencies(pick_and_place_node
  rclcpp rclcpp_action moveit_ros_planning_interface
  geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs
  franka_description linkattacher_msgs)
# target_link_libraries(pick_and_place_node nlohmann_json::nlohmann_json)

# Optional: action client (has its own main(), so it needs its own executable)
add_executable(pick_and_place_client src/pick_and_place_client.cpp)
ament_target_dependencies(pick_and_place_client
  rclcpp rclcpp_action franka_description)

install(TARGETS
  pick_and_place_node
  pick_and_place_client
  DESTINATION lib/${PROJECT_NAME})

# ---- Python nodes (must be executable and start with #!/usr/bin/env python3) ----
install(PROGRAMS
  src/gz_link_attacher.py
  src/testmodel.py
  src/pice_spawn.py
  src/chess_engine.py
  src/chess_managment.py
  DESTINATION lib/${PROJECT_NAME})

# ---- Shared files ----
foreach(dir launch config models world)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()   # always last