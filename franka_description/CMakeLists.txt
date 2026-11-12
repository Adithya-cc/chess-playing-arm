cmake_minimum_required(VERSION 3.5)
project(franka_description)
find_package(ament_cmake REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_urdf_tests test/urdf_tests.py)

endif()
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickAndPlace.action"
  DEPENDENCIES geometry_msgs
)



# Install launch files.
install(DIRECTORY launch meshes robots rviz test
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
