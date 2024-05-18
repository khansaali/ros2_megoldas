ros2_ws$ colcon build --packages-select my_cpp_pkg
Starting >>> my_cpp_pkg
--- stderr: my_cpp_pkg                             
/usr/bin/ld: /usr/lib/gcc/x86_64-linux-gnu/11/../../../x86_64-linux-gnu/Scrt1.o: in function `_start':
(.text+0x1b): undefined reference to `main'
collect2: error: ld returned 1 exit status
gmake[2]: *** [CMakeFiles/cpp_node.dir/build.make:151: cpp_node] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:137: CMakeFiles/cpp_node.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< my_cpp_pkg [0.89s, exited with code 2]

Summary: 0 packages finished [1.68s]
  1 package failed: my_cpp_pkg
  1 package had stderr output: my_cpp_pkg

cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

#default to c++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)



add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

install(TARGETS 
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
