cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)



# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cpp_node src/first_node.cpp)
ament_target_dependencies(cpp_node rclcpp) # to link the depndencies with the executable 

install(TARGETS  
  cpp_node 
  DESTINATION lib/${PROJECT_NAME} #this lib wilfolder in the ws l be inside install 
)
ament_package()
