### ROS2 Commands and Notes

#### Create Workspace

```
mkdir ws
cd ws
colcon build 
source install/setup.bash
```

#### Create C++ Package

`ros2 pkg create my_pkg --build-type ament_cmake --dependencies rclcpp `

`colocon build or colcon build --packages-select my_pkg `

#### Colcon auto-complete

`source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash`

#### Vscode Includes

go to c/c++ configuration then c_cpp_properties.json and add this path in includes

` "/opt/ros/foxy/include/**",`

#### Make Node Executable

```

add_executable(cpp_node src/first_node.cpp)
ament_target_dependencies(cpp_node rclcpp) # to link the depndencies with the executable 

install(TARGETS  
  cpp_node 
  DESTINATION lib/${PROJECT_NAME}
)
```

#### Ros2 Action

`ros2 interface show pkg /acion/action name `

### Add Ros2 Action

#### In xml

```
 <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

```

#### In cmake

```

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"action/CountUntil.action"
""
)

ament_export_dependencies(rosidl_default_runtime)

```
