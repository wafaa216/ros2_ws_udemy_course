### Create Workspace

1. mkdir ws
2. cd ws
3. colcon build
4. cd install && source setup.bash

### Create c++ package

`ros2 pkg create my_pkg --build-type ament_cmake --dependencies rclcpp `

`colocon build or colcon build --packages-select my_pkg `

#### Colcon auto-complete

`source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash`

#### vscode includes

go to c/c++ configuration then c_cpp_properties.json and add this path in includes

` "/opt/ros/foxy/include/**",`

#### MAKE NODE EXECUTABLE

```

add_executable(cpp_node src/first_node.cpp)
ament_target_dependencies(cpp_node rclcpp) # to link the depndencies with the executable 

install(TARGETS  
  cpp_node 
  DESTINATION lib/${PROJECT_NAME}
)
```

#### ros2 action

`ros2 interface show pkg /acion/action name `

### Add ros2 action

#### in xml

```
 <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

```

#### in cmake

```

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"action/CountUntil.action"
""
)

ament_export_dependencies(rosidl_default_runtime)

```
