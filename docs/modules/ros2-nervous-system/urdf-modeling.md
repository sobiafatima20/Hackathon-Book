# URDF Robot Modeling

## Overview

Unified Robot Description Format (URDF) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including links, joints, and their relationships.

## URDF Structure

A URDF file consists of:
- **Links**: Rigid bodies that make up the robot
- **Joints**: Connections between links that allow relative motion
- **Visual**: Visual representation for display and simulation
- **Collision**: Collision properties for physics simulation
- **Inertial**: Mass, center of mass, and inertia properties

## Basic URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head link connected to base -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
</robot>
```

## Link Elements

Links represent rigid bodies in the robot. Each link can have:

- **Visual**: How the link appears in visualization tools
- **Collision**: How the link behaves in collision detection
- **Inertial**: Physical properties for dynamics simulation

### Visual Properties
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
    <!-- Other options: <sphere radius="0.5"/>, <cylinder radius="0.1" length="0.5"/> -->
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

### Collision Properties
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
  </geometry>
</collision>
```

### Inertial Properties
```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
</inertial>
```

## Joint Types

### Fixed Joint
No movement between parent and child links.

### Revolute Joint
Rotational movement around a single axis.

### Continuous Joint
Like revolute but unlimited rotation.

### Prismatic Joint
Linear sliding movement.

### Floating Joint
6 degrees of freedom (3 translation, 3 rotation).

### Planar Joint
Movement on a plane.

## Complex Humanoid Example

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="2"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0.1 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Additional joints and links for arms, legs, etc. would follow -->
</robot>
```

## URDF Processing Algorithm

```
ALGORITHM: URDF_Parser
INPUT: urdf_file_path
OUTPUT: robot_model

BEGIN
    urdf_content = read_file(urdf_file_path)
    robot_element = parse_xml(urdf_content)

    robot_name = get_attribute(robot_element, "name")
    links = extract_links(robot_element)
    joints = extract_joints(robot_element)

    robot_model = create_robot_model(robot_name, links, joints)
    return robot_model
END
```

## Xacro for Complex Models

Xacro (XML Macros) allows you to create more maintainable URDF files by using variables, macros, and includes:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="base_size" value="0.5 0.5 0.5" />

  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_size}"/>
      </geometry>
    </visual>
  </link>

  <xacro:macro name="simple_joint" params="name type parent child origin_xyz axis_xyz">
    <joint name="${name}" type="${type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${origin_xyz}"/>
      <axis xyz="${axis_xyz}"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    </joint>
  </xacro:macro>
</robot>
```

## Validation and Tools

### URDF Validation
Use ROS tools to validate your URDF:
- `check_urdf`: Validates URDF syntax
- `urdf_to_graphiz`: Creates visual graph of robot structure
- `rviz`: Visualizes the robot model

### Common Issues
- Missing inertial properties for physics simulation
- Invalid joint limits or ranges
- Incorrect parent-child relationships
- Units inconsistency (URDF expects meters, radians, kilograms)

## Next Steps

Continue to [AI Integration with ROS 2](./ai-integration.md) to learn about connecting AI agents with ROS 2.