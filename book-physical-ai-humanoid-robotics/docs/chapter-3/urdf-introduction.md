---
sidebar_position: 3
---

# URDF Introduction: Robot Modeling for Humanoid Robots

## Introduction

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. For humanoid robots, which have complex kinematic structures with many degrees of freedom, URDF provides a standardized way to represent the robot's physical structure, joints, and sensors. Understanding URDF is essential for simulating, visualizing, and controlling humanoid robots in ROS-based systems.

## What is URDF?

### Definition and Purpose
URDF stands for Unified Robot Description Format. It is an XML format that describes:
- **Physical structure**: Links (rigid bodies) and their properties
- **Kinematic structure**: Joints connecting links
- **Visual appearance**: How the robot looks in visualization tools
- **Collision properties**: How the robot interacts with the environment
- **Sensors and actuators**: Where sensors are mounted on the robot

### Role in ROS Ecosystem
URDF serves as the foundation for:
- **Robot State Publishing**: Converting joint states to transforms
- **Visualization**: Displaying the robot in RViz and other tools
- **Simulation**: Creating physics models in Gazebo and other simulators
- **Motion Planning**: Understanding robot kinematics for path planning
- **Control**: Providing kinematic models for controllers

## URDF Structure for Humanoid Robots

### Basic Elements
A humanoid robot URDF consists of:

**Links**: Represent rigid bodies (torso, limbs, head)
```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
</link>
```

**Joints**: Connect links with specific degrees of freedom
```xml
<joint name="torso_to_head" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
```

### Humanoid Robot Kinematic Structure
Humanoid robots typically follow a tree-like structure:
```
base_link (usually pelvis/torso)
├── left_leg
│   ├── left_upper_leg
│   ├── left_lower_leg
│   └── left_foot
├── right_leg
│   ├── right_upper_leg
│   ├── right_lower_leg
│   └── right_foot
├── torso
│   ├── left_arm
│   │   ├── left_upper_arm
│   │   ├── left_lower_arm
│   │   └── left_hand
│   ├── right_arm
│   │   ├── right_upper_arm
│   │   ├── right_lower_arm
│   │   └── right_hand
│   └── head
└── sensors (mounted on various links)
```

## URDF Components for Humanoid Robots

### Link Definition
Each link represents a rigid body part of the robot:

```xml
<link name="left_upper_arm">
  <!-- Visual properties for display -->
  <visual>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <!-- Collision properties for physics simulation -->
  <collision>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </collision>

  <!-- Inertial properties for dynamics -->
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0"
             iyy="0.01" iyz="0.0" izz="0.005"/>
  </inertial>
</link>
```

### Joint Definition
Joints define how links connect and move relative to each other:

```xml
<joint name="left_shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.1 -0.1 0.3" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-2.0" upper="1.0" effort="50" velocity="2.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Joint Types for Humanoid Robots
Different joint types serve different purposes:

- **Revolute**: Rotational joint with one degree of freedom (most common in humanoid robots)
- **Continuous**: Like revolute but unlimited rotation (rarely used in humanoid robots)
- **Prismatic**: Linear sliding joint (rarely used in humanoid robots)
- **Fixed**: No movement, rigid connection (used for mounting sensors)

## Complete Humanoid URDF Example

Here's a simplified example of a humanoid robot URDF:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0"
               iyy="0.2" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <origin xyz="0 0 0.25"/>
      <inertia ixx="0.3" ixy="0.0" ixz="0.0"
               iyy="0.3" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.05"/>
  </joint>

  <!-- Left leg -->
  <link name="left_upper_leg">
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="-0.1 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <!-- Additional links and joints would continue similarly -->

  <!-- Sensors -->
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
               iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.4"/>
  </joint>
</robot>
```

## URDF Tools and Visualization

### Robot State Publisher
The robot_state_publisher node converts joint states to transforms:
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat robot.urdf)"
```

### Visualization in RViz
URDF models can be visualized in RViz by adding a RobotModel display and setting the Robot Description parameter.

### URDF Validation
Validate URDF files using:
```bash
check_urdf robot.urdf
```

## Best Practices for Humanoid URDF

### Naming Conventions
- Use consistent, descriptive names
- Follow the pattern: `side_component` (e.g., `left_upper_arm`, `right_foot`)
- Use lowercase with underscores

### Physical Accuracy
- Ensure inertial properties are realistic
- Use appropriate collision geometries
- Consider the actual physical dimensions

### Joint Limits
- Set realistic joint limits based on hardware capabilities
- Include effort and velocity limits for safety
- Consider biological joint ranges for humanoid robots

### Modularity
- Organize URDF in separate files for different parts
- Use xacro for parameterization and macros
- Include proper documentation

## Integration with Control Systems

### Kinematic Chains
URDF defines kinematic chains that are used by:
- Forward kinematics solvers
- Inverse kinematics solvers
- Motion planning algorithms
- Visualization tools

### Sensor Integration
Sensors are mounted on URDF links and referenced by other ROS nodes:
- IMU sensors for balance control
- Force/torque sensors in joints
- Cameras for perception
- Joint encoders for state estimation

## Advanced URDF Features

### Xacro for Complex Models
Xacro allows parameterization and macros:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  <xacro:property name="pi" value="3.14159"/>

  <xacro:macro name="arm_segment" params="name length mass">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="${length}"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
      </inertial>
    </link>
  </xacro:macro>

  <xacro:arm_segment name="left_upper_arm" length="0.3" mass="1.5"/>
</robot>
```

### Transmission Elements
Define how actuators connect to joints:
```xml
<transmission name="left_hip_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_hip">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_hip_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

URDF provides the essential foundation for representing humanoid robots in ROS systems. Understanding URDF is crucial for simulation, visualization, motion planning, and control of complex humanoid robots. The standardized format allows different tools and algorithms to work with robot models consistently, enabling the development of sophisticated robotic applications.