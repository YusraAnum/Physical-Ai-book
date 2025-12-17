---
sidebar_position: 1
---

# Nodes, Topics, and Services: Core Communication Primitives

## Introduction

ROS 2 provides three fundamental communication primitives that enable different components of a humanoid robot to interact: Nodes, Topics, and Services (Open Robotics, 2023). Understanding these primitives is essential for designing effective robotic systems, as they determine how information flows between different components and how coordination is achieved.

## Nodes: The Foundation of ROS 2 Architecture

### Definition
A node is a process that performs computation in a ROS 2 system. Nodes are the basic execution units that contain the robot's functionality. Each node typically performs a specific task and communicates with other nodes through topics, services, or actions.

### Characteristics of Nodes
- **Process Isolation**: Each node runs in its own process, providing fault isolation
- **Single Responsibility**: Each node should have a well-defined, specific purpose
- **Communication Interface**: Nodes expose interfaces for communication with other nodes
- **Lifecycle Management**: Nodes can be started, stopped, and restarted independently

### Node Implementation
In practice, a node is implemented as a class that inherits from `rclcpp::Node` (C++) or `rclpy.node.Node` (Python). The node class provides access to ROS 2 functionality such as creating publishers, subscribers, services, and clients.

### Example Node Responsibilities in Humanoid Robots
- **Sensor Driver Nodes**: Interface with hardware sensors and publish sensor data
- **Controller Nodes**: Implement control algorithms and publish commands
- **Perception Nodes**: Process sensor data to extract meaningful information
- **Planning Nodes**: Generate motion plans and trajectories
- **State Estimation Nodes**: Combine sensor data to estimate robot state

## Topics: Asynchronous Data Streaming

### Definition
Topics enable asynchronous, many-to-many communication through a publish-subscribe pattern (Object Management Group, 2020). Publishers send messages to topics, and subscribers receive messages from topics. Multiple publishers can publish to the same topic, and multiple subscribers can subscribe to the same topic.

### Characteristics of Topics
- **Asynchronous**: Publishers and subscribers don't need to be synchronized
- **Many-to-Many**: Multiple publishers and subscribers can use the same topic
- **Data Streaming**: Designed for continuous data streams (e.g., sensor data)
- **Fire-and-Forget**: Publishers don't wait for acknowledgment from subscribers

### Topic Communication Pattern
```
Publisher(s) → Topic → Subscriber(s)
```

### Quality of Service (QoS) in Topics
Topics support various QoS settings that control delivery guarantees:
- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local (for late-joining subscribers)
- **History**: Keep all messages vs. keep last N messages
- **Deadline**: Maximum time between messages

### Common Topic Uses in Humanoid Robots
- **Sensor Data**: IMU readings, camera images, joint states
- **Control Commands**: Joint position/velocity/effort commands
- **Robot State**: Current pose, joint angles, system status
- **Perception Results**: Detected objects, recognized features

## Services: Synchronous Request-Response

### Definition
Services enable synchronous, request-response communication between nodes. A service client sends a request to a service server, which processes the request and sends back a response. This pattern is similar to remote procedure calls (RPC).

### Characteristics of Services
- **Synchronous**: Client waits for response before continuing
- **One-to-One**: One client communicates with one server at a time
- **Request-Response**: Client sends request, server sends response
- **Blocking**: Client operation blocks until response is received

### Service Communication Pattern
```
Client → Request → Server
Client ← Response ← Server
```

### Service Implementation
Services are defined using service definition files (.srv) that specify the request and response message types. Both client and server must use the same service definition.

### Common Service Uses in Humanoid Robots
- **Calibration**: Request sensor calibration and receive confirmation
- **Configuration**: Set parameters and receive status
- **Emergency Operations**: Request emergency stop and confirm execution
- **State Queries**: Request current robot state information
- **File Operations**: Load configuration files or save data

## Comparison of Communication Primitives

| Aspect | Topics | Services |
|--------|--------|----------|
| **Pattern** | Publish-Subscribe | Request-Response |
| **Synchronization** | Asynchronous | Synchronous |
| **Communication Type** | One-to-many, many-to-many | One-to-one |
| **Latency** | Lower (non-blocking) | Higher (blocking) |
| **Use Case** | Continuous data streams | Discrete operations |
| **Reliability** | Best effort or reliable | Reliable by design |
| **Error Handling** | Message loss possible | Explicit error responses |

## Practical Examples in Humanoid Robotics

### Topic Example: Joint State Publishing
```
Joint State Publisher Node:
- Reads joint encoders
- Publishes joint angles, velocities, efforts to /joint_states topic

Multiple Subscriber Nodes:
- State estimation node: Uses joint states for robot state
- Visualization node: Updates robot model display
- Safety node: Monitors joint limits
```

### Service Example: Robot Calibration
```
Calibration Service Server Node:
- Receives calibration request
- Performs sensor calibration procedure
- Returns success/failure response

Calibration Client Node:
- Sends calibration request
- Waits for response
- Proceeds based on calibration result
```

## Design Principles for Communication

### When to Use Topics
- Streaming continuous data (sensors, robot state)
- Broadcasting information to multiple recipients
- When real-time performance is critical
- When message loss is acceptable

### When to Use Services
- Discrete operations that require confirmation
- Configuration and setup operations
- When reliable delivery and response are required
- When the operation has a clear start and end

### Hybrid Approaches
Many robotic applications combine both primitives:
- Use services for setup and configuration
- Use topics for ongoing data exchange
- Use services for commands that require acknowledgment
- Use topics for continuous monitoring

## Advanced Communication Concepts

### Actions
While not one of the three core primitives, actions are important for long-running operations that require feedback and goal management. Actions combine features of topics and services for operations like navigation that take time and provide continuous feedback.

### Parameters
Parameters provide a way to configure nodes with static or slowly-changing values. Parameters are stored in a central parameter server and can be accessed by all nodes.

## Best Practices

### Node Design
- Keep nodes focused on single responsibilities
- Use descriptive names that indicate the node's function
- Handle errors gracefully and provide appropriate logging
- Implement proper shutdown procedures

### Topic Design
- Use consistent naming conventions
- Choose appropriate QoS settings for your use case
- Consider bandwidth and processing requirements
- Design message types that are efficient and extensible

### Service Design
- Define clear request and response semantics
- Handle errors and edge cases explicitly
- Consider timeout values for service calls
- Document expected behavior and potential failures

Understanding these core communication primitives is fundamental to designing effective humanoid robot systems that can scale and maintain modularity while achieving the coordination required for complex robotic behaviors.