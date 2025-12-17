---
sidebar_position: 3
---

# Architecture Comparison: Monolithic vs Nodes-Based Systems

## Introduction

When designing control systems for humanoid robots, engineers face a fundamental architectural choice: whether to implement a monolithic system where all functionality runs in a single process, or a nodes-based system where functionality is distributed across multiple communicating processes (Siciliano & Khatib, 2016). This chapter compares these two approaches, highlighting the advantages of the nodes-based architecture that ROS 2 promotes.

## Monolithic Architecture

### Definition
A monolithic architecture implements all robot functionality within a single, unified program. All sensors, controllers, perception algorithms, and planning modules run within one process, sharing memory space and communicating through function calls or shared variables.

### Characteristics
- **Single Process**: All functionality runs in one executable
- **Shared Memory**: Components communicate through direct memory access
- **Tight Coupling**: Components are highly interdependent
- **Centralized Control**: One main loop coordinates all activities

### Advantages
1. **Simplicity**: Easier to design and implement initially
2. **Performance**: Direct function calls can be faster than message passing
3. **Debugging**: Single process is easier to debug with traditional tools
4. **Memory Efficiency**: No overhead from inter-process communication
5. **Predictability**: Deterministic execution patterns

### Disadvantages
1. **Scalability**: Adding new functionality becomes increasingly complex
2. **Maintainability**: Changes in one component can affect the entire system
3. **Fault Propagation**: A failure in one component can crash the entire system
4. **Team Development**: Difficult for multiple developers to work simultaneously
5. **Technology Lock-in**: Hard to integrate components written in different languages
6. **Testing**: Components cannot be easily tested in isolation
7. **Deployment**: Cannot distribute computation across multiple processors easily

## Nodes-Based Architecture

### Definition
A nodes-based architecture decomposes the robot system into multiple independent processes (nodes) that communicate through standardized message passing (Quigley et al., 2009). Each node performs a specific function and can be developed, tested, and maintained independently.

### Characteristics
- **Multiple Processes**: Each component runs in its own process
- **Message Passing**: Components communicate through published messages and services
- **Loose Coupling**: Components are largely independent
- **Distributed Control**: Each node manages its own execution

### Advantages
1. **Modularity**: Components can be developed and maintained independently
2. **Scalability**: New components can be added without affecting existing ones
3. **Fault Isolation**: A failure in one node doesn't necessarily affect others
4. **Technology Diversity**: Different nodes can use different languages and frameworks
5. **Team Development**: Multiple teams can work on different nodes simultaneously
6. **Testing**: Individual nodes can be tested in isolation
7. **Deployment Flexibility**: Nodes can run on different hardware platforms
8. **Reusability**: Nodes can be reused across different robot projects
9. **Maintenance**: Components can be updated without stopping the entire system

### Disadvantages
1. **Complexity**: More complex initial setup and configuration
2. **Communication Overhead**: Message passing has latency and bandwidth costs
3. **Distributed Debugging**: More challenging to debug across multiple processes
4. **Synchronization**: Coordinating timing between nodes requires careful design
5. **Resource Usage**: Multiple processes may use more system resources

## Comparative Analysis for Humanoid Robots

### Complexity Management
Humanoid robots inherently have high complexity due to numerous degrees of freedom and sensors. The nodes-based approach helps manage this complexity by breaking it into manageable pieces:

- **Monolithic**: Complexity grows quadratically as new components are added
- **Nodes-Based**: Complexity grows linearly as each node manages its own complexity

### Safety Considerations
For humanoid robots operating near humans, safety is paramount:

- **Monolithic**: A bug in any component can potentially affect safety-critical functions
- **Nodes-Based**: Safety-critical nodes can be isolated and monitored independently

### Real-time Performance
Both architectures can achieve real-time performance, but with different approaches:

- **Monolithic**: Easier to guarantee timing through centralized scheduling
- **Nodes-Based**: Requires careful QoS configuration and potentially real-time middleware

### Development Team Coordination
Humanoid robot development typically involves multiple specialists:

- **Monolithic**: Coordination required to avoid conflicts in shared code
- **Nodes-Based**: Teams can work independently as long as interface contracts are maintained

## Practical Examples

### Monolithic Example (Pseudocode)
```
// All functionality in one program
int main() {
    initialize_all_sensors();
    initialize_all_controllers();
    initialize_perception();
    initialize_planning();

    while(robot_running) {
        read_sensor_data();
        process_perception();
        run_planning();
        run_control();
        send_commands();
    }
}
```

### Nodes-Based Example (ROS 2)
```
// Separate nodes communicate through ROS 2
// Sensor Node: reads hardware and publishes data
// Controller Node: subscribes to sensor data and publishes commands
// Perception Node: processes sensor data and publishes results
// Planning Node: uses perception results to generate plans
```

## Transition Strategies

Many projects transition from monolithic to nodes-based architectures:

1. **Incremental Migration**: Gradually extract components from monolithic system
2. **Parallel Development**: Maintain monolithic version while developing nodes-based
3. **Hybrid Approach**: Keep safety-critical components monolithic, distribute others

## When to Choose Each Architecture

### Monolithic Architecture is Suitable When:
- Robot system is simple with few components
- Performance requirements are extremely stringent
- Development team is small and highly coordinated
- Project timeline is very short
- Safety requirements are simple

### Nodes-Based Architecture is Suitable When:
- Robot system is complex (like humanoid robots)
- Multiple teams are involved in development
- Long-term maintenance and evolution are important
- Components need to run on different hardware
- Reusability across projects is desired
- Fault tolerance is important

## ROS 2 Facilitation

ROS 2 makes nodes-based architecture practical by providing:

- **Standardized Communication**: Common interfaces for message passing
- **Tooling**: Visualization, debugging, and monitoring tools
- **Language Support**: Multiple programming languages can interoperate
- **Deployment Tools**: Easy launching and management of multiple nodes
- **Quality of Service**: Configurable communication reliability

For humanoid robots, the complexity and long-term benefits of nodes-based architecture typically outweigh the initial complexity, making ROS 2's approach the preferred solution for most advanced robotic systems.