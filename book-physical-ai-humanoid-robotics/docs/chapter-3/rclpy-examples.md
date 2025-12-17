---
sidebar_position: 2
---

# rclpy Examples: Minimal Implementation for Humanoid Robotics

## Introduction

This chapter provides practical, minimal examples of `rclpy` implementations specifically designed for humanoid robotics applications. Each example demonstrates a specific pattern or concept that can be adapted and extended for more complex robotic systems. The examples follow best practices and are structured to be easily understood and modified.

## Basic Node Example

Let's start with the most fundamental ROS 2 Python node:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publisher Example: Joint Command Node

A simple node that publishes joint commands for humanoid robot control:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Time
import time

class JointCommandNode(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')

        # Create publisher for joint commands
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        # Create a timer to publish commands at regular intervals
        self.timer = self.create_timer(0.1, self.publish_joint_commands)  # 10Hz
        self.joint_positions = [0.0] * 28  # Example: 28 DOF humanoid
        self.get_logger().info('Joint command publisher started')

    def publish_joint_commands(self):
        msg = Float64MultiArray()

        # Update joint positions (example: simple sine wave pattern)
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = 0.5 * 3.14159 * (i % 3) / 3  # Example pattern

        msg.data = self.joint_positions
        self.publisher.publish(msg)
        self.get_logger().debug(f'Published joint commands: {msg.data[:3]}...')

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down joint command publisher')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscriber Example: Sensor Data Processor

A node that subscribes to sensor data and processes it:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publish processed data
        self.publisher = self.create_publisher(
            Float64,
            '/average_joint_velocity',
            10
        )

        self.get_logger().info('Sensor processor started')

    def joint_state_callback(self, msg):
        if len(msg.velocity) > 0:
            # Calculate average velocity
            avg_velocity = sum(msg.velocity) / len(msg.velocity)

            # Publish result
            result_msg = Float64()
            result_msg.data = avg_velocity
            self.publisher.publish(result_msg)

            self.get_logger().info(f'Average velocity: {avg_velocity:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down sensor processor')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Example: Robot Control Service

A service that allows other nodes to control robot behavior:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from std_msgs.msg import String

class RobotControlService(Node):
    def __init__(self):
        super().__init__('robot_control_service')

        # Create service
        self.srv = self.create_service(
            Trigger,
            'robot_control',
            self.control_callback
        )

        # Publisher for status updates
        self.status_publisher = self.create_publisher(
            String,
            '/robot_status',
            10
        )

        self.robot_state = 'IDLE'
        self.get_logger().info('Robot control service started')

    def control_callback(self, request, response):
        self.get_logger().info(f'Received control request: {request}')

        # Process control command
        if self.robot_state == 'IDLE':
            self.robot_state = 'ACTIVE'
            response.success = True
            response.message = 'Robot activated successfully'
        else:
            response.success = False
            response.message = f'Cannot activate, robot is {self.robot_state}'

        # Publish status update
        status_msg = String()
        status_msg.data = self.robot_state
        self.status_publisher.publish(status_msg)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down robot control service')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Action Example: Trajectory Execution

An action server for executing complex trajectories:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.task import Future
from example_interfaces.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import time

class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')

        # Create action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            execute_callback=self.execute_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Subscribe to current joint states
        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_controller/state',
            self.state_callback,
            10
        )

        self.current_state = None
        self.get_logger().info('Trajectory executor started')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received trajectory goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received trajectory cancel request')
        return CancelResponse.ACCEPT

    def state_callback(self, msg):
        self.current_state = msg

    def execute_trajectory(self, goal_handle):
        self.get_logger().info('Executing trajectory...')

        feedback_msg = FollowJointTrajectory.Feedback()
        result_msg = FollowJointTrajectory.Result()

        trajectory = goal_handle.request.trajectory
        total_points = len(trajectory.points)

        for i, point in enumerate(trajectory.points):
            # Publish feedback
            feedback_msg.joint_names = trajectory.joint_names
            feedback_msg.actual.positions = point.positions
            feedback_msg.desired.positions = point.positions
            feedback_msg.error.positions = [0.0] * len(point.positions)

            goal_handle.publish_feedback(feedback_msg)

            # Simulate trajectory execution
            time.sleep(0.01)  # Simulate execution time

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.error_code = -1
                result_msg.error_string = 'Goal canceled'
                return result_msg

        # Complete successfully
        goal_handle.succeed()
        result_msg.error_code = 0
        result_msg.error_string = 'Trajectory completed successfully'

        return result_msg

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down trajectory executor')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Parameter Example: Configurable Robot Node

A node that uses parameters for configuration:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.msg import ParameterDescriptor

class ConfigurableRobotNode(Node):
    def __init__(self):
        super().__init__('configurable_robot_node')

        # Declare parameters with default values and descriptions
        self.declare_parameter(
            'control_frequency',
            50,
            ParameterDescriptor(description='Control loop frequency in Hz')
        )

        self.declare_parameter(
            'max_velocity',
            1.0,
            ParameterDescriptor(description='Maximum joint velocity in rad/s')
        )

        self.declare_parameter(
            'robot_name',
            'humanoid_robot',
            ParameterDescriptor(description='Name of the robot')
        )

        # Create publisher
        self.publisher = self.create_publisher(Float64, '/control_output', 10)

        # Get parameter values
        self.control_freq = self.get_parameter('control_frequency').value
        self.max_vel = self.get_parameter('max_velocity').value
        self.robot_name = self.get_parameter('robot_name').value

        # Create timer based on parameter
        self.timer = self.create_timer(
            1.0 / self.control_freq,
            self.control_loop
        )

        self.get_logger().info(
            f'Configurable robot node started: {self.robot_name}, '
            f'freq={self.control_freq}Hz, max_vel={self.max_vel}rad/s'
        )

    def control_loop(self):
        # Example control logic using parameters
        output = Float64()
        output.data = min(self.max_vel, 0.5)  # Use parameter in control
        self.publisher.publish(output)

def main(args=None):
    rclpy.init(args=args)
    node = ConfigurableRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down configurable robot node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete Humanoid Control Example

A more comprehensive example combining multiple concepts:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist
import numpy as np

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Publishers
        self.cmd_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            '/robot_status',
            10
        )

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz

        # Internal state
        self.current_joints = {}
        self.imu_data = None
        self.desired_pose = np.zeros(28)  # 28 DOF example humanoid
        self.balance_active = True

        self.get_logger().info('Humanoid controller initialized')

    def joint_callback(self, msg):
        """Process joint state messages"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joints[name] = msg.position[i]

    def imu_callback(self, msg):
        """Process IMU data for balance control"""
        self.imu_data = {
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }

    def control_loop(self):
        """Main control loop"""
        if self.balance_active and self.imu_data:
            # Simple balance control based on IMU data
            pitch = self.get_pitch_from_imu()
            correction = self.calculate_balance_correction(pitch)

            # Apply correction to desired pose
            corrected_pose = self.desired_pose + correction

            # Publish commands
            cmd_msg = Float64MultiArray()
            cmd_msg.data = corrected_pose.tolist()
            self.cmd_publisher.publish(cmd_msg)

        # Publish status
        status_msg = String()
        status_msg.data = 'BALANCING' if self.balance_active else 'IDLE'
        self.status_publisher.publish(status_msg)

    def get_pitch_from_imu(self):
        """Extract pitch angle from IMU quaternion"""
        if not self.imu_data:
            return 0.0

        # Convert quaternion to pitch (simplified)
        q = self.imu_data['orientation']
        pitch = np.arcsin(2.0 * (q[3] * q[1] - q[0] * q[2]))
        return pitch

    def calculate_balance_correction(self, pitch):
        """Simple PD controller for balance"""
        kp = 10.0  # Proportional gain
        kd = 1.0   # Derivative gain

        correction = np.zeros(28)
        correction[0] = -kp * pitch  # Apply correction to first joint
        correction[1] = -kd * pitch  # Apply to second joint as well

        return correction

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down humanoid controller')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Examples

To run these examples:

1. **Save the code** to `.py` files in your ROS 2 workspace
2. **Make them executable**: `chmod +x your_script.py`
3. **Source your ROS 2 installation**: `source /opt/ros/humble/setup.bash` (or your ROS 2 version)
4. **Run the node**: `python3 your_script.py`

## Best Practices Demonstrated

These examples demonstrate several important best practices:

1. **Proper Node Lifecycle**: Initialize, spin, handle shutdown gracefully
2. **Error Handling**: Use try/except blocks and proper cleanup
3. **Logging**: Use ROS logging for debugging and monitoring
4. **Parameter Usage**: Make nodes configurable through parameters
5. **Message Efficiency**: Consider message size and frequency
6. **Modular Design**: Separate concerns and keep functions focused

These minimal examples provide a solid foundation for building more complex humanoid robot applications using Python and ROS 2 integration.