#!/usr/bin/env python3
"""
lift_controller.py

PID position controller cho encoder_lift_joint trong Gazebo.
Nhận /lift/command (Float64) - vị trí mục tiêu (0.0 - 0.87m)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import ApplyJointEffort, JointRequest
from builtin_interfaces.msg import Duration


class LiftController(Node):
    def __init__(self):
        super().__init__('lift_controller')

        # Parameters
        self.declare_parameter('joint_name', 'encoder_lift_joint')
        self.declare_parameter('Kp', 500.0)   # Proportional gain
        self.declare_parameter('Ki', 10.0)     # Integral gain
        self.declare_parameter('Kd', 50.0)     # Derivative gain
        self.declare_parameter('max_effort', 100.0)  # Max effort N.m
        self.declare_parameter('min_position', 0.0)
        self.declare_parameter('max_position', 0.87)

        self.joint_name = self.get_parameter('joint_name').value
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.max_effort = self.get_parameter('max_effort').value
        self.min_pos = self.get_parameter('min_position').value
        self.max_pos = self.get_parameter('max_position').value

        # PID state
        self.target_position = 0.0
        self.current_position = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()

        # Service clients
        self.get_logger().info('Waiting for Gazebo services...')
        self.effort_client = self.create_client(ApplyJointEffort, '/gazebo/apply_joint_effort')
        self.clear_client = self.create_client(JointRequest, '/gazebo/clear_joint_effort')

        while not self.effort_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                return
            self.get_logger().info('Waiting for /gazebo/apply_joint_effort...')

        self.get_logger().info('Gazebo services ready!')

        # Subscribers
        self.lift_sub = self.create_subscription(
            Float64,
            '/lift/command',
            self.lift_command_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for PID control (50 Hz)
        self.timer = self.create_timer(0.02, self.pid_control)

        self.get_logger().info(f'lift_controller started (PID: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd})')

    def lift_command_callback(self, msg: Float64):
        """Set target position from /lift/command topic"""
        self.target_position = max(self.min_pos, min(self.max_pos, msg.data))
        self.get_logger().debug(f'New target: {self.target_position:.3f}m')

    def joint_state_callback(self, msg: JointState):
        """Get current lift position from joint_states"""
        try:
            idx = msg.name.index(self.joint_name)
            self.current_position = msg.position[idx]
        except (ValueError, IndexError):
            pass  # Joint not found yet

    def pid_control(self):
        """PID control loop at 50 Hz"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0 or dt > 1.0:
            self.last_time = current_time
            return

        # Calculate error
        error = self.target_position - self.current_position

        # PID terms
        self.integral += error * dt
        self.integral = max(-10.0, min(10.0, self.integral))  # Anti-windup
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        # PID output
        effort = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Clamp effort
        effort = max(-self.max_effort, min(self.max_effort, effort))

        # Apply effort to joint
        self.apply_effort(effort, duration=0.025)

        self.prev_error = error
        self.last_time = current_time

        # Debug output
        if abs(error) > 0.01:  # Only log if far from target
            self.get_logger().debug(
                f'pos={self.current_position:.3f} target={self.target_position:.3f} '
                f'error={error:.3f} effort={effort:.2f}'
            )

    def apply_effort(self, effort, duration):
        """Apply effort to the joint for specified duration"""
        request = ApplyJointEffort.Request()
        request.joint_name = self.joint_name
        request.effort = effort
        request.duration = Duration(sec=0, nanosec=int(duration * 1e9))

        self.effort_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = LiftController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
