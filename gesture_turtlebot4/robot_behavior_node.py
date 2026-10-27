#!/usr/bin/env python3
"""Map stable gestures to confirmed TurtleBot 4 motion behaviors."""

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class RobotBehaviorNode(Node):
    """Safety-oriented gesture state machine for /cmd_vel."""

    def __init__(self) -> None:
        super().__init__('robot_behavior_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('gesture_topic', '/detected_gesture')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('stable_messages', 2)
        self.declare_parameter('signal_timeout_s', 1.5)
        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 0.70)
        self.declare_parameter('greeting_half_s', 0.80)
        self.declare_parameter('forward_s', 1.00)
        self.declare_parameter('backward_s', 1.00)
        self.declare_parameter('pause_s', 0.30)
        self.declare_parameter('forward_backward_repetitions', 4)
        self.declare_parameter('circle_linear_speed', 0.18)
        self.declare_parameter('circle_angular_speed', 0.35)

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        gesture_topic = str(self.get_parameter('gesture_topic').value)
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._gesture_sub = self.create_subscription(String, gesture_topic, self._on_gesture, 10)

        self._control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self._stable_messages = max(1, int(self.get_parameter('stable_messages').value))
        self._signal_timeout_s = max(0.2, float(self.get_parameter('signal_timeout_s').value))
        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._greeting_half_s = float(self.get_parameter('greeting_half_s').value)
        self._forward_s = float(self.get_parameter('forward_s').value)
        self._backward_s = float(self.get_parameter('backward_s').value)
        self._pause_s = float(self.get_parameter('pause_s').value)
        self._repetitions = max(1, int(self.get_parameter('forward_backward_repetitions').value))
        self._circle_linear_speed = float(self.get_parameter('circle_linear_speed').value)
        self._circle_angular_speed = float(self.get_parameter('circle_angular_speed').value)

        self._selected_behavior: Optional[str] = None
        self._active_behavior: Optional[str] = None
        self._phase_index = 0
        self._repeat_index = 0
        self._phase_started = time.monotonic()
        self._last_signal_time = time.monotonic()
        self._pending_gesture: Optional[str] = None
        self._pending_count = 0
        self._last_event_gesture: Optional[str] = None
        self._last_timeout_log = 0.0

        self._timer = self.create_timer(1.0 / self._control_rate_hz, self._control_loop)
        self.get_logger().info(f'Subscribing to {gesture_topic}; publishing velocity on {cmd_vel_topic}')
        self.get_logger().info(
            'Select with Wave/Pointing_Up/Victory, confirm with Thumb_Up, '
            'stop with Open_Palm, cancel with Closed_Fist.'
        )

    def _publish_velocity(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        self._publish_velocity(0.0, 0.0)

    def _on_gesture(self, msg: String) -> None:
        gesture = msg.data.strip()
        now = time.monotonic()
        self._last_signal_time = now

        # Open Palm is the emergency stop path. It does not wait for the debounce counter.
        if gesture == 'Open_Palm':
            self._pending_gesture = None
            self._pending_count = 0
            self._last_event_gesture = 'Open_Palm'
            self._cancel_active(clear_selection=True)
            self._publish_stop()
            return

        if gesture in ('', 'None'):
            self._pending_gesture = None
            self._pending_count = 0
            self._last_event_gesture = None
            return

        if gesture not in ('Wave', 'Pointing_Up', 'Victory', 'Thumb_Up', 'Closed_Fist'):
            return

        if gesture == self._pending_gesture:
            self._pending_count += 1
        else:
            self._pending_gesture = gesture
            self._pending_count = 1

        if self._pending_count < self._stable_messages or gesture == self._last_event_gesture:
            return

        self._last_event_gesture = gesture
        self._handle_gesture_event(gesture)

    def _handle_gesture_event(self, gesture: str) -> None:
        if gesture == 'Closed_Fist':
            self._cancel_active(clear_selection=True)
            self.get_logger().info('Closed Fist: behavior cancelled; waiting')
            return

        if gesture == 'Wave':
            self._select_behavior('greeting')
        elif gesture == 'Pointing_Up':
            self._select_behavior('forward_backward')
        elif gesture == 'Victory':
            self._select_behavior('circle')
        elif gesture == 'Thumb_Up':
            if self._selected_behavior is None:
                self.get_logger().info('Thumb Up received, but no behavior is selected')
            elif self._active_behavior is None:
                self._start_selected_behavior()
            else:
                self.get_logger().info('A behavior is already running; Thumb Up ignored')

    def _select_behavior(self, behavior: str) -> None:
        if self._active_behavior is not None:
            self.get_logger().info('A behavior is running; selection ignored until it finishes')
            return
        self._selected_behavior = behavior
        labels = {
            'greeting': 'Wave greeting',
            'forward_backward': 'forward/backward x4',
            'circle': 'circular trajectory',
        }
        self.get_logger().info(
            f'Selected: {labels[behavior]}. Show Thumb Up to confirm and execute.'
        )

    def _start_selected_behavior(self) -> None:
        self._active_behavior = self._selected_behavior
        self._phase_index = 0
        self._repeat_index = 0
        self._phase_started = time.monotonic()
        self.get_logger().info(f'Confirmed: executing {self._active_behavior}')

    def _cancel_active(self, clear_selection: bool) -> None:
        self._active_behavior = None
        self._phase_index = 0
        self._repeat_index = 0
        if clear_selection:
            self._selected_behavior = None
        self._publish_stop()

    def _finish_behavior(self) -> None:
        completed = self._active_behavior
        self._active_behavior = None
        self._selected_behavior = None
        self._phase_index = 0
        self._repeat_index = 0
        self._publish_stop()
        if completed:
            self.get_logger().info(f'Completed: {completed}')

    def _run_greeting(self, now: float) -> None:
        phases = (self._angular_speed, -self._angular_speed, self._angular_speed)
        if now - self._phase_started >= self._greeting_half_s:
            self._phase_index += 1
            self._phase_started = now
            if self._phase_index >= len(phases):
                self._finish_behavior()
                return
        self._publish_velocity(0.0, phases[self._phase_index])

    def _run_forward_backward(self, now: float) -> None:
        phases = (
            (self._linear_speed, self._forward_s),
            (0.0, self._pause_s),
            (-self._linear_speed, self._backward_s),
            (0.0, self._pause_s),
        )
        speed, duration = phases[self._phase_index]
        if now - self._phase_started >= duration:
            self._phase_index += 1
            self._phase_started = now
            if self._phase_index >= len(phases):
                self._phase_index = 0
                self._repeat_index += 1
                if self._repeat_index >= self._repetitions:
                    self._finish_behavior()
                    return
            speed, _ = phases[self._phase_index]
        self._publish_velocity(speed, 0.0)

    def _run_circle(self, now: float) -> None:
        angular = abs(self._circle_angular_speed)
        if angular < 1e-6:
            self.get_logger().error('circle_angular_speed must be non-zero')
            self._finish_behavior()
            return
        duration = 2.0 * math.pi / angular
        if now - self._phase_started >= duration:
            self._finish_behavior()
            return
        self._publish_velocity(self._circle_linear_speed, self._circle_angular_speed)

    def _control_loop(self) -> None:
        now = time.monotonic()
        if now - self._last_signal_time > self._signal_timeout_s:
            self._cancel_active(clear_selection=True)
            if now - self._last_timeout_log > self._signal_timeout_s:
                self.get_logger().warning('Gesture signal timeout: robot stopped and state cleared')
                self._last_timeout_log = now
            return

        if self._active_behavior == 'greeting':
            self._run_greeting(now)
        elif self._active_behavior == 'forward_backward':
            self._run_forward_backward(now)
        elif self._active_behavior == 'circle':
            self._run_circle(now)
        else:
            self._publish_stop()

    def destroy_node(self):
        if hasattr(self, '_timer'):
            self._timer.cancel()
        # rclpy may already have invalidated the context during Ctrl-C shutdown.
        if rclpy.ok():
            self._publish_stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
