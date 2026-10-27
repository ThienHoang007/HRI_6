#!/usr/bin/env python3
"""Read a laptop webcam, recognize hand gestures, and publish stable results."""

from collections import deque
from pathlib import Path
from typing import Optional

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

try:
    import mediapipe as mp
except ImportError as exc:  # pragma: no cover - depends on the target machine
    mp = None
    _MEDIAPIPE_IMPORT_ERROR = exc
else:
    _MEDIAPIPE_IMPORT_ERROR = None


class GestureRecognitionNode(Node):
    """MediaPipe Gesture Recognizer node with temporal filtering and wave detection."""

    def __init__(self) -> None:
        super().__init__('gesture_recognition_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('min_gesture_score', 0.65)
        self.declare_parameter('min_hand_detection_confidence', 0.60)
        self.declare_parameter('min_hand_presence_confidence', 0.60)
        self.declare_parameter('min_tracking_confidence', 0.60)
        self.declare_parameter('stable_frames', 4)
        self.declare_parameter('num_hands', 1)
        self.declare_parameter('model_path', '')
        self.declare_parameter('show_window', True)
        self.declare_parameter('window_name', 'TurtleBot 4 - Gesture Recognition')
        self.declare_parameter('wave_min_x_range', 0.08)
        self.declare_parameter('wave_min_direction_changes', 2)
        self.declare_parameter('wave_hold_ms', 800)

        self.gesture_pub = self.create_publisher(String, '/detected_gesture', 10)
        self._window_name = str(self.get_parameter('window_name').value)
        self._show_window = bool(self.get_parameter('show_window').value)
        self._min_score = float(self.get_parameter('min_gesture_score').value)
        self._stable_frames_required = max(1, int(self.get_parameter('stable_frames').value))
        self._wave_min_x_range = float(self.get_parameter('wave_min_x_range').value)
        self._wave_min_direction_changes = max(
            1, int(self.get_parameter('wave_min_direction_changes').value)
        )
        self._wave_hold_ms = int(self.get_parameter('wave_hold_ms').value)

        self._candidate = 'None'
        self._candidate_count = 0
        self._stable_gesture = 'None'
        self._stable_score = 0.0
        self._palm_history = deque(maxlen=24)
        self._last_wave_ms: Optional[int] = None
        self._last_timestamp_ms = 0

        if mp is None:
            raise RuntimeError(
                'MediaPipe is not installed. Install it with: python3 -m pip install mediapipe'
            ) from _MEDIAPIPE_IMPORT_ERROR

        model_path = self._resolve_model_path()
        if not model_path.is_file():
            raise FileNotFoundError(
                f'MediaPipe model not found: {model_path}. '
                'Run scripts/download_gesture_model.sh before building the package, '
                'or pass model_path:=/absolute/path/gesture_recognizer.task.'
            )

        base_options = mp.tasks.BaseOptions(model_asset_path=str(model_path))
        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=base_options,
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=max(1, int(self.get_parameter('num_hands').value)),
            min_hand_detection_confidence=float(
                self.get_parameter('min_hand_detection_confidence').value
            ),
            min_hand_presence_confidence=float(
                self.get_parameter('min_hand_presence_confidence').value
            ),
            min_tracking_confidence=float(self.get_parameter('min_tracking_confidence').value),
        )
        self._recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)

        camera_index = int(self.get_parameter('camera_index').value)
        self._camera = cv2.VideoCapture(camera_index)
        self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter('frame_width').value))
        self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter('frame_height').value))
        if not self._camera.isOpened():
            self._recognizer.close()
            raise RuntimeError(f'Cannot open webcam at camera_index={camera_index}')

        fps = max(1.0, float(self.get_parameter('fps').value))
        self._timer = self.create_timer(1.0 / fps, self._process_frame)
        self.get_logger().info(f'Webcam opened at index {camera_index}')
        self.get_logger().info(f'Using MediaPipe model: {model_path}')
        self.get_logger().info('Publishing stable gestures on /detected_gesture')

    def _resolve_model_path(self) -> Path:
        configured = str(self.get_parameter('model_path').value).strip()
        if configured:
            return Path(configured).expanduser().resolve()
        share_dir = Path(get_package_share_directory('gesture_turtlebot4'))
        return share_dir / 'models' / 'gesture_recognizer.task'

    @staticmethod
    def _top_category(result):
        if not result.gestures or not result.gestures[0]:
            return 'None', 0.0
        category = result.gestures[0][0]
        name = getattr(category, 'category_name', None) or getattr(category, 'display_name', None)
        score = float(getattr(category, 'score', 0.0) or 0.0)
        if not name or score < 0.0:
            return 'None', 0.0
        return str(name), score

    def _detect_wave(self, gesture: str, landmarks, timestamp_ms: int) -> bool:
        if gesture != 'Open_Palm' or not landmarks:
            if timestamp_ms - (self._palm_history[-1][0] if self._palm_history else timestamp_ms) > 900:
                self._palm_history.clear()
            return False

        # Use a palm center instead of a fingertip so the test is less sensitive to pose.
        indices = (0, 5, 9, 13, 17)
        palm_x = sum(float(landmarks[i].x) for i in indices) / len(indices)
        self._palm_history.append((timestamp_ms, palm_x))
        recent = [x for t, x in self._palm_history if timestamp_ms - t <= 1200]
        if len(recent) < 6:
            return self._last_wave_ms is not None and timestamp_ms - self._last_wave_ms <= self._wave_hold_ms

        movement = [recent[i + 1] - recent[i] for i in range(len(recent) - 1)]
        movement = [delta for delta in movement if abs(delta) >= 0.008]
        if movement:
            signs = [1 if delta > 0 else -1 for delta in movement]
            direction_changes = sum(signs[i] != signs[i - 1] for i in range(1, len(signs)))
        else:
            direction_changes = 0
        if (
            max(recent) - min(recent) >= self._wave_min_x_range
            and direction_changes >= self._wave_min_direction_changes
        ):
            self._last_wave_ms = timestamp_ms

        return self._last_wave_ms is not None and timestamp_ms - self._last_wave_ms <= self._wave_hold_ms

    def _update_stability(self, gesture: str, score: float) -> None:
        if gesture == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = gesture
            self._candidate_count = 1

        if self._candidate_count >= self._stable_frames_required:
            self._stable_gesture = self._candidate
            self._stable_score = score if self._stable_gesture != 'None' else 0.0

    def _publish_gesture(self) -> None:
        msg = String()
        msg.data = self._stable_gesture
        self.gesture_pub.publish(msg)

    def _draw_overlay(self, frame, current_gesture: str, current_score: float) -> None:
        stable = self._stable_gesture
        stable_text = f'Stable: {stable}'
        if stable != 'None':
            stable_text += f' ({self._stable_score:.0%})'
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 92), (20, 20, 20), -1)
        cv2.putText(
            frame,
            f'Current: {current_gesture} ({current_score:.0%})',
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            stable_text,
            (12, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            'Pointing Up/Victory/Wave -> select | Thumb Up -> confirm | Palm -> STOP | Fist -> cancel',
            (12, 82),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def _process_frame(self) -> None:
        ok, frame = self._camera.read()
        if not ok or frame is None:
            self.get_logger().warning('Cannot read a frame from webcam')
            self._update_stability('None', 0.0)
            self._publish_gesture()
            return

        timestamp_ms = self.get_clock().now().nanoseconds // 1_000_000
        if timestamp_ms <= self._last_timestamp_ms:
            timestamp_ms = self._last_timestamp_ms + 1
        self._last_timestamp_ms = timestamp_ms

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self._recognizer.recognize_for_video(mp_image, timestamp_ms)
        gesture, score = self._top_category(result)
        landmarks = result.hand_landmarks[0] if result.hand_landmarks else None
        if self._detect_wave(gesture, landmarks, timestamp_ms):
            gesture = 'Wave'

        self._update_stability(gesture, score)
        self._publish_gesture()

        if self._show_window:
            self._draw_overlay(frame, gesture, score)
            cv2.imshow(self._window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or cv2.getWindowProperty(self._window_name, cv2.WND_PROP_VISIBLE) < 1:
                self.get_logger().info('Webcam window closed; stopping gesture node')
                rclpy.shutdown()

    def destroy_node(self):
        if hasattr(self, '_timer'):
            self._timer.cancel()
        if hasattr(self, '_camera'):
            self._camera.release()
        if hasattr(self, '_recognizer'):
            self._recognizer.close()
        if self._show_window:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GestureRecognitionNode()
        rclpy.spin(node)
    except (FileNotFoundError, RuntimeError) as exc:
        print(f'[gesture_recognition_node] {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
