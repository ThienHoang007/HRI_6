# Điều khiển TurtleBot 4 bằng cử chỉ tay

Package ROS 2 này thực hiện bài tập gồm webcam OpenCV, MediaPipe Gesture Recognizer và điều khiển TurtleBot 4 qua `/cmd_vel`.

## Cấu trúc

- `gesture_recognition_node`: đọc webcam, nhận diện cử chỉ, lọc ổn định theo thời gian, phát hiện `Wave` từ chuyển động ngang của bàn tay mở và publish `/detected_gesture` (`std_msgs/msg/String`).
- `robot_behavior_node`: nhận cử chỉ, cho phép chọn hành vi, yêu cầu `Thumb_Up` để xác nhận, phát lệnh `/cmd_vel`, dừng khi `Open_Palm` hoặc khi mất heartbeat.
- `launch/gesture_turtlebot4.launch.py`: chạy hai node cùng lúc.
- `config/params.yaml`: tham số camera, ngưỡng nhận diện và tốc độ robot.

MediaPipe Gesture Recognizer mặc định nhận các nhãn `Closed_Fist`, `Open_Palm`, `Pointing_Up`, `Thumb_Up`, `Victory` và một số nhãn khác. `Wave` được bổ sung bằng kiểm tra dao động ngang của tâm lòng bàn tay khi nhãn nền là `Open_Palm`.

## Cài đặt trên Ubuntu + ROS 2

TurtleBot 4 hỗ trợ ROS 2 Humble trên Ubuntu 22.04 và ROS 2 Jazzy trên Ubuntu 24.04. Chọn đúng `$ROS_DISTRO` đang cài trên máy:

```bash
sudo apt update
sudo apt install -y   ros-$ROS_DISTRO-turtlebot4-simulator   ros-$ROS_DISTRO-irobot-create-nodes   python3-opencv python3-pip python3-venv
```

Tạo workspace và chép package:

```bash
mkdir -p ~/tb4_gesture_ws/src
cp -r /home/thienhoang/Documents/Codex/2026-10-26/3-n-i-dung-b-i/outputs/gesture_turtlebot4 ~/tb4_gesture_ws/src/
cd ~/tb4_gesture_ws/src/gesture_turtlebot4
python3 -m pip install --user -r requirements.txt
./scripts/download_gesture_model.sh
cd ~/tb4_gesture_ws
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Nếu hệ thống chặn `pip --user`, tạo virtualenv theo chính sách Python của máy rồi cài `mediapipe` và `opencv-python` trong virtualenv đó. ROS 2 vẫn cần được source trước khi chạy node.

## Chạy mô phỏng và kiểm tra `/cmd_vel`

Terminal 1. Máy Ubuntu 22.04 của bạn đang dùng ROS 2 Humble, vì vậy dùng launch file Ignition của Humble:

```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py \
  world:=warehouse \
  model:=standard
```

Nếu muốn dùng TurtleBot 4 Lite:

```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py model:=lite
```

Không thêm `rviz:=false`, `nav2:=false` hoặc `slam:=false` vào launch file Humble này; các tham số đó không có trong phiên bản đang cài trên máy bạn.

Chờ Gazebo chạy và nhấn Play nếu giao diện yêu cầu. Kiểm tra topic:

```bash
source ~/tb4_gesture_ws/install/setup.bash
ros2 topic list | grep cmd_vel
ros2 topic info /cmd_vel
```

Có thể kiểm tra ngắn bằng một lệnh vận tốc nhỏ trong vùng mô phỏng rộng, sau đó gửi lệnh dừng:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Lệnh trên chỉ là kiểm tra topic trước demo; trong demo chính, robot được điều khiển bởi `robot_behavior_node`, không dùng bàn phím.

## Chạy bài demo

Terminal 2:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/tb4_gesture_ws/install/setup.bash
ros2 launch gesture_turtlebot4 gesture_turtlebot4.launch.py
```

Nếu webcam không phải camera số 0:

```bash
ros2 launch gesture_turtlebot4 gesture_turtlebot4.launch.py camera_index:=1
```

Nếu muốn chạy không mở cửa sổ webcam:

```bash
ros2 launch gesture_turtlebot4 gesture_turtlebot4.launch.py show_window:=false
```

Theo dõi kết quả nhận diện ở terminal 3:

```bash
source ~/tb4_gesture_ws/install/setup.bash
ros2 topic echo /detected_gesture
```

## Kịch bản thao tác

Mỗi lựa chọn cần giữ ổn định vài khung hình. Sau đó giữ `Thumb_Up` để xác nhận:

| Cử chỉ | Kết quả |
|---|---|
| `Wave` | Chọn robot xoay trái-phải để chào |
| `Pointing_Up` | Chọn tiến-lùi 4 lần |
| `Victory` | Chọn chạy một vòng quỹ đạo tròn |
| `Thumb_Up` | Xác nhận và chạy lựa chọn |
| `Open_Palm` | Dừng ngay, hủy hành vi đang chạy và xóa lựa chọn |
| `Closed_Fist` | Hủy lựa chọn/hành vi và về trạng thái chờ |

Ví dụ: giơ một ngón tay, chờ log `Selected`, sau đó giơ ngón cái. Với `Wave`, hãy vẫy ngang bàn tay mở đủ rõ để bộ nhận diện phân biệt với `Open_Palm` đứng yên.

## Cơ chế an toàn và chống nhiễu

- Gesture node chỉ công nhận một nhãn sau `stable_frames` khung hình liên tiếp.
- Behavior node tiếp tục yêu cầu `stable_messages` bản tin giống nhau trước khi xử lý các sự kiện chọn/xác nhận/hủy.
- Một cử chỉ giữ lâu chỉ tạo một sự kiện; phải chuyển về `None` hoặc nhãn khác rồi mới tạo sự kiện mới.
- `Open_Palm` là đường dừng khẩn cấp và publish zero velocity ngay.
- Nếu không nhận được heartbeat `/detected_gesture` trong `signal_timeout_s` giây, robot publish vận tốc 0 và xóa trạng thái.
- Sau khi hành vi hoàn tất, node publish zero velocity.
- Tốc độ mặc định thấp và bán kính quỹ đạo tròn khoảng `0.18 / 0.35 = 0.51 m`, phù hợp để chạy trong warehouse world đủ rộng. Có thể giảm tiếp trong `config/params.yaml` nếu cần.

## Lưu ý khi nộp bài

Ảnh/video demo nên thể hiện: cửa sổ webcam có tên cử chỉ và độ tin cậy, `ros2 topic echo /detected_gesture`, Gazebo đang chạy và log chọn/xác nhận hành vi. Không điều khiển robot bằng bàn phím trong phần demo; chỉ dùng cử chỉ tay.
