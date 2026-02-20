# h264_decoder_node

ROS 2 node that decodes H264 compressed images to `sensor_msgs/Image` using GStreamer with NVIDIA hardware decoder (`nvv4l2decoder`).

## Architecture

```
/cam0/h264 (sensor_msgs/CompressedImage, H264)
    │
    ▼
[h264_decoder_node]
  GStreamer pipeline:
  appsrc → h264parse → nvv4l2decoder → nvvidconv → BGRx → appsink
    │
    ▼
/cam0/image_raw (sensor_msgs/Image, bgr8)
```

## Requirements

- NVIDIA Jetson (AGX Orin, etc.) with JetPack
- ROS 2 Humble
- GStreamer 1.0
  - `libgstreamer1.0-dev`
  - `libgstreamer-plugins-base1.0-dev`
  - `gstreamer1.0-plugins-bad` (provides `nvv4l2decoder`)

## Build

```bash
cd <your_workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-select h264_decoder_node --symlink-install
```

## Usage

```bash
source install/setup.bash
ros2 launch h264_decoder_node h264_decoder.launch.py
```

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `input_topic` | `/cam0/h264` | Input H264 CompressedImage topic |
| `output_topic` | `/cam0/image_raw` | Output decoded Image topic |
| `frame_id` | `cam0` | frame_id in output Image header |

### Example: Multiple cameras

```bash
# Camera 0
ros2 launch h264_decoder_node h264_decoder.launch.py \
  input_topic:=/cam0/h264 output_topic:=/cam0/image_raw frame_id:=cam0

# Camera 1
ros2 launch h264_decoder_node h264_decoder.launch.py \
  input_topic:=/cam1/h264 output_topic:=/cam1/image_raw frame_id:=cam1
```

## QoS

- **Subscriber** (H264 input): best_effort, depth 1, volatile
- **Publisher** (decoded output): reliable, depth 1

## Performance

Tested on Jetson AGX Orin (JetPack 6, TensorRT 10.3):

- 1920x1280 H264 stream: hardware decoded at input rate with minimal CPU overhead
- NVIDIA `nvv4l2decoder` provides zero-copy GPU decoding
