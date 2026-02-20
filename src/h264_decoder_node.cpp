#include <atomic>
#include <cstring>
#include <string>
#include <thread>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

class H264DecoderNode : public rclcpp::Node
{
public:
  H264DecoderNode()
  : Node("h264_decoder_node")
  {
    this->declare_parameter<std::string>("input_topic", "/cam0/h264");
    this->declare_parameter<std::string>("output_topic", "/cam0/image_raw");
    this->declare_parameter<std::string>("frame_id", "cam0");

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    rclcpp::QoS sub_qos(1);
    sub_qos.best_effort();
    sub_qos.durability_volatile();

    rclcpp::QoS pub_qos(1);
    pub_qos.reliable();

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, pub_qos);

    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic_, sub_qos,
      std::bind(&H264DecoderNode::onH264Msg, this, std::placeholders::_1));

    initPipeline();

    RCLCPP_INFO(this->get_logger(), "H264 decoder: %s -> %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

  ~H264DecoderNode()
  {
    destroyPipeline();
  }

private:
  void initPipeline()
  {
    gst_init(nullptr, nullptr);

    std::string pipeline_str =
      "appsrc name=src is-live=true block=false format=3"
      " ! h264parse"
      " ! nvv4l2decoder enable-max-performance=true disable-dpb=true"
      " ! nvvidconv"
      " ! video/x-raw,format=BGRx"
      " ! appsink name=sink max-buffers=1 drop=true sync=false emit-signals=false";

    GError * error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!pipeline_ || error) {
      std::string err_msg = error ? error->message : "unknown error";
      if (error) {
        g_error_free(error);
      }
      RCLCPP_ERROR(this->get_logger(), "Pipeline creation failed: %s", err_msg.c_str());
      pipeline_ = nullptr;
      return;
    }

    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");

    if (!appsrc_ || !appsink_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get appsrc/appsink elements");
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      return;
    }

    GstCaps * caps = gst_caps_new_simple(
      "video/x-h264",
      "stream-format", G_TYPE_STRING, "byte-stream",
      "alignment", G_TYPE_STRING, "au",
      nullptr);
    gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps);
    gst_caps_unref(caps);

    g_object_set(
      appsrc_,
      "stream-type", 0,
      "max-bytes", static_cast<guint64>(2 * 1024 * 1024),
      nullptr);

    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PAUSED);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set pipeline to PAUSED");
      gst_object_unref(appsrc_);
      gst_object_unref(appsink_);
      gst_object_unref(pipeline_);
      appsrc_ = nullptr;
      appsink_ = nullptr;
      pipeline_ = nullptr;
      return;
    }

    GstState state;
    ret = gst_element_get_state(pipeline_, &state, nullptr, 5 * GST_SECOND);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "Pipeline failed to reach PAUSED state");
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(appsrc_);
      gst_object_unref(appsink_);
      gst_object_unref(pipeline_);
      appsrc_ = nullptr;
      appsink_ = nullptr;
      pipeline_ = nullptr;
      return;
    }

    stop_ = false;
    decode_thread_ = std::thread(&H264DecoderNode::decodeThread, this);

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    pipeline_ready_ = true;

    RCLCPP_INFO(this->get_logger(), "GStreamer pipeline ready");
  }

  void destroyPipeline()
  {
    pipeline_ready_ = false;

    if (pipeline_) {
      stop_ = true;
      gst_element_send_event(pipeline_, gst_event_new_eos());

      if (decode_thread_.joinable()) {
        decode_thread_.join();
      }

      gst_element_set_state(pipeline_, GST_STATE_NULL);

      if (appsrc_) {
        gst_object_unref(appsrc_);
        appsrc_ = nullptr;
      }
      if (appsink_) {
        gst_object_unref(appsink_);
        appsink_ = nullptr;
      }
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  void onH264Msg(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    if (!pipeline_ready_ || !appsrc_) {
      return;
    }

    if (msg->format.find("h264") == std::string::npos) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Unexpected format: %s (expected h264)",
        msg->format.c_str());
      return;
    }

    if (msg->data.empty()) {
      return;
    }

    GstBuffer * buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
      std::memcpy(map.data, msg->data.data(), msg->data.size());
      gst_buffer_unmap(buffer, &map);
    }

    gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
  }

  void decodeThread()
  {
    int width = 0, height = 0;
    bool first_frame = true;

    RCLCPP_INFO(this->get_logger(), "Decode thread started");

    while (!stop_.load()) {
      GstSample * sample = gst_app_sink_try_pull_sample(
        GST_APP_SINK(appsink_), GST_SECOND);

      if (!sample) {
        if (gst_app_sink_is_eos(GST_APP_SINK(appsink_))) {
          RCLCPP_INFO(this->get_logger(), "Decode thread: EOS received");
          break;
        }
        continue;
      }

      GstBuffer * buf = gst_sample_get_buffer(sample);
      if (!buf) {
        gst_sample_unref(sample);
        continue;
      }

      if (first_frame) {
        GstCaps * caps = gst_sample_get_caps(sample);
        if (caps) {
          GstStructure * s = gst_caps_get_structure(caps, 0);
          gst_structure_get_int(s, "width", &width);
          gst_structure_get_int(s, "height", &height);
          if (width > 0 && height > 0) {
            RCLCPP_INFO(this->get_logger(), "First decoded frame: %dx%d", width, height);
            first_frame = false;
          }
        }
      }

      if (width <= 0 || height <= 0) {
        gst_sample_unref(sample);
        continue;
      }

      GstMapInfo info;
      if (!gst_buffer_map(buf, &info, GST_MAP_READ)) {
        gst_sample_unref(sample);
        continue;
      }

      // Build and publish sensor_msgs/Image (bgr8)
      auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
      img_msg->header.stamp = this->now();
      img_msg->header.frame_id = frame_id_;
      img_msg->height = height;
      img_msg->width = width;
      img_msg->encoding = "bgr8";
      img_msg->is_bigendian = false;
      img_msg->step = width * 3;
      img_msg->data.resize(width * height * 3);

      // Convert BGRx (4 bytes) -> BGR (3 bytes)
      const uint8_t * src = info.data;
      uint8_t * dst = img_msg->data.data();
      const int num_pixels = width * height;
      for (int i = 0; i < num_pixels; ++i) {
        dst[0] = src[0];
        dst[1] = src[1];
        dst[2] = src[2];
        src += 4;
        dst += 3;
      }

      pub_->publish(std::move(img_msg));

      gst_buffer_unmap(buf, &info);
      gst_sample_unref(sample);
    }

    RCLCPP_INFO(this->get_logger(), "Decode thread stopped");
  }

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;

  // GStreamer
  GstElement * pipeline_ = nullptr;
  GstElement * appsrc_ = nullptr;
  GstElement * appsink_ = nullptr;

  // Decode thread
  std::thread decode_thread_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> pipeline_ready_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<H264DecoderNode>());
  rclcpp::shutdown();
  return 0;
}
