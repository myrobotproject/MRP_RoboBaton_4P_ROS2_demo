#include <image_transport/simple_publisher_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <hb_media_codec.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef ROBOBATON_RELEASE_VERSION
#define ROBOBATON_RELEASE_VERSION "0.0.0+unknown"
#endif

extern "C" const char* robobaton_nv12_compressed_image_transport_get_version(void) {
  return ROBOBATON_RELEASE_VERSION;
}

namespace robobaton_4p_ros2_demo {
namespace {

constexpr int kDefaultJpegQuality = 80;
constexpr hb_s32 kCodecTimeoutMs = 2000;

struct Nv12Planes {
  const unsigned char* y_data = nullptr;
  const unsigned char* uv_data = nullptr;
  int width = 0;
  int height = 0;
  int source_stride = 0;
  std::size_t chroma_height = 0U;
};

std::size_t CheckedProduct(std::size_t lhs, std::size_t rhs, const char* what) {
  if (lhs != 0U && rhs > std::numeric_limits<std::size_t>::max() / lhs) {
    throw std::runtime_error(what);
  }
  return lhs * rhs;
}

Nv12Planes ValidateAndLocatePlanes(const sensor_msgs::msg::Image& image) {
  if (image.encoding != "nv12" || image.width == 0U || image.height == 0U ||
      (image.width % 2U) != 0U || (image.height % 2U) != 0U || image.step < image.width ||
      image.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      image.height > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      image.step > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
    throw std::runtime_error("invalid NV12 image metadata for X5 JPEG compression");
  }

  const std::size_t stride = static_cast<std::size_t>(image.step);
  const std::size_t height = static_cast<std::size_t>(image.height);
  const std::size_t chroma_height = height / 2U;
  const std::size_t y_visible_size =
      CheckedProduct(stride, height, "invalid NV12 luma plane dimensions");
  const std::size_t uv_visible_size =
      CheckedProduct(stride, chroma_height, "invalid NV12 chroma plane dimensions");
  if (y_visible_size > std::numeric_limits<std::size_t>::max() - uv_visible_size) {
    throw std::runtime_error("invalid NV12 image byte dimensions");
  }
  const std::size_t compact_size = y_visible_size + uv_visible_size;
  if (image.data.size() < compact_size) {
    throw std::runtime_error("NV12 image data does not cover visible rows");
  }

  std::size_t y_plane_span = y_visible_size;
  if (image.data.size() != compact_size) {
    const std::size_t denominator = CheckedProduct(stride, 3U, "invalid NV12 stride");
    if (image.data.size() > std::numeric_limits<std::size_t>::max() / 2U ||
        (image.data.size() * 2U) % denominator != 0U) {
      throw std::runtime_error("unsupported NV12 padded image layout");
    }
    const std::size_t vstride = (image.data.size() * 2U) / denominator;
    if (vstride < height || (vstride % 2U) != 0U) {
      throw std::runtime_error("invalid NV12 padded vstride");
    }
    y_plane_span = CheckedProduct(stride, vstride, "invalid NV12 padded luma span");
    if (y_plane_span > image.data.size() || image.data.size() - y_plane_span < uv_visible_size) {
      throw std::runtime_error("NV12 padded image data does not cover chroma rows");
    }
  }

  const auto* data = reinterpret_cast<const unsigned char*>(image.data.data());
  return {data, data + y_plane_span, static_cast<int>(image.width),
          static_cast<int>(image.height), static_cast<int>(image.step), chroma_height};
}

std::size_t FindJpegEnd(const unsigned char* data, std::size_t capacity) {
  if (data == nullptr || capacity < 4U || data[0] != 0xFFU || data[1] != 0xD8U) {
    throw std::runtime_error("X5 JPEG output is missing SOI marker");
  }
  for (std::size_t index = capacity; index >= 2U; --index) {
    if (data[index - 2U] == 0xFFU && data[index - 1U] == 0xD9U) {
      return index;
    }
  }
  throw std::runtime_error("X5 JPEG output is missing EOI marker");
}

class X5Nv12JpegEncoder final {
 public:
  explicit X5Nv12JpegEncoder(int jpeg_quality) : jpeg_quality_(jpeg_quality) {
    if (jpeg_quality_ < 1 || jpeg_quality_ > 100) {
      throw std::invalid_argument("JPEG quality must be between 1 and 100");
    }
  }

  ~X5Nv12JpegEncoder() { Reset(); }
  X5Nv12JpegEncoder(const X5Nv12JpegEncoder&) = delete;
  X5Nv12JpegEncoder& operator=(const X5Nv12JpegEncoder&) = delete;

  std::vector<uint8_t> Encode(const sensor_msgs::msg::Image& image) {
    const Nv12Planes planes = ValidateAndLocatePlanes(image);
    EnsureContext(planes.width, planes.height);
    media_codec_buffer_t input{};
    media_codec_buffer_t output{};
    media_codec_output_buffer_info_t output_info{};
    try {
      CheckCodec(hb_mm_mc_dequeue_input_buffer(&context_, &input, kCodecTimeoutMs),
                 "dequeue JPEG input buffer");
      CopyIntoCodecInput(planes, &input);
      CheckCodec(hb_mm_mc_queue_input_buffer(&context_, &input, kCodecTimeoutMs),
                 "queue JPEG input buffer");
      CheckCodec(hb_mm_mc_dequeue_output_buffer(
                     &context_, &output, &output_info, kCodecTimeoutMs),
                 "dequeue JPEG output buffer");
      const std::size_t jpeg_size = FindJpegEnd(
          output.vstream_buf.vir_ptr, static_cast<std::size_t>(output.vstream_buf.size));
      std::vector<uint8_t> jpeg(output.vstream_buf.vir_ptr,
                                output.vstream_buf.vir_ptr + jpeg_size);
      CheckCodec(hb_mm_mc_queue_output_buffer(&context_, &output, kCodecTimeoutMs),
                 "queue JPEG output buffer");
      return jpeg;
    } catch (...) {
      Reset();
      throw;
    }
  }

 private:
  void CheckCodec(hb_s32 result, const char* operation) {
    if (result != 0) {
      throw std::runtime_error(std::string("X5 media codec failed to ") + operation +
                               ", status=" + std::to_string(result));
    }
  }

  void EnsureContext(int width, int height) {
    if (started_ && width == width_ && height == height_) {
      return;
    }
    Reset();
    const uint64_t pixels = static_cast<uint64_t>(width) * static_cast<uint64_t>(height);
    const uint64_t bitstream_size = ((pixels * 3U / 2U) + 4095U) & ~UINT64_C(4095);
    if (bitstream_size == 0U || bitstream_size > std::numeric_limits<uint32_t>::max()) {
      throw std::runtime_error("invalid X5 JPEG bitstream capacity");
    }

    context_ = {};
    startup_ = {};
    context_.encoder = true;
    context_.instance_index = -1;
    context_.codec_id = MEDIA_CODEC_ID_JPEG;
    context_.video_enc_params.width = width;
    context_.video_enc_params.height = height;
    context_.video_enc_params.pix_fmt = MC_PIXEL_FORMAT_NV12;
    context_.video_enc_params.bitstream_buf_size = static_cast<uint32_t>(bitstream_size);
    context_.video_enc_params.external_frame_buf = false;
    context_.video_enc_params.frame_buf_count = 3;
    context_.video_enc_params.bitstream_buf_count = 3;
    context_.video_enc_params.rot_degree = MC_CCW_0;
    context_.video_enc_params.mir_direction = MC_DIRECTION_NONE;
    context_.video_enc_params.frame_cropping_flag = false;
    context_.video_enc_params.enable_user_pts = 1;
    context_.video_enc_params.jpeg_enc_config.quality_factor = jpeg_quality_;
    context_.video_enc_params.jpeg_enc_config.restart_interval = width / 16;
    startup_.video_enc_startup_params.receive_frame_number = 0;

    CheckCodec(hb_mm_mc_initialize(&context_), "initialize JPEG encoder");
    initialized_ = true;
    CheckCodec(hb_mm_mc_configure(&context_), "configure JPEG encoder");
    CheckCodec(hb_mm_mc_start(&context_, &startup_), "start JPEG encoder");
    started_ = true;
    width_ = width;
    height_ = height;
  }

  static void CopyIntoCodecInput(const Nv12Planes& planes, media_codec_buffer_t* input) {
    if (input == nullptr || input->vframe_buf.vir_ptr[0] == nullptr ||
        input->vframe_buf.vir_ptr[1] == nullptr || input->vframe_buf.stride < planes.width) {
      throw std::runtime_error("X5 JPEG input buffer has invalid layout");
    }
    const int y_stride = input->vframe_buf.stride;
    const int uv_stride = input->vframe_buf.vstride > 0 ? input->vframe_buf.vstride : y_stride;
    if (uv_stride < planes.width) {
      throw std::runtime_error("X5 JPEG chroma buffer has invalid stride");
    }
    const uint64_t y_required =
        static_cast<uint64_t>(y_stride) * static_cast<uint64_t>(planes.height);
    const uint64_t uv_required =
        static_cast<uint64_t>(uv_stride) * static_cast<uint64_t>(planes.height / 2);
    if (input->vframe_buf.compSize[0] < y_required ||
        input->vframe_buf.compSize[1] < uv_required) {
      throw std::runtime_error("X5 JPEG input buffer capacity is too small");
    }

    for (int row = 0; row < planes.height; ++row) {
      std::memcpy(input->vframe_buf.vir_ptr[0] + static_cast<std::size_t>(row) * y_stride,
                  planes.y_data + static_cast<std::size_t>(row) * planes.source_stride,
                  static_cast<std::size_t>(planes.width));
    }
    for (std::size_t row = 0U; row < planes.chroma_height; ++row) {
      std::memcpy(input->vframe_buf.vir_ptr[1] + row * static_cast<std::size_t>(uv_stride),
                  planes.uv_data + row * static_cast<std::size_t>(planes.source_stride),
                  static_cast<std::size_t>(planes.width));
    }
    const uint64_t compact_size =
        static_cast<uint64_t>(planes.width) * static_cast<uint64_t>(planes.height) * 3U / 2U;
    input->type = MC_VIDEO_FRAME_BUFFER;
    input->vframe_buf.width = planes.width;
    input->vframe_buf.height = planes.height;
    input->vframe_buf.pix_fmt = MC_PIXEL_FORMAT_NV12;
    input->vframe_buf.size = static_cast<uint32_t>(compact_size);
  }

  void Reset() noexcept {
    if (started_) {
      (void)hb_mm_mc_pause(&context_);
      started_ = false;
    }
    if (initialized_) {
      (void)hb_mm_mc_release(&context_);
      initialized_ = false;
    }
    context_ = {};
    startup_ = {};
    width_ = 0;
    height_ = 0;
  }

  const int jpeg_quality_;
  media_codec_context_t context_{};
  mc_av_codec_startup_params_t startup_{};
  int width_ = 0;
  int height_ = 0;
  bool initialized_ = false;
  bool started_ = false;
};

std::string ParameterBaseName(const rclcpp::Node* node, const std::string& base_topic) {
  const auto namespace_length = node->get_effective_namespace().length();
  std::string param_base = base_topic.size() >= namespace_length
                               ? base_topic.substr(namespace_length)
                               : base_topic;
  std::replace(param_base.begin(), param_base.end(), '/', '.');
  if (!param_base.empty() && param_base.front() == '.') {
    param_base.erase(0, 1);
  }
  return param_base;
}

int DeclareJpegQuality(rclcpp::Node* node, const std::string& base_topic,
                       const rclcpp::Logger& logger) {
  const std::string param_name = ParameterBaseName(node, base_topic) + ".jpeg_quality";
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.name = "jpeg_quality";
  descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  descriptor.description = "JPEG quality for X5 hardware NV12 compressed image_transport output";
  descriptor.read_only = false;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 1;
  range.to_value = 100;
  range.step = 1;
  descriptor.integer_range.push_back(range);

  int64_t value = kDefaultJpegQuality;
  try {
    value = node->declare_parameter<int64_t>(param_name, kDefaultJpegQuality, descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
    RCLCPP_DEBUG(logger, "%s was previously declared", param_name.c_str());
    value = node->get_parameter(param_name).as_int();
  }
  if (value < 1 || value > 100) {
    throw std::invalid_argument("compressed jpeg_quality must be between 1 and 100");
  }
  return static_cast<int>(value);
}

}  // namespace

class Nv12CompressedPublisher final
    : public image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage> {
 public:
  Nv12CompressedPublisher() : logger_(rclcpp::get_logger("Nv12CompressedPublisher")) {}

  std::string getTransportName() const override { return "compressed"; }

 protected:
  void advertiseImpl(rclcpp::Node* node, const std::string& base_topic,
                     rmw_qos_profile_t custom_qos) override {
    using Base = image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage>;
    Base::advertiseImpl(node, base_topic, custom_qos);
    encoder_ = std::make_unique<X5Nv12JpegEncoder>(
        DeclareJpegQuality(node, base_topic, logger_));
  }

  void publish(const sensor_msgs::msg::Image& message, const PublishFn& publish_fn) const override {
    sensor_msgs::msg::CompressedImage compressed;
    compressed.header = message.header;
    compressed.format = "nv12; jpeg compressed bgr8";
    {
      std::lock_guard<std::mutex> lock(encoder_mutex_);
      if (!encoder_) {
        throw std::runtime_error("NV12 compressed publisher used before advertise");
      }
      compressed.data = encoder_->Encode(message);
    }
    publish_fn(compressed);
  }

 private:
  rclcpp::Logger logger_;
  mutable std::mutex encoder_mutex_;
  mutable std::unique_ptr<X5Nv12JpegEncoder> encoder_;
};

}  // namespace robobaton_4p_ros2_demo

PLUGINLIB_EXPORT_CLASS(robobaton_4p_ros2_demo::Nv12CompressedPublisher,
                       image_transport::PublisherPlugin)
