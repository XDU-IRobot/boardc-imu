
#ifndef DECODER_HPP
#define DECODER_HPP

#include <deque>
#include <array>
#include <functional>

#include <geometry_msgs/msg/detail/quaternion_stamped__struct.hpp>
#include <memory>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include "crc.h"

enum class DecoderState {
  kSof,
  kSeq,
  kCrc8,
  kPayloadCrc16,
};

/**
 * @brief 解包器，读取字节流，解析数据帧
 */
class Decoder {
 public:
  explicit Decoder(std::function<void(geometry_msgs::msg::QuaternionStamped::SharedPtr)> callback);
  Decoder() = delete;

  Decoder &operator<<(const std::string &s);

 private:
  DecoderState state_{DecoderState::kSof};
  std::function<void(geometry_msgs::msg::QuaternionStamped::SharedPtr)> callback_;
  geometry_msgs::msg::QuaternionStamped::SharedPtr quaternion_msg_{std::make_shared<geometry_msgs::msg::QuaternionStamped>()};

  std::deque<uint8_t> in_queue_{};
  std::array<uint8_t, 20> out_buffer_{};

 private:  // internal states
  usize idx_{0}, seq_{0};
};

#endif