
#include "decoder.h"

#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "typedefs.h"

using namespace std::string_literals;

static f32 IntToFloat(int x_int, f32 x_min, f32 x_max, int bits) {
  f32 span = x_max - x_min;
  f32 offset = x_min;
  return ((f32)x_int) * span / ((f32)((1 << bits) - 1)) + offset;
}

constexpr u8 kSof = 0xa5;
constexpr u8 kPacketLength = 13;

Decoder::Decoder(std::function<void(geometry_msgs::msg::QuaternionStamped::SharedPtr)> callback)
    : callback_(callback) {}

Decoder &Decoder::operator<<(const std::string &s) {
  for (const auto &c : s) {
    in_queue_.push_back(c);
  }
  u8 byte;

  while (!in_queue_.empty()) {
    // RCLCPP_INFO(rclcpp::get_logger("boardc_imu_node"), "idx = %ld, data = %x, state = %d", idx_, byte, state_);

    byte = in_queue_.front();
    in_queue_.pop_front();
    switch (state_) {
      case DecoderState::kSof: {
        if (byte == kSof) {
          state_ = DecoderState::kSeq;
          out_buffer_[idx_++] = byte;
        }
        break;
      }
      case DecoderState::kSeq: {
        out_buffer_[idx_++] = byte;
        if (idx_ == 2) {
          state_ = DecoderState::kCrc8;
        } else {
          state_ = DecoderState::kSof;
          idx_ = 0;
        }
        break;
      }
      case DecoderState::kCrc8: {
        out_buffer_[idx_++] = byte;
        if (idx_ == 3 &&
            rm::modules::algorithm::Crc8(out_buffer_.data(), 2, rm::modules::algorithm::CRC8_INIT) == byte) {
          state_ = DecoderState::kPayloadCrc16;
        } else {
          RCLCPP_WARN(rclcpp::get_logger("boardc_imu_node"), "CRC8 error, drop packet, calc = %x, recv = %x",
                      rm::modules::algorithm::Crc8(out_buffer_.data(), 2, rm::modules::algorithm::CRC8_INIT), byte);
          state_ = DecoderState::kSof;
          idx_ = 0;
        }
        break;
      }
      case DecoderState::kPayloadCrc16: {
        if (idx_ < kPacketLength) {
          out_buffer_[idx_++] = byte;
        } else {
          if (rm::modules::algorithm::Crc16(out_buffer_.data(), kPacketLength - 2,
                                            rm::modules::algorithm::CRC16_INIT) ==
              (u16)(out_buffer_[kPacketLength - 2] | out_buffer_[kPacketLength - 1] << 8)) {
            quaternion_msg_->header.stamp = rclcpp::Clock().now();
            quaternion_msg_->header.frame_id = "imu";
            quaternion_msg_->quaternion.w = IntToFloat(out_buffer_[3] | out_buffer_[4] << 8, -1.f, 1.f, 16);
            quaternion_msg_->quaternion.x = IntToFloat(out_buffer_[5] | out_buffer_[6] << 8, -1.f, 1.f, 16);
            quaternion_msg_->quaternion.y = IntToFloat(out_buffer_[7] | out_buffer_[8] << 8, -1.f, 1.f, 16);
            quaternion_msg_->quaternion.z = IntToFloat(out_buffer_[9] | out_buffer_[10] << 8, -1.f, 1.f, 16);
            callback_(quaternion_msg_);
          } else {
            RCLCPP_WARN(rclcpp::get_logger("boardc_imu_node"), "CRC16 error, drop packet, calc = %x, recv = %x",
                        rm::modules::algorithm::Crc16(out_buffer_.data(), kPacketLength - 2,
                                                      rm::modules::algorithm::CRC16_INIT),
                        (u16)(out_buffer_[kPacketLength - 2] | out_buffer_[kPacketLength - 1] << 8));
          }
          state_ = DecoderState::kSof;
          idx_ = 0;
        }
      }
    }
  }

  return *this;
}