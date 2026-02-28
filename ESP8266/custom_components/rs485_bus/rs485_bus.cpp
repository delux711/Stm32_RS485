#include "rs485_bus.h"
#include <algorithm>
#include "esphome/core/log.h"

namespace esphome {
namespace rs485_bus {

static const char *const TAG = "rs485_bus";
static constexpr uint8_t FRAME_START = 0xAA;
static constexpr uint8_t MAX_LEN = 64;
static constexpr uint32_t RESPONSE_TIMEOUT_MS = 100;
static constexpr bool REQUIRE_CHECKSUM = false;
static constexpr uint8_t POLL_COMMANDS[] = {0x01, 0x02, 0x03};

RS485Bus::RS485Bus(uart::UARTComponent *parent, GPIOPin *de_pin)
    : uart::UARTDevice(parent), de_pin_(de_pin) {}

void RS485Bus::setup() {
  ESP_LOGI(TAG, "RS485 Bus setup");

  if (de_pin_ != nullptr) {
    de_pin_->setup();
    de_pin_->digital_write(false);
  }

  this->set_interval("rs485_poll", poll_interval_ms_, [this]() { this->poll_once_(); });
}

void RS485Bus::loop() {
  while (available()) {
    uint8_t byte;
    if (read_byte(&byte)) {
      parse_byte_(byte);
    }
  }
}

void RS485Bus::register_temperature_sensor(uint8_t addr, sensor::Sensor *s) {
  temp_sensors_[addr] = s;
}

void RS485Bus::register_humidity_sensor(uint8_t addr, sensor::Sensor *s) {
  hum_sensors_[addr] = s;
}

void RS485Bus::register_pir_sensor(uint8_t addr, binary_sensor::BinarySensor *s) {
  pir_sensors_[addr] = s;
}

void RS485Bus::set_ping_enabled(bool enabled) {
  ping_enabled_ = enabled;

  if (!enabled) {
    if (waiting_for_pong_)
      waiting_for_response_ = false;
    waiting_for_pong_ = false;
    if (pong_status_sensor_ != nullptr)
      pong_status_sensor_->publish_state(false);
  }
}

std::vector<uint8_t> RS485Bus::build_poll_nodes_() const {
  if (!nodes_.empty())
    return nodes_;

  std::vector<uint8_t> result;
  result.reserve(temp_sensors_.size() + hum_sensors_.size() + pir_sensors_.size());

  for (const auto &entry : temp_sensors_)
    result.push_back(entry.first);

  for (const auto &entry : hum_sensors_) {
    if (std::find(result.begin(), result.end(), entry.first) == result.end())
      result.push_back(entry.first);
  }

  for (const auto &entry : pir_sensors_) {
    if (std::find(result.begin(), result.end(), entry.first) == result.end())
      result.push_back(entry.first);
  }

  return result;
}

void RS485Bus::send_poll_frame_(uint8_t addr, uint8_t cmd) {
  const std::vector<uint8_t> frame = {
      FRAME_START,
      addr,
      0x02,
      cmd,
      0x00,
  };

  if (de_pin_ != nullptr)
    de_pin_->digital_write(true);

  this->write_array(frame);
  this->flush();

  if (de_pin_ != nullptr)
    de_pin_->digital_write(false);

  ESP_LOGV(TAG, "TX poll frame addr=%u cmd=0x%02X", addr, cmd);
}

void RS485Bus::send_ping_() {
  static const uint8_t ping[] = {'A', 'A', 'P', 'I', 'N', 'G', '\r', '\n'};

  if (de_pin_ != nullptr)
    de_pin_->digital_write(true);

  this->write_array(ping, sizeof(ping));
  this->flush();

  if (de_pin_ != nullptr)
    de_pin_->digital_write(false);

  ESP_LOGV(TAG, "TX ASCII PING");
}

void RS485Bus::poll_once_() {
  const uint32_t now = millis();
  if (waiting_for_response_) {
    if (now - last_request_ms_ < RESPONSE_TIMEOUT_MS)
      return;

    if (waiting_for_pong_) {
      waiting_for_pong_ = false;
      if (pong_status_sensor_ != nullptr)
        pong_status_sensor_->publish_state(false);
      ESP_LOGW(TAG, "PING timeout - no PONG received");
    }

    waiting_for_response_ = false;
    ESP_LOGV(TAG, "Response timeout (%ums), continuing with next request", RESPONSE_TIMEOUT_MS);
  }

  if (ping_enabled_ && send_ping_next_) {
    this->send_ping_();
    last_request_ms_ = now;
    waiting_for_response_ = true;
    waiting_for_pong_ = true;
    send_ping_next_ = false;
    return;
  }

  const auto poll_nodes = this->build_poll_nodes_();
  if (poll_nodes.empty()) {
    send_ping_next_ = ping_enabled_;
    return;
  }

  if (poll_node_index_ >= poll_nodes.size())
    poll_node_index_ = 0;

  if (poll_cmd_index_ >= sizeof(POLL_COMMANDS))
    poll_cmd_index_ = 0;

  const uint8_t addr = poll_nodes[poll_node_index_];
  const uint8_t cmd = POLL_COMMANDS[poll_cmd_index_];
  this->send_poll_frame_(addr, cmd);
  last_request_ms_ = now;
  waiting_for_response_ = true;
  send_ping_next_ = ping_enabled_;

  poll_cmd_index_++;
  if (poll_cmd_index_ >= sizeof(POLL_COMMANDS)) {
    poll_cmd_index_ = 0;
    poll_node_index_++;
  }
}

void RS485Bus::parse_ascii_byte_(uint8_t byte) {
  pong_window_[0] = pong_window_[1];
  pong_window_[1] = pong_window_[2];
  pong_window_[2] = pong_window_[3];
  pong_window_[3] = byte;

  if (waiting_for_pong_ && pong_window_[0] == 'P' && pong_window_[1] == 'O' && pong_window_[2] == 'N' && pong_window_[3] == 'G') {
    waiting_for_response_ = false;
    waiting_for_pong_ = false;
    if (pong_status_sensor_ != nullptr)
      pong_status_sensor_->publish_state(true);
    ESP_LOGI(TAG, "RX ASCII PONG");
  }
}

bool RS485Bus::is_allowed_node_(uint8_t addr) const {
  if (nodes_.empty())
    return true;

  for (auto node : nodes_) {
    if (node == addr)
      return true;
  }
  return false;
}

bool RS485Bus::validate_checksum_(const std::vector<uint8_t> &frame) const {
  if (frame.size() < 2)
    return false;

  uint8_t checksum = 0;
  for (size_t index = 0; index < frame.size() - 1; index++) {
    checksum = static_cast<uint8_t>(checksum + frame[index]);
  }
  return checksum == frame.back();
}

void RS485Bus::parse_byte_(uint8_t byte) {
  this->parse_ascii_byte_(byte);
  rx_buffer_.push_back(byte);

  while (!rx_buffer_.empty()) {
    if (rx_buffer_[0] != FRAME_START) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    if (rx_buffer_.size() < 4)
      return;

    const uint8_t len = rx_buffer_[2];
    if (len < 2 || len > MAX_LEN) {
      ESP_LOGW(TAG, "Invalid frame len=%u, dropping start byte", len);
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    const size_t frame_size = static_cast<size_t>(len) + 3;
    if (rx_buffer_.size() < frame_size)
      return;

    const std::vector<uint8_t> frame(rx_buffer_.begin(), rx_buffer_.begin() + frame_size);

    if (REQUIRE_CHECKSUM && !validate_checksum_(frame)) {
      ESP_LOGW(TAG, "Checksum mismatch, frame dropped");
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    const uint8_t addr = frame[1];
    const uint8_t cmd = frame[3];

    if (!is_allowed_node_(addr)) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_size);
      continue;
    }

    waiting_for_response_ = false;

    if (cmd == 0x01 && frame_size >= 6) {
      auto sensor_it = temp_sensors_.find(addr);
      if (sensor_it != temp_sensors_.end()) {
        const float temp = (static_cast<uint16_t>(frame[4]) << 8 | frame[5]) / 100.0f;
        sensor_it->second->publish_state(temp);
      }
    }

    if (cmd == 0x02 && frame_size >= 6) {
      auto sensor_it = hum_sensors_.find(addr);
      if (sensor_it != hum_sensors_.end()) {
        const float hum = (static_cast<uint16_t>(frame[4]) << 8 | frame[5]) / 100.0f;
        sensor_it->second->publish_state(hum);
      }
    }

    if (cmd == 0x03 && frame_size >= 5) {
      auto sensor_it = pir_sensors_.find(addr);
      if (sensor_it != pir_sensors_.end()) {
        sensor_it->second->publish_state(frame[4] != 0);
      }
    }

    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_size);
  }
}

void RS485PingSwitch::write_state(bool state) {
  if (bus_ != nullptr)
    bus_->set_ping_enabled(state);
  publish_state(state);
}

void RS485PingSwitch::setup() {
  if (bus_ != nullptr) {
    publish_state(bus_->is_ping_enabled());
    return;
  }
  publish_state(true);
}

}  // namespace rs485_bus
}  // namespace esphome
