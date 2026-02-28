#include "rs485_bus.h"
#include <algorithm>
#include <cstdlib>
#include "esphome/core/log.h"

namespace esphome {
namespace rs485_bus {

static const char *const TAG = "rs485_bus";
static constexpr uint8_t FRAME_START = 0xAA;
static constexpr uint8_t MAX_LEN = 64;
static constexpr uint32_t RESPONSE_TIMEOUT_MS = 100;
static constexpr bool REQUIRE_CHECKSUM = false;
static constexpr const char *const POLL_COMMANDS[] = {"TEMP", "PIR", "ALL", "PING"};
static constexpr size_t POLL_COMMAND_COUNT = sizeof(POLL_COMMANDS) / sizeof(POLL_COMMANDS[0]);

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

void RS485Bus::send_ascii_command_(uint8_t addr, const char *cmd) {
  std::vector<uint8_t> frame;
  frame.reserve(2 + std::char_traits<char>::length(cmd) + 2);
  frame.push_back(static_cast<uint8_t>(addr & 0xFF));
  frame.push_back(addr);
  for (size_t index = 0; cmd[index] != '\0'; index++)
    frame.push_back(static_cast<uint8_t>(cmd[index]));
  frame.push_back('\r');
  frame.push_back('\n');

  if (de_pin_ != nullptr)
    de_pin_->digital_write(true);

  this->write_array(frame);
  this->flush();

  if (de_pin_ != nullptr)
    de_pin_->digital_write(false);

  ESP_LOGV(TAG, "TX ASCII cmd a0=0x%02X a1=0x%02X cmd=%s", static_cast<uint8_t>(addr & 0xFF), addr, cmd);
}

void RS485Bus::send_ping_(uint8_t addr) {
  this->send_ascii_command_(addr, "PING");
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

  const auto poll_nodes = this->build_poll_nodes_();

  if (ping_enabled_ && send_ping_next_) {
    uint8_t ping_addr = 0;
    if (!poll_nodes.empty()) {
      if (poll_node_index_ >= poll_nodes.size())
        poll_node_index_ = 0;
      ping_addr = poll_nodes[poll_node_index_];
    }

    this->send_ping_(ping_addr);
    active_request_addr_ = ping_addr;
    last_request_ms_ = now;
    waiting_for_response_ = true;
    waiting_for_pong_ = true;
    send_ping_next_ = false;
    return;
  }

  if (poll_nodes.empty()) {
    send_ping_next_ = ping_enabled_;
    return;
  }

  if (poll_node_index_ >= poll_nodes.size())
    poll_node_index_ = 0;

  if (poll_cmd_index_ >= POLL_COMMAND_COUNT)
    poll_cmd_index_ = 0;

  const uint8_t addr = poll_nodes[poll_node_index_];
  const char *cmd = POLL_COMMANDS[poll_cmd_index_];
  this->send_ascii_command_(addr, cmd);
  active_request_addr_ = addr;
  last_request_ms_ = now;
  waiting_for_response_ = true;
  send_ping_next_ = ping_enabled_;

  poll_cmd_index_++;
  if (poll_cmd_index_ >= POLL_COMMAND_COUNT) {
    poll_cmd_index_ = 0;
    poll_node_index_++;
  }
}

void RS485Bus::parse_ascii_byte_(uint8_t byte) {
  if (byte == '\r')
    return;

  if (byte == '\n') {
    if (!ascii_line_buffer_.empty()) {
      this->parse_ascii_line_(ascii_line_buffer_);
      ascii_line_buffer_.clear();
    }
    return;
  }

  if (ascii_line_buffer_.size() >= MAX_LEN) {
    ascii_line_buffer_.clear();
    ESP_LOGW(TAG, "ASCII response too long, dropping line");
    return;
  }

  ascii_line_buffer_.push_back(static_cast<char>(byte));
}

void RS485Bus::parse_ascii_line_(const std::string &line) {
  if (line == "PONG") {
    waiting_for_response_ = false;
    waiting_for_pong_ = false;
    if (pong_status_sensor_ != nullptr)
      pong_status_sensor_->publish_state(true);
    ESP_LOGI(TAG, "RX ASCII PONG");
    return;
  }

  waiting_for_response_ = false;

  size_t token_start = 0;
  while (token_start < line.size()) {
    const size_t token_end = line.find(',', token_start);
    const size_t token_len = (token_end == std::string::npos) ? (line.size() - token_start) : (token_end - token_start);
    const std::string token = line.substr(token_start, token_len);

    const size_t equals_pos = token.find('=');
    if (equals_pos != std::string::npos && equals_pos > 0 && equals_pos + 1 < token.size()) {
      const std::string key = token.substr(0, equals_pos);
      const std::string value = token.substr(equals_pos + 1);
      this->publish_key_value_(active_request_addr_, key, value);
    }

    if (token_end == std::string::npos)
      break;
    token_start = token_end + 1;
  }
}

void RS485Bus::publish_key_value_(uint8_t addr, const std::string &key, const std::string &value) {
  if (key == "TEMP") {
    auto sensor_it = temp_sensors_.find(addr);
    if (sensor_it != temp_sensors_.end()) {
      float temp = 0.0f;
      if (parse_float_(value, &temp))
        sensor_it->second->publish_state(temp);
    }
    return;
  }

  if (key == "HUM") {
    auto sensor_it = hum_sensors_.find(addr);
    if (sensor_it != hum_sensors_.end()) {
      float hum = 0.0f;
      if (parse_float_(value, &hum))
        sensor_it->second->publish_state(hum);
    }
    return;
  }

  if (key == "PIR") {
    auto sensor_it = pir_sensors_.find(addr);
    if (sensor_it != pir_sensors_.end()) {
      bool pir = false;
      if (parse_bool_(value, &pir))
        sensor_it->second->publish_state(pir);
    }
    return;
  }
}

bool RS485Bus::parse_float_(const std::string &value, float *out) const {
  if (out == nullptr)
    return false;

  char *end = nullptr;
  const float parsed = std::strtof(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0')
    return false;

  *out = parsed;
  return true;
}

bool RS485Bus::parse_bool_(const std::string &value, bool *out) const {
  if (out == nullptr)
    return false;

  if (value == "1" || value == "ON" || value == "TRUE") {
    *out = true;
    return true;
  }

  if (value == "0" || value == "OFF" || value == "FALSE") {
    *out = false;
    return true;
  }

  return false;
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
