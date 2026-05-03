#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <map>
#include <string>
#include <vector>
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/gpio.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace rs485_bus {

class RS485PingSwitch;

class RS485Bus : public Component, public uart::UARTDevice {
 public:
  RS485Bus(uart::UARTComponent *parent, GPIOPin *de_pin);

  void setup() override;
  void loop() override;

  void register_temperature_sensor(uint8_t addr, sensor::Sensor *s);
  void register_humidity_sensor(uint8_t addr, sensor::Sensor *s);
  void register_pir_sensor(uint8_t addr, binary_sensor::BinarySensor *s);
  void set_pong_status_sensor(binary_sensor::BinarySensor *s) { pong_status_sensor_ = s; }
  void set_poll_interval(uint32_t interval_ms) { poll_interval_ms_ = interval_ms; }
  void set_ping_enabled(bool enabled);
  bool is_ping_enabled() const { return ping_enabled_; }

  void set_nodes(const std::vector<uint8_t> &nodes) {
    nodes_ = nodes;
  }

 private:
  std::vector<uint8_t> nodes_;
  uint32_t poll_interval_ms_{1000};
  uint32_t last_request_ms_{0};
  bool waiting_for_response_{false};
  bool waiting_for_pong_{false};
  bool send_ping_next_{true};
  bool ping_enabled_{true};
  size_t poll_node_index_{0};
  size_t poll_cmd_index_{0};
  uint8_t active_request_addr_{0};
  std::array<uint8_t, 4> pong_window_{{0, 0, 0, 0}};
  std::string ascii_line_buffer_;

  void poll_once_();
  void send_ascii_command_(uint8_t addr, const char *cmd);
  void send_ping_(uint8_t addr);
  std::vector<uint8_t> build_poll_nodes_() const;
  void parse_ascii_byte_(uint8_t byte);
  void parse_ascii_line_(const std::string &line);
  void publish_key_value_(uint8_t addr, const std::string &key, const std::string &value);
  bool parse_float_(const std::string &value, float *out) const;
  bool parse_bool_(const std::string &value, bool *out) const;

  bool is_allowed_node_(uint8_t addr) const;
  bool validate_checksum_(const std::vector<uint8_t> &frame) const;

 protected:
  GPIOPin *de_pin_{nullptr};
  std::vector<uint8_t> rx_buffer_;

  std::map<uint8_t, sensor::Sensor *> temp_sensors_;
  std::map<uint8_t, sensor::Sensor *> hum_sensors_;
  std::map<uint8_t, binary_sensor::BinarySensor *> pir_sensors_;
  binary_sensor::BinarySensor *pong_status_sensor_{nullptr};

  void parse_byte_(uint8_t byte);
};

class RS485PingSwitch : public switch_::Switch, public Component {
 public:
  void set_bus(RS485Bus *bus) { bus_ = bus; }
  void setup() override;

 protected:
  void write_state(bool state) override;

 private:
  RS485Bus *bus_{nullptr};
};

}  // namespace rs485_bus
}  // namespace esphome

