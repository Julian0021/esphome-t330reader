#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {
namespace t330reader {

class T330Reader : public PollingComponent,
                   public uart::UARTDevice,
                   public api::CustomAPIDevice {
 public:
  using Sensor = sensor::Sensor;
  using TextSensor = text_sensor::TextSensor;

  void set_uart_out(uart::UARTComponent *uart_out) {
    this->uart_out_ = uart_out;
  }

  void set_energy_kwh_sensor(Sensor *sensor) {
    this->energy_kwh_sensor_ = sensor;
  }
  void set_volume_m3_sensor(Sensor *sensor) {
    this->volume_m3_sensor_ = sensor;
  }
  void set_power_kw_sensor(Sensor *sensor) { this->power_kw_sensor_ = sensor; }
  void set_flow_rate_m3h_sensor(Sensor *sensor) {
    this->flow_rate_m3h_sensor_ = sensor;
  }
  void set_flow_temperature_c_sensor(Sensor *sensor) {
    this->flow_temperature_c_sensor_ = sensor;
  }
  void set_return_temperature_c_sensor(Sensor *sensor) {
    this->return_temperature_c_sensor_ = sensor;
  }
  void set_temperature_difference_k_sensor(Sensor *sensor) {
    this->temperature_difference_k_sensor_ = sensor;
  }
  void set_on_time_h_sensor(Sensor *sensor) {
    this->on_time_h_sensor_ = sensor;
  }
  void set_operating_time_h_sensor(Sensor *sensor) {
    this->operating_time_h_sensor_ = sensor;
  }
  void set_activity_duration_s_sensor(Sensor *sensor) {
    this->activity_duration_s_sensor_ = sensor;
  }
  void set_averaging_duration_s_sensor(Sensor *sensor) {
    this->averaging_duration_s_sensor_ = sensor;
  }
  void set_access_number_sensor(Sensor *sensor) {
    this->access_number_sensor_ = sensor;
  }
  void set_status_sensor(Sensor *sensor) { this->status_sensor_ = sensor; }
  void set_error_status_sensor(Sensor *sensor) {
    this->error_status_sensor_ = sensor;
  }
  void set_max_power_kw_sensor(Sensor *sensor) {
    this->max_power_kw_sensor_ = sensor;
  }
  void set_max_flow_rate_m3h_sensor(Sensor *sensor) {
    this->max_flow_rate_m3h_sensor_ = sensor;
  }
  void set_max_flow_temperature_c_sensor(Sensor *sensor) {
    this->max_flow_temperature_c_sensor_ = sensor;
  }
  void set_max_return_temperature_c_sensor(Sensor *sensor) {
    this->max_return_temperature_c_sensor_ = sensor;
  }
  void set_previous_energy_kwh_sensor(Sensor *sensor) {
    this->previous_energy_kwh_sensor_ = sensor;
  }
  void set_previous_volume_m3_sensor(Sensor *sensor) {
    this->previous_volume_m3_sensor_ = sensor;
  }
  void set_previous_on_time_h_sensor(Sensor *sensor) {
    this->previous_on_time_h_sensor_ = sensor;
  }
  void set_previous_operating_time_h_sensor(Sensor *sensor) {
    this->previous_operating_time_h_sensor_ = sensor;
  }

  void set_version_string_sensor(TextSensor *sensor) {
    this->version_string_sensor_ = sensor;
  }
  void set_mbus_address_sensor(TextSensor *sensor) {
    this->mbus_address_sensor_ = sensor;
  }
  void set_fabrication_number_sensor(TextSensor *sensor) {
    this->fabrication_number_sensor_ = sensor;
  }
  void set_meter_datetime_sensor(TextSensor *sensor) {
    this->meter_datetime_sensor_ = sensor;
  }
  void set_max_power_datetime_sensor(TextSensor *sensor) {
    this->max_power_datetime_sensor_ = sensor;
  }
  void set_max_flow_rate_datetime_sensor(TextSensor *sensor) {
    this->max_flow_rate_datetime_sensor_ = sensor;
  }
  void set_max_flow_temperature_datetime_sensor(TextSensor *sensor) {
    this->max_flow_temperature_datetime_sensor_ = sensor;
  }
  void set_max_return_temperature_datetime_sensor(TextSensor *sensor) {
    this->max_return_temperature_datetime_sensor_ = sensor;
  }

  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void update() override;
  void dump_config() override;

 protected:
  enum class ErrorStatus : uint8_t {
    OK = 0,
    INPUT_BAUD_2400_FAILED = 1,
    HANDSHAKE_FAILED = 2,
    INPUT_BAUD_9600_FAILED = 3,
    NO_DATA_PACKETS = 4,
    NO_VALID_PACKET_DECODED = 5,
  };

  void read_meter_();
  bool perform_handshake_(std::string &version_string);
  bool read_all_data_packets_(std::vector<std::vector<uint8_t>> &packets);

  void send_command_(const uint8_t *frame, size_t frame_len);
  bool read_packet_(std::vector<uint8_t> &packet, uint32_t timeout_ms);
  bool read_exact_(std::vector<uint8_t> &bytes,
                   size_t len,
                   uint32_t timeout_ms);
  void clear_input_buffer_();
  bool set_input_baud_(uint32_t baud_rate);

  bool decode_data_packet_(const std::vector<uint8_t> &packet);
  bool decode_records_(const std::vector<uint8_t> &payload);
  bool decode_record_(const std::vector<uint8_t> &payload, size_t &offset);

  static bool is_printable_ascii_(uint8_t byte);
  static std::string format_bcd_bytes_(const uint8_t *data, size_t len);
  static uint64_t decode_bcd_(const uint8_t *data, size_t len);
  static uint64_t decode_uint_le_(const uint8_t *data, size_t len);
  static bool decode_datetime_cp32_(uint32_t raw, std::string &datetime);
  static uint32_t duration_to_seconds_(uint64_t value, uint8_t unit_code);

  void publish_error_status_(ErrorStatus status);
  void publish_numeric_(sensor::Sensor *sensor, float value);
  void publish_text_(text_sensor::TextSensor *sensor, const std::string &value);

  uart::UARTComponent *uart_out_{nullptr};

  Sensor *energy_kwh_sensor_{nullptr};
  Sensor *volume_m3_sensor_{nullptr};
  Sensor *power_kw_sensor_{nullptr};
  Sensor *flow_rate_m3h_sensor_{nullptr};
  Sensor *flow_temperature_c_sensor_{nullptr};
  Sensor *return_temperature_c_sensor_{nullptr};
  Sensor *temperature_difference_k_sensor_{nullptr};
  Sensor *on_time_h_sensor_{nullptr};
  Sensor *operating_time_h_sensor_{nullptr};
  Sensor *activity_duration_s_sensor_{nullptr};
  Sensor *averaging_duration_s_sensor_{nullptr};
  Sensor *access_number_sensor_{nullptr};
  Sensor *status_sensor_{nullptr};
  Sensor *error_status_sensor_{nullptr};
  Sensor *max_power_kw_sensor_{nullptr};
  Sensor *max_flow_rate_m3h_sensor_{nullptr};
  Sensor *max_flow_temperature_c_sensor_{nullptr};
  Sensor *max_return_temperature_c_sensor_{nullptr};
  Sensor *previous_energy_kwh_sensor_{nullptr};
  Sensor *previous_volume_m3_sensor_{nullptr};
  Sensor *previous_on_time_h_sensor_{nullptr};
  Sensor *previous_operating_time_h_sensor_{nullptr};

  TextSensor *version_string_sensor_{nullptr};
  TextSensor *mbus_address_sensor_{nullptr};
  TextSensor *fabrication_number_sensor_{nullptr};
  TextSensor *meter_datetime_sensor_{nullptr};
  TextSensor *max_power_datetime_sensor_{nullptr};
  TextSensor *max_flow_rate_datetime_sensor_{nullptr};
  TextSensor *max_flow_temperature_datetime_sensor_{nullptr};
  TextSensor *max_return_temperature_datetime_sensor_{nullptr};

  static const size_t PREAMBLE_ZERO_COUNT = 240;
  static const uint32_t COMMAND_TIMEOUT_MS = 1500;
  static const uint32_t DATA_FIRST_PACKET_TIMEOUT_MS = 2000;
  static const uint32_t DATA_NEXT_PACKET_TIMEOUT_MS = 2000;
  static const uint32_t POST_SWITCH_SETTLE_MS = 1500;
  static const int SEQ1_RETRIES_AFTER_FIRST = 10;
  static const int SEQ2_RETRIES_AFTER_FIRST = 5;
  static const int SEQ3_RETRIES_AFTER_FIRST = 2;
  static const int DATA_FIRST_PACKET_RETRIES_AFTER_FIRST = 4;
  static const size_t MAX_PACKET_SIZE = 2048;
};

}  // namespace t330reader
}  // namespace esphome
