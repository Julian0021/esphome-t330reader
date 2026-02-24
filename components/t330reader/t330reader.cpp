#include "t330reader.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "esphome/core/log.h"

namespace esphome {
namespace t330reader {

static const char *const TAG = "t330reader";

static const uint8_t SEQ_READ_VERSION[] = {
    0x68, 0x05, 0x05, 0x68, 0x73, 0xFE, 0x51, 0x0F, 0x0F, 0xE0, 0x16,
};

static const uint8_t SEQ_APP_RESET[] = {
    0x68, 0x04, 0x04, 0x68, 0x53, 0xFE, 0x50, 0x00, 0xA1, 0x16,
};

static const uint8_t SEQ_REQ_78[] = {
    0x68, 0x07, 0x07, 0x68, 0x73, 0xFE, 0x51,
    0x0F, 0x70, 0x00, 0x01, 0x42, 0x16,
};

static const uint8_t SEQ_SWITCH_9600[] = {
    0x10, 0x7C, 0xFE, 0x7A, 0x16,
};

static bool is_valid_long_frame_packet(const std::vector<uint8_t> &packet) {
  if (packet.size() < 6 || packet[0] != 0x68) {
    return false;
  }
  if (packet[1] != packet[2] || packet[3] != 0x68) {
    return false;
  }
  const size_t expected_size = static_cast<size_t>(packet[1]) + 6U;
  if (packet.size() != expected_size) {
    return false;
  }

  uint8_t checksum = 0;
  for (size_t i = 4; i < (4U + packet[1]); i++) {
    checksum = static_cast<uint8_t>(checksum + packet[i]);
  }
  return checksum == packet[4 + packet[1]] && packet[5 + packet[1]] == 0x16;
}

static bool is_seq3_confirmation_packet(const std::vector<uint8_t> &packet) {
  if (!is_valid_long_frame_packet(packet)) {
    return false;
  }
  if (packet.size() != 11 || packet[1] != 0x05) {
    return false;
  }
  // Strict sequence-3 confirmation:
  // 68 05 05 68 08 00 78 0F 00 8F 16
  return packet[4] == 0x08 && packet[5] == 0x00 && packet[6] == 0x78 &&
         packet[7] == 0x0F && packet[8] == 0x00;
}

static bool parse_long_frame_summary_(const std::vector<uint8_t> &packet,
                                      uint8_t &control,
                                      uint8_t &ci,
                                      uint8_t &access_number,
                                      uint8_t &first_dif,
                                      uint8_t &checksum_calc,
                                      uint8_t &checksum_rx) {
  if (packet.size() < 6 || packet[0] != 0x68) {
    return false;
  }
  if (packet[1] != packet[2] || packet[3] != 0x68) {
    return false;
  }
  const size_t expected_size = static_cast<size_t>(packet[1]) + 6U;
  if (packet.size() != expected_size) {
    return false;
  }

  const uint8_t len = packet[1];
  checksum_calc = 0;
  for (size_t i = 0; i < len; i++) {
    checksum_calc = static_cast<uint8_t>(checksum_calc + packet[4 + i]);
  }
  checksum_rx = packet[4 + len];
  control = packet[4];
  ci = (len > 2) ? packet[6] : 0;
  access_number = (len > 11) ? packet[15] : 0;
  first_dif = (len > 15) ? packet[19] : 0;
  return true;
}

static void log_long_frame_summary_(const std::vector<uint8_t> &packet,
                                    unsigned index,
                                    size_t stream_offset) {
  uint8_t control = 0;
  uint8_t ci = 0;
  uint8_t access_number = 0;
  uint8_t first_dif = 0;
  uint8_t checksum_calc = 0;
  uint8_t checksum_rx = 0;
  if (!parse_long_frame_summary_(packet, control, ci, access_number, first_dif,
                                 checksum_calc, checksum_rx)) {
    ESP_LOGD(TAG,
             "Frame %u @ stream offset %u: invalid long frame format (size=%u)",
             index, static_cast<unsigned>(stream_offset),
             static_cast<unsigned>(packet.size()));
    return;
  }

  const bool checksum_ok =
      (checksum_calc == checksum_rx) && (packet.back() == 0x16);
  ESP_LOGD(TAG,
           "Frame %u @ stream offset %u: len=%u C=0x%02X CI=0x%02X access=0x%02X "
           "first_dif=0x%02X checksum=%s (calc=0x%02X frame=0x%02X)",
           index, static_cast<unsigned>(stream_offset),
           static_cast<unsigned>(packet[1]), static_cast<unsigned>(control),
           static_cast<unsigned>(ci), static_cast<unsigned>(access_number),
           static_cast<unsigned>(first_dif), checksum_ok ? "OK" : "BAD",
           static_cast<unsigned>(checksum_calc),
           static_cast<unsigned>(checksum_rx));
}

void T330Reader::setup() {
  if (this->uart_out_ == nullptr) {
    ESP_LOGE(TAG, "No output UART configured");
    this->mark_failed();
    return;
  }

  this->clear_input_buffer_();
  this->register_service(&T330Reader::read_meter_, "start_read_meter");
}

void T330Reader::update() { this->read_meter_(); }

void T330Reader::dump_config() {
  ESP_LOGCONFIG(TAG, "T330Reader:");
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
  }
  LOG_UPDATE_INTERVAL(this);

  LOG_SENSOR("  ", "Energy (kWh)", this->energy_kwh_sensor_);
  LOG_SENSOR("  ", "Volume (m3)", this->volume_m3_sensor_);
  LOG_SENSOR("  ", "Power (kW)", this->power_kw_sensor_);
  LOG_SENSOR("  ", "Flow Rate (m3/h)", this->flow_rate_m3h_sensor_);
  LOG_SENSOR("  ", "Flow Temperature (C)", this->flow_temperature_c_sensor_);
  LOG_SENSOR("  ", "Return Temperature (C)",
             this->return_temperature_c_sensor_);
  LOG_SENSOR("  ", "Temperature Difference (K)",
             this->temperature_difference_k_sensor_);
  LOG_SENSOR("  ", "On Time (h)", this->on_time_h_sensor_);
  LOG_SENSOR("  ", "Operating Time (h)", this->operating_time_h_sensor_);
  LOG_SENSOR("  ", "Activity Duration (s)", this->activity_duration_s_sensor_);
  LOG_SENSOR("  ", "Averaging Duration (s)",
             this->averaging_duration_s_sensor_);
  LOG_SENSOR("  ", "Access Number", this->access_number_sensor_);
  LOG_SENSOR("  ", "Status", this->status_sensor_);
  LOG_SENSOR("  ", "Error Status", this->error_status_sensor_);
  LOG_SENSOR("  ", "Max Power (kW)", this->max_power_kw_sensor_);
  LOG_SENSOR("  ", "Max Flow Rate (m3/h)", this->max_flow_rate_m3h_sensor_);
  LOG_SENSOR("  ", "Max Flow Temperature (C)",
             this->max_flow_temperature_c_sensor_);
  LOG_SENSOR("  ", "Max Return Temperature (C)",
             this->max_return_temperature_c_sensor_);
  LOG_SENSOR("  ", "Previous Energy (kWh)", this->previous_energy_kwh_sensor_);
  LOG_SENSOR("  ", "Previous Volume (m3)", this->previous_volume_m3_sensor_);
  LOG_SENSOR("  ", "Previous On Time (h)", this->previous_on_time_h_sensor_);
  LOG_SENSOR("  ", "Previous Operating Time (h)",
             this->previous_operating_time_h_sensor_);

  LOG_TEXT_SENSOR("  ", "Version String", this->version_string_sensor_);
  LOG_TEXT_SENSOR("  ", "M-Bus Address", this->mbus_address_sensor_);
  LOG_TEXT_SENSOR("  ", "Fabrication Number", this->fabrication_number_sensor_);
  LOG_TEXT_SENSOR("  ", "Meter Date Time", this->meter_datetime_sensor_);
  LOG_TEXT_SENSOR("  ", "Max Power Date Time",
                  this->max_power_datetime_sensor_);
  LOG_TEXT_SENSOR("  ", "Max Flow Rate Date Time",
                  this->max_flow_rate_datetime_sensor_);
  LOG_TEXT_SENSOR("  ", "Max Flow Temperature Date Time",
                  this->max_flow_temperature_datetime_sensor_);
  LOG_TEXT_SENSOR("  ", "Max Return Temperature Date Time",
                  this->max_return_temperature_datetime_sensor_);

  this->check_uart_settings(2400, 1, uart::UART_CONFIG_PARITY_EVEN, 8);
}

void T330Reader::clear_input_buffer_() {
  size_t total = 0;

  uint8_t byte;
  while (this->read_byte(&byte)) {
    total++;
  }

  if (total > 0) {
    ESP_LOGD(TAG, "Cleared %u stale UART byte(s) from input buffer",
             static_cast<unsigned>(total));
  }
}

bool T330Reader::set_input_baud_(uint32_t baud_rate) {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Input UART is not configured");
    return false;
  }

  this->parent_->set_baud_rate(baud_rate);
#if defined(USE_ESP8266) || defined(USE_ESP32)
  this->parent_->load_settings(false);
#endif
  return true;
}

void T330Reader::send_command_(const uint8_t *frame, size_t frame_len) {
  if (this->uart_out_ == nullptr) {
    return;
  }

  std::array<uint8_t, PREAMBLE_ZERO_COUNT> preamble{};
  this->uart_out_->write_array(preamble.data(), preamble.size());
  this->uart_out_->write_array(frame, frame_len);
  this->uart_out_->flush();
}

bool T330Reader::read_exact_(std::vector<uint8_t> &bytes,
                             size_t len,
                             uint32_t timeout_ms) {
  bytes.clear();
  bytes.reserve(len);

  uint32_t started_at = millis();
  while (bytes.size() < len) {
    uint8_t value = 0;
    if (this->read_byte(&value)) {
      bytes.push_back(value);
      started_at = millis();
      continue;
    }
    if ((millis() - started_at) > timeout_ms) {
      return false;
    }
    delay(1);
  }
  return true;
}

bool T330Reader::read_packet_(std::vector<uint8_t> &packet,
                              uint32_t timeout_ms) {
  packet.clear();
  uint32_t started_at = millis();

  while ((millis() - started_at) <= timeout_ms) {
    int next = this->read();
    if (next < 0) {
      delay(1);
      continue;
    }

    uint8_t first = static_cast<uint8_t>(next);
    started_at = millis();
    if (first == 0x00) {
      continue;
    }

    if (first == 0xE5) {
      packet.push_back(first);
      ESP_LOGD(TAG, "read_packet_: received ACK frame");
      return true;
    }

    if (first == 0x10) {
      packet.push_back(first);
      std::vector<uint8_t> rest;
      if (!this->read_exact_(rest, 4, timeout_ms)) {
        ESP_LOGD(TAG,
                 "read_packet_: short frame start received but timed out while "
                 "reading remainder (%u ms timeout)",
                 static_cast<unsigned>(timeout_ms));
        return false;
      }
      packet.insert(packet.end(), rest.begin(), rest.end());
      ESP_LOGD(TAG, "read_packet_: received short frame (%u bytes)",
               static_cast<unsigned>(packet.size()));
      return true;
    }

    if (first == 0x68) {
      packet.push_back(first);
      std::vector<uint8_t> header_rest;
      if (!this->read_exact_(header_rest, 3, timeout_ms)) {
        ESP_LOGD(TAG,
                 "read_packet_: long frame start received but timed out while "
                 "reading header (%u ms timeout)",
                 static_cast<unsigned>(timeout_ms));
        return false;
      }
      packet.insert(packet.end(), header_rest.begin(), header_rest.end());

      const uint8_t len_1 = header_rest[0];
      const uint8_t len_2 = header_rest[1];
      const uint8_t frame_marker = header_rest[2];
      if (len_1 != len_2 || frame_marker != 0x68) {
        ESP_LOGW(TAG,
                 "Invalid long frame header (len1=0x%02X len2=0x%02X marker=0x%02X)",
                 static_cast<unsigned>(len_1), static_cast<unsigned>(len_2),
                 static_cast<unsigned>(frame_marker));
        return false;
      }

      if ((static_cast<size_t>(len_1) + 6U) > MAX_PACKET_SIZE) {
        ESP_LOGW(TAG, "Frame too large: %u", static_cast<unsigned>(len_1));
        return false;
      }

      std::vector<uint8_t> body_and_tail;
      if (!this->read_exact_(body_and_tail,
                             static_cast<size_t>(len_1) + 2U, timeout_ms)) {
        ESP_LOGD(TAG,
                 "read_packet_: long frame header read but timed out while "
                 "reading body/tail (L=%u, timeout=%u ms)",
                 static_cast<unsigned>(len_1),
                 static_cast<unsigned>(timeout_ms));
        return false;
      }
      packet.insert(packet.end(), body_and_tail.begin(), body_and_tail.end());
      ESP_LOGD(TAG, "read_packet_: received long frame (%u bytes)",
               static_cast<unsigned>(packet.size()));
      return true;
    }

    // Keep searching for a valid frame start marker. Treat unexpected leading
    // bytes as line noise so we don't swallow a following valid long frame.
    ESP_LOGV(TAG,
             "read_packet_: skipping non-frame byte 0x%02X while waiting for "
             "frame start",
             static_cast<unsigned>(first));
    continue;
  }

  ESP_LOGD(TAG, "read_packet_: timeout after %u ms with no complete frame",
           static_cast<unsigned>(timeout_ms));
  return false;
}

bool T330Reader::is_printable_ascii_(uint8_t byte) {
  return byte >= 32 && byte <= 126;
}

bool T330Reader::perform_handshake_(std::string &version_string) {
  version_string.clear();

  for (int attempt = 0; attempt <= SEQ1_RETRIES_AFTER_FIRST; attempt++) {
    this->clear_input_buffer_();
    this->send_command_(SEQ_READ_VERSION, sizeof(SEQ_READ_VERSION));

    std::vector<uint8_t> packet;
    if (!this->read_packet_(packet, COMMAND_TIMEOUT_MS)) {
      continue;
    }

    size_t scan_offset = 0;
    size_t scan_size = packet.size();
    if (packet.size() >= 8 && packet[0] == 0x68 && packet[1] == packet[2] &&
        packet.size() == static_cast<size_t>(packet[1]) + 6U) {
      scan_offset = 4;
      scan_size = packet[1];
      if (scan_size >= 3) {
        scan_offset += 3;
        scan_size -= 3;
      }
    }

    std::string current;
    std::string longest;
    for (size_t i = 0; i < scan_size; i++) {
      const uint8_t byte = packet[scan_offset + i];
      if (is_printable_ascii_(byte)) {
        current.push_back(static_cast<char>(byte));
      } else if (!current.empty()) {
        if (current.size() > longest.size()) {
          longest = current;
        }
        current.clear();
      }
    }
    if (!current.empty() && current.size() > longest.size()) {
      longest = current;
    }

    if (!longest.empty()) {
      version_string = longest;
      break;
    }
  }

  if (version_string.empty()) {
    ESP_LOGE(TAG, "No version string found in sequence 1");
    return false;
  }

  bool reset_ack = false;
  for (int attempt = 0; attempt <= SEQ2_RETRIES_AFTER_FIRST; attempt++) {
    this->clear_input_buffer_();
    this->send_command_(SEQ_APP_RESET, sizeof(SEQ_APP_RESET));

    std::vector<uint8_t> packet;
    if (!this->read_packet_(packet, COMMAND_TIMEOUT_MS)) {
      continue;
    }
    if (std::find(packet.begin(), packet.end(), 0xE5) != packet.end()) {
      reset_ack = true;
      break;
    }
  }

  if (!reset_ack) {
    ESP_LOGE(TAG, "No ACK (0xE5) received for sequence 2");
    return false;
  }

  bool seq3_ok = false;
  for (int attempt = 0; attempt <= SEQ3_RETRIES_AFTER_FIRST; attempt++) {
    this->clear_input_buffer_();
    this->send_command_(SEQ_REQ_78, sizeof(SEQ_REQ_78));

    std::vector<uint8_t> packet;
    if (!this->read_packet_(packet, COMMAND_TIMEOUT_MS)) {
      continue;
    }
    if (is_seq3_confirmation_packet(packet)) {
      seq3_ok = true;
      break;
    }
    ESP_LOGD(TAG,
             "Sequence 3 attempt %d returned unexpected frame; strict "
             "confirmation not met (size=%u first=0x%02X)",
             attempt + 1, static_cast<unsigned>(packet.size()),
             static_cast<unsigned>(packet.empty() ? 0 : packet[0]));
  }

  if (!seq3_ok) {
    ESP_LOGE(TAG, "No sequence 3 confirmation frame received");
    return false;
  }

  this->clear_input_buffer_();
  this->send_command_(SEQ_SWITCH_9600, sizeof(SEQ_SWITCH_9600));
  return true;
}

bool T330Reader::capture_data_stream_(std::vector<uint8_t> &stream) {
  stream.clear();
  stream.reserve(1024);

  const uint32_t started_at = millis();
  uint32_t last_rx_at = started_at;
  bool saw_data = false;
  bool truncated = false;

  while ((millis() - started_at) <= DATA_CAPTURE_WINDOW_MS) {
    uint8_t byte = 0;
    if (this->read_byte(&byte)) {
      if (stream.size() < MAX_CAPTURE_BYTES) {
        stream.push_back(byte);
      } else {
        truncated = true;
      }
      saw_data = true;
      last_rx_at = millis();
      continue;
    }

    if (saw_data && (millis() - last_rx_at) >= DATA_CAPTURE_IDLE_TIMEOUT_MS) {
      break;
    }
    delay(1);
  }

  const uint32_t elapsed = millis() - started_at;
  if (!saw_data) {
    ESP_LOGD(TAG,
             "Data capture timeout: no data seen within %u ms capture window",
             static_cast<unsigned>(DATA_CAPTURE_WINDOW_MS));
    return false;
  }

  ESP_LOGD(TAG,
           "Captured %u data byte(s) in %u ms (idle timeout=%u ms, "
           "capture window=%u ms)%s",
           static_cast<unsigned>(stream.size()), static_cast<unsigned>(elapsed),
           static_cast<unsigned>(DATA_CAPTURE_IDLE_TIMEOUT_MS),
           static_cast<unsigned>(DATA_CAPTURE_WINDOW_MS),
           truncated ? "; stream truncated" : "");
  if (truncated) {
    ESP_LOGW(TAG,
             "Captured stream exceeded %u bytes; trailing data was dropped",
             static_cast<unsigned>(MAX_CAPTURE_BYTES));
  }
  return true;
}

void T330Reader::extract_long_frames_from_stream_(
    const std::vector<uint8_t> &stream,
    std::vector<std::vector<uint8_t>> &packets) {
  packets.clear();

  size_t noise_bytes = 0;
  size_t offset = 0;
  while ((offset + 4U) <= stream.size()) {
    if (stream[offset] != 0x68) {
      noise_bytes++;
      offset++;
      continue;
    }

    const uint8_t len_1 = stream[offset + 1];
    const uint8_t len_2 = stream[offset + 2];
    const uint8_t frame_marker = stream[offset + 3];
    if (len_1 != len_2 || frame_marker != 0x68) {
      noise_bytes++;
      offset++;
      continue;
    }

    const size_t frame_size = static_cast<size_t>(len_1) + 6U;
    if (frame_size > MAX_PACKET_SIZE) {
      ESP_LOGW(TAG, "Discarding oversized long frame at stream offset %u (L=%u)",
               static_cast<unsigned>(offset), static_cast<unsigned>(len_1));
      noise_bytes++;
      offset++;
      continue;
    }
    if ((offset + frame_size) > stream.size()) {
      ESP_LOGD(TAG,
               "Trailing partial long frame at stream offset %u (need %u bytes, "
               "have %u)",
               static_cast<unsigned>(offset), static_cast<unsigned>(frame_size),
               static_cast<unsigned>(stream.size() - offset));
      break;
    }

    std::vector<uint8_t> packet(
        stream.begin() + static_cast<std::ptrdiff_t>(offset),
        stream.begin() + static_cast<std::ptrdiff_t>(offset + frame_size));
    log_long_frame_summary_(packet, static_cast<unsigned>(packets.size() + 1U),
                            offset);
    if (is_valid_long_frame_packet(packet)) {
      packets.push_back(std::move(packet));
    } else {
      ESP_LOGW(TAG,
               "Discarding long frame at stream offset %u: checksum/end marker "
               "invalid",
               static_cast<unsigned>(offset));
    }

    offset += frame_size;
  }

  if (noise_bytes > 0) {
    ESP_LOGD(TAG, "Skipped %u non-frame/noise byte(s) while parsing stream",
             static_cast<unsigned>(noise_bytes));
  }
}

bool T330Reader::read_all_data_packets_(
    std::vector<std::vector<uint8_t>> &packets) {
  packets.clear();

  std::vector<uint8_t> stream;
  if (!this->capture_data_stream_(stream)) {
    return false;
  }

  this->extract_long_frames_from_stream_(stream, packets);
  ESP_LOGD(TAG, "Data retrieval complete: captured %u valid long frame(s)",
           static_cast<unsigned>(packets.size()));
  return !packets.empty();
}

uint64_t T330Reader::decode_uint_le_(const uint8_t *data, size_t len) {
  uint64_t value = 0;
  for (size_t i = 0; i < len; i++) {
    value |= static_cast<uint64_t>(data[i]) << (8U * i);
  }
  return value;
}

uint64_t T330Reader::decode_bcd_(const uint8_t *data, size_t len) {
  uint64_t value = 0;
  uint64_t factor = 1;
  for (size_t i = 0; i < len; i++) {
    const uint8_t low = data[i] & 0x0F;
    const uint8_t high = (data[i] >> 4) & 0x0F;
    if (low <= 9) {
      value += static_cast<uint64_t>(low) * factor;
      factor *= 10;
    }
    if (high <= 9) {
      value += static_cast<uint64_t>(high) * factor;
      factor *= 10;
    }
  }
  return value;
}

std::string T330Reader::format_bcd_bytes_(const uint8_t *data, size_t len) {
  std::string out;
  out.reserve(len * 2);
  for (int i = static_cast<int>(len) - 1; i >= 0; i--) {
    const uint8_t high = (data[i] >> 4) & 0x0F;
    const uint8_t low = data[i] & 0x0F;
    out.push_back(static_cast<char>('0' + (high <= 9 ? high : 0)));
    out.push_back(static_cast<char>('0' + (low <= 9 ? low : 0)));
  }
  return out;
}

bool T330Reader::decode_datetime_cp32_(uint32_t raw, std::string &datetime) {
  const uint8_t minute = raw & 0x3F;
  const uint8_t hour = (raw >> 8) & 0x1F;
  const uint8_t day = (raw >> 16) & 0x1F;
  const uint8_t month = (raw >> 24) & 0x0F;
  const uint16_t year_offset =
      ((raw >> 21) & 0x07) | (((raw >> 28) & 0x0F) << 3);
  const uint16_t year = 2000 + year_offset;

  if (minute > 59 || hour > 23 || day < 1 || day > 31 || month < 1 ||
      month > 12) {
    return false;
  }

  char buffer[24];
  std::snprintf(buffer, sizeof(buffer), "%04u-%02u-%02u %02u:%02u", year,
                month, day, hour, minute);
  datetime.assign(buffer);
  return true;
}

uint32_t T330Reader::duration_to_seconds_(uint64_t value, uint8_t unit_code) {
  switch (unit_code) {
    case 0:
      return static_cast<uint32_t>(value);
    case 1:
      return static_cast<uint32_t>(value * 60ULL);
    case 2:
      return static_cast<uint32_t>(value * 3600ULL);
    case 3:
      return static_cast<uint32_t>(value * 86400ULL);
    default:
      return static_cast<uint32_t>(value);
  }
}

void T330Reader::publish_numeric_(sensor::Sensor *sensor, float value) {
  if (sensor != nullptr) {
    sensor->publish_state(value);
  }
}

void T330Reader::publish_error_status_(ErrorStatus status) {
  this->publish_numeric_(
      this->error_status_sensor_,
      static_cast<float>(static_cast<uint8_t>(status)));
}

void T330Reader::publish_text_(text_sensor::TextSensor *sensor,
                               const std::string &value) {
  if (sensor != nullptr) {
    sensor->publish_state(value);
  }
}

bool T330Reader::decode_record_(const std::vector<uint8_t> &payload,
                                size_t &offset) {
  if (offset >= payload.size()) {
    ESP_LOGD(TAG, "Decode record failed: offset %u out of range (size=%u)",
             static_cast<unsigned>(offset),
             static_cast<unsigned>(payload.size()));
    return false;
  }

  size_t i = offset;
  const uint8_t dif = payload[i++];
  const uint8_t data_field = dif & 0x0F;
  const uint8_t function_field = (dif >> 4) & 0x03;
  uint32_t storage = (dif >> 6) & 0x01;
  uint32_t tariff = 0;
  uint32_t subunit = 0;

  bool has_dife = (dif & 0x80) != 0;
  for (uint8_t dife_idx = 0; has_dife; dife_idx++) {
    if (i >= payload.size() || dife_idx > 10) {
      ESP_LOGD(TAG,
               "Decode record failed: invalid DIFE chain at payload index %u "
               "(dife_idx=%u, size=%u)",
               static_cast<unsigned>(i), static_cast<unsigned>(dife_idx),
               static_cast<unsigned>(payload.size()));
      return false;
    }
    const uint8_t dife = payload[i++];
    has_dife = (dife & 0x80) != 0;
    subunit += ((dife >> 6) & 0x01) << dife_idx;
    tariff += ((dife >> 4) & 0x03) << (dife_idx * 2);
    storage += static_cast<uint32_t>(dife & 0x0F) << (1 + dife_idx * 4);
  }

  if (i >= payload.size()) {
    ESP_LOGD(TAG, "Decode record failed: missing VIF at payload index %u",
             static_cast<unsigned>(i));
    return false;
  }

  const uint8_t vif_raw = payload[i++];
  const uint8_t vif = vif_raw & 0x7F;
  bool has_vife = (vif_raw & 0x80) != 0;
  std::vector<uint8_t> vifes;
  for (uint8_t vife_idx = 0; has_vife; vife_idx++) {
    if (i >= payload.size() || vife_idx > 10) {
      ESP_LOGD(TAG,
               "Decode record failed: invalid VIFE chain at payload index %u "
               "(vife_idx=%u, size=%u)",
               static_cast<unsigned>(i), static_cast<unsigned>(vife_idx),
               static_cast<unsigned>(payload.size()));
      return false;
    }
    const uint8_t vife = payload[i++];
    has_vife = (vife & 0x80) != 0;
    vifes.push_back(vife & 0x7F);
  }

  if (data_field == 15) {
    const uint8_t special = dif & 0x7F;
    if (special == 0x0F || special == 0x1F) {
      offset = payload.size();
      return true;
    }
    offset = i;
    return true;
  }

  size_t data_len = 0;
  switch (data_field) {
    case 0:
      data_len = 0;
      break;
    case 1:
      data_len = 1;
      break;
    case 2:
      data_len = 2;
      break;
    case 3:
      data_len = 3;
      break;
    case 4:
      data_len = 4;
      break;
    case 5:
      data_len = 4;
      break;
    case 6:
      data_len = 6;
      break;
    case 7:
      data_len = 8;
      break;
    case 8:
      data_len = 0;
      break;
    case 9:
      data_len = 1;
      break;
    case 10:
      data_len = 2;
      break;
    case 11:
      data_len = 3;
      break;
    case 12:
      data_len = 4;
      break;
    case 13:
      if (i >= payload.size()) {
        ESP_LOGD(TAG,
                 "Decode record failed: missing LVAR byte at payload index %u",
                 static_cast<unsigned>(i));
        return false;
      }
      data_len = payload[i++];
      break;
    case 14:
      data_len = 6;
      break;
    default:
      ESP_LOGD(TAG, "Decode record failed: unsupported data field %u",
               static_cast<unsigned>(data_field));
      return false;
  }

  if ((i + data_len) > payload.size()) {
    ESP_LOGD(TAG,
             "Decode record failed: data length overflow "
             "(index=%u, data_len=%u, size=%u)",
             static_cast<unsigned>(i), static_cast<unsigned>(data_len),
             static_cast<unsigned>(payload.size()));
    return false;
  }

  const uint8_t *raw_data = payload.data() + i;
  i += data_len;
  offset = i;

  if (data_field == 0 || data_field == 8 || data_len == 0) {
    return true;
  }

  bool datetime_from_vife = false;
  for (uint8_t vife : vifes) {
    if ((vife & 0x7A) == 0x6A || vife == 0x39) {
      datetime_from_vife = true;
      break;
    }
  }

  enum class Metric {
    UNKNOWN,
    ENERGY_KWH,
    VOLUME_M3,
    POWER_KW,
    FLOW_M3H,
    FLOW_TEMP_C,
    RETURN_TEMP_C,
    TEMP_DIFF_K,
    ON_TIME,
    OPERATING_TIME,
    AVG_DURATION,
    ACT_DURATION,
    DATETIME,
    FAB_NO,
  };

  Metric metric = Metric::UNKNOWN;
  double scale = 1.0;
  uint8_t duration_unit_code = 0;

  if (vif <= 0x07) {
    metric = Metric::ENERGY_KWH;
    scale = std::pow(10.0, static_cast<int>(vif & 0x07) - 6);
  } else if ((vif & 0xF8) == 0x10) {
    metric = Metric::VOLUME_M3;
    scale = std::pow(10.0, static_cast<int>(vif & 0x07) - 6);
  } else if ((vif & 0xF8) == 0x28) {
    metric = Metric::POWER_KW;
    scale = std::pow(10.0, static_cast<int>(vif & 0x07) - 3);
  } else if ((vif & 0xF8) == 0x38) {
    metric = Metric::FLOW_M3H;
    scale = std::pow(10.0, static_cast<int>(vif & 0x07) - 6);
  } else if ((vif & 0xFC) == 0x58) {
    metric = Metric::FLOW_TEMP_C;
    scale = std::pow(10.0, static_cast<int>(vif & 0x03) - 3);
  } else if ((vif & 0xFC) == 0x5C) {
    metric = Metric::RETURN_TEMP_C;
    scale = std::pow(10.0, static_cast<int>(vif & 0x03) - 3);
  } else if ((vif & 0xFC) == 0x60) {
    metric = Metric::TEMP_DIFF_K;
    scale = std::pow(10.0, static_cast<int>(vif & 0x03) - 3);
  } else if ((vif & 0xFC) == 0x20) {
    metric = Metric::ON_TIME;
    duration_unit_code = vif & 0x03;
  } else if ((vif & 0xFC) == 0x24) {
    metric = Metric::OPERATING_TIME;
    duration_unit_code = vif & 0x03;
  } else if ((vif & 0xFC) == 0x70) {
    metric = Metric::AVG_DURATION;
    duration_unit_code = vif & 0x03;
  } else if ((vif & 0xFC) == 0x74) {
    metric = Metric::ACT_DURATION;
    duration_unit_code = vif & 0x03;
  } else if ((vif & 0xFE) == 0x6C) {
    metric = Metric::DATETIME;
  } else if (vif == 0x78) {
    metric = Metric::FAB_NO;
  }

  bool is_datetime = metric == Metric::DATETIME || datetime_from_vife;
  if (is_datetime && data_len >= 4) {
    std::string datetime;
    if (decode_datetime_cp32_(
            static_cast<uint32_t>(decode_uint_le_(raw_data, 4)), datetime)) {
      if (metric == Metric::DATETIME && storage == 0 && tariff == 0 &&
          subunit == 0) {
        this->publish_text_(this->meter_datetime_sensor_, datetime);
      } else if (metric == Metric::POWER_KW && function_field == 1 &&
                 tariff == 1 && storage == 0) {
        this->publish_text_(this->max_power_datetime_sensor_, datetime);
      } else if (metric == Metric::FLOW_M3H && function_field == 1 &&
                 tariff == 1 && storage == 0) {
        this->publish_text_(this->max_flow_rate_datetime_sensor_, datetime);
      } else if (metric == Metric::FLOW_TEMP_C && function_field == 1 &&
                 tariff == 1 && storage == 0) {
        this->publish_text_(this->max_flow_temperature_datetime_sensor_,
                            datetime);
      } else if (metric == Metric::RETURN_TEMP_C && function_field == 1 &&
                 tariff == 1 && storage == 0) {
        this->publish_text_(this->max_return_temperature_datetime_sensor_,
                            datetime);
      }
    }
    return true;
  }

  if (metric == Metric::FAB_NO) {
    this->publish_text_(this->fabrication_number_sensor_,
                        format_bcd_bytes_(raw_data, data_len));
    return true;
  }

  uint64_t raw_value = 0;
  double decoded_value = 0.0;
  switch (data_field) {
    case 1:
    case 2:
    case 3:
    case 4:
    case 6:
    case 7:
      raw_value = decode_uint_le_(raw_data, data_len);
      decoded_value = static_cast<double>(raw_value);
      break;
    case 5: {
      uint32_t raw_float = static_cast<uint32_t>(decode_uint_le_(raw_data, 4));
      float converted = 0.0f;
      std::memcpy(&converted, &raw_float, sizeof(converted));
      decoded_value = static_cast<double>(converted);
      raw_value = static_cast<uint64_t>(std::llround(decoded_value));
      break;
    }
    case 9:
    case 10:
    case 11:
    case 12:
    case 14:
      raw_value = decode_bcd_(raw_data, data_len);
      decoded_value = static_cast<double>(raw_value);
      break;
    case 13:
      return true;
    default:
      return true;
  }

  const double scaled = decoded_value * scale;

  switch (metric) {
    case Metric::ENERGY_KWH:
      if (function_field == 0 && tariff == 0 && subunit == 0 && storage == 0) {
        this->publish_numeric_(this->energy_kwh_sensor_,
                               static_cast<float>(scaled));
      } else if (function_field == 0 && tariff == 0 && subunit == 0 &&
                 storage == 1) {
        this->publish_numeric_(this->previous_energy_kwh_sensor_,
                               static_cast<float>(scaled));
      }
      break;
    case Metric::VOLUME_M3:
      if (function_field == 0 && tariff == 0 && subunit == 0 && storage == 0) {
        this->publish_numeric_(this->volume_m3_sensor_,
                               static_cast<float>(scaled));
      } else if (function_field == 0 && tariff == 0 && subunit == 0 &&
                 storage == 1) {
        this->publish_numeric_(this->previous_volume_m3_sensor_,
                               static_cast<float>(scaled));
      }
      break;
    case Metric::POWER_KW:
      if (function_field == 0 && tariff == 0 && subunit == 0 && storage == 0) {
        this->publish_numeric_(this->power_kw_sensor_,
                               static_cast<float>(scaled));
      } else if (function_field == 1 && tariff == 1 && subunit == 0 &&
                 storage == 0) {
        this->publish_numeric_(this->max_power_kw_sensor_,
                               static_cast<float>(scaled / 1000.0));
      }
      break;
    case Metric::FLOW_M3H:
      if (function_field == 0 && tariff == 0 && subunit == 0 && storage == 0) {
        this->publish_numeric_(this->flow_rate_m3h_sensor_,
                               static_cast<float>(scaled));
      } else if (function_field == 1 && tariff == 1 && subunit == 0 &&
                 storage == 0) {
        this->publish_numeric_(this->max_flow_rate_m3h_sensor_,
                               static_cast<float>(scaled));
      }
      break;
    case Metric::FLOW_TEMP_C:
      if (function_field == 0 && tariff == 0 && subunit == 0 && storage == 0) {
        this->publish_numeric_(this->flow_temperature_c_sensor_,
                               static_cast<float>(scaled));
      } else if (function_field == 1 && tariff == 1 && subunit == 0 &&
                 storage == 0) {
        this->publish_numeric_(this->max_flow_temperature_c_sensor_,
                               static_cast<float>(scaled));
      }
      break;
    case Metric::RETURN_TEMP_C:
      if (function_field == 0 && tariff == 0 && subunit == 0 && storage == 0) {
        this->publish_numeric_(this->return_temperature_c_sensor_,
                               static_cast<float>(scaled));
      } else if (function_field == 1 && tariff == 1 && subunit == 0 &&
                 storage == 0) {
        this->publish_numeric_(this->max_return_temperature_c_sensor_,
                               static_cast<float>(scaled));
      }
      break;
    case Metric::TEMP_DIFF_K:
      if (function_field == 0 && tariff == 0 && subunit == 0 && storage == 0) {
        this->publish_numeric_(this->temperature_difference_k_sensor_,
                               static_cast<float>(scaled));
      }
      break;
    case Metric::ON_TIME: {
      const float hours = static_cast<float>(
          duration_to_seconds_(raw_value, duration_unit_code) / 3600.0);
      if (function_field == 0 && tariff == 0 && subunit == 0 && storage == 0) {
        this->publish_numeric_(this->on_time_h_sensor_, hours);
      } else if (function_field == 0 && tariff == 0 && subunit == 0 &&
                 storage == 1) {
        this->publish_numeric_(this->previous_on_time_h_sensor_, hours);
      }
      break;
    }
    case Metric::OPERATING_TIME: {
      const float hours = static_cast<float>(
          duration_to_seconds_(raw_value, duration_unit_code) / 3600.0);
      if (function_field == 0 && tariff == 0 && subunit == 0 && storage == 0) {
        this->publish_numeric_(this->operating_time_h_sensor_, hours);
      } else if (function_field == 0 && tariff == 0 && subunit == 0 &&
                 storage == 1) {
        this->publish_numeric_(this->previous_operating_time_h_sensor_, hours);
      }
      break;
    }
    case Metric::AVG_DURATION:
      if (function_field == 0 && tariff == 0 && subunit == 0 && storage == 0) {
        this->publish_numeric_(
            this->averaging_duration_s_sensor_,
            static_cast<float>(duration_to_seconds_(raw_value,
                                                    duration_unit_code)));
      }
      break;
    case Metric::ACT_DURATION:
      if (function_field == 0 && tariff == 0 && subunit == 0 && storage == 0) {
        this->publish_numeric_(
            this->activity_duration_s_sensor_,
            static_cast<float>(duration_to_seconds_(raw_value,
                                                    duration_unit_code)));
      }
      break;
    case Metric::UNKNOWN:
    case Metric::DATETIME:
    case Metric::FAB_NO:
      break;
  }

  return true;
}

bool T330Reader::decode_records_(const std::vector<uint8_t> &payload) {
  if (payload.size() <= 15) {
    ESP_LOGD(TAG, "Decode records failed: payload too short (%u)",
             static_cast<unsigned>(payload.size()));
    return false;
  }

  size_t offset = 15;
  while (offset < payload.size()) {
    const size_t previous = offset;
    if (!this->decode_record_(payload, offset)) {
      ESP_LOGD(TAG,
               "Decode records failed: decode_record_ returned false at "
               "offset %u",
               static_cast<unsigned>(previous));
      return false;
    }
    if (offset <= previous) {
      ESP_LOGD(TAG,
               "Decode records failed: parser made no progress "
               "(offset=%u -> %u)",
               static_cast<unsigned>(previous), static_cast<unsigned>(offset));
      return false;
    }
  }
  return true;
}

bool T330Reader::decode_data_packet_(const std::vector<uint8_t> &packet) {
  if (packet.size() < 8 || packet[0] != 0x68) {
    ESP_LOGD(TAG,
             "Decode packet failed: invalid frame start (size=%u, first=0x%02X)",
             static_cast<unsigned>(packet.size()),
             static_cast<unsigned>(packet.empty() ? 0 : packet[0]));
    return false;
  }

  const uint8_t len_1 = packet[1];
  const uint8_t len_2 = packet[2];
  if (len_1 != len_2 || packet[3] != 0x68) {
    ESP_LOGD(TAG,
             "Decode packet failed: invalid long frame header "
             "(len1=%u, len2=%u, marker=0x%02X)",
             static_cast<unsigned>(len_1), static_cast<unsigned>(len_2),
             static_cast<unsigned>(packet[3]));
    return false;
  }

  const size_t expected_size = static_cast<size_t>(len_1) + 6U;
  if (packet.size() != expected_size) {
    ESP_LOGD(TAG,
             "Decode packet failed: size mismatch (got=%u, expected=%u)",
             static_cast<unsigned>(packet.size()),
             static_cast<unsigned>(expected_size));
    return false;
  }

  const uint8_t *data = packet.data() + 4;
  uint8_t checksum = 0;
  for (size_t i = 0; i < len_1; i++) {
    checksum = static_cast<uint8_t>(checksum + data[i]);
  }
  if (checksum != packet[4 + len_1] || packet[5 + len_1] != 0x16) {
    ESP_LOGD(TAG,
             "Decode packet failed: checksum/terminator mismatch "
             "(checksum=0x%02X, frame=0x%02X, end=0x%02X)",
             static_cast<unsigned>(checksum),
             static_cast<unsigned>(packet[4 + len_1]),
             static_cast<unsigned>(packet[5 + len_1]));
    return false;
  }

  const uint8_t control = data[0] & 0x0F;
  if (control != 0x08) {
    ESP_LOGD(TAG, "Decode packet failed: unexpected control 0x%02X",
             static_cast<unsigned>(control));
    return false;
  }
  if (len_1 < 15 || (data[2] != 0x72 && data[2] != 0x76)) {
    ESP_LOGD(TAG,
             "Decode packet failed: unsupported CI/length (len=%u, CI=0x%02X)",
             static_cast<unsigned>(len_1), static_cast<unsigned>(data[2]));
    return false;
  }

  this->publish_text_(this->mbus_address_sensor_,
                      format_bcd_bytes_(data + 3, 4));
  this->publish_numeric_(this->access_number_sensor_, data[11]);
  this->publish_numeric_(this->status_sensor_, data[12]);

  std::vector<uint8_t> payload(data, data + len_1);
  const bool ok = this->decode_records_(payload);
  if (!ok) {
    ESP_LOGD(TAG, "Decode packet failed: record parsing failed");
  }
  return ok;
}

void T330Reader::read_meter_() {
  if (!this->set_input_baud_(2400)) {
    ESP_LOGD(TAG, "Read summary: captured_frames=0 decoded_frames=0 error_status=%u",
             static_cast<unsigned>(
                 static_cast<uint8_t>(ErrorStatus::INPUT_BAUD_2400_FAILED)));
    this->publish_error_status_(ErrorStatus::INPUT_BAUD_2400_FAILED);
    return;
  }
  this->clear_input_buffer_();

  std::string version_string;
  if (!this->perform_handshake_(version_string)) {
    ESP_LOGD(TAG, "Read summary: captured_frames=0 decoded_frames=0 error_status=%u",
             static_cast<unsigned>(
                 static_cast<uint8_t>(ErrorStatus::HANDSHAKE_FAILED)));
    this->publish_error_status_(ErrorStatus::HANDSHAKE_FAILED);
    this->set_input_baud_(2400);
    return;
  }
  ESP_LOGD(TAG, "Handshake successful, meter version: %s",
           version_string.c_str());
  this->publish_text_(this->version_string_sensor_, version_string);

  ESP_LOGD(TAG,
           "Entering data phase: switching input UART to 9600 immediately after "
           "CI=0x7A");
  if (!this->set_input_baud_(9600)) {
    ESP_LOGD(TAG, "Read summary: captured_frames=0 decoded_frames=0 error_status=%u",
             static_cast<unsigned>(
                 static_cast<uint8_t>(ErrorStatus::INPUT_BAUD_9600_FAILED)));
    this->publish_error_status_(ErrorStatus::INPUT_BAUD_9600_FAILED);
    this->set_input_baud_(2400);
    return;
  }
  if (POST_SWITCH_SETTLE_MS > 0) {
    ESP_LOGD(TAG, "Waiting %u ms for baud switch settle",
             static_cast<unsigned>(POST_SWITCH_SETTLE_MS));
    delay(POST_SWITCH_SETTLE_MS);
  }
  ESP_LOGD(TAG, "Input UART switched to 9600; starting raw data capture");

  std::vector<std::vector<uint8_t>> packets;
  if (!this->read_all_data_packets_(packets)) {
    ESP_LOGW(TAG, "No T330 data packets received");
    ESP_LOGD(TAG,
             "Publishing error_status=4 (NO_DATA_PACKETS): data retrieval "
             "window completed without accepted long frame");
    ESP_LOGD(TAG, "Read summary: captured_frames=0 decoded_frames=0 error_status=%u",
             static_cast<unsigned>(
                 static_cast<uint8_t>(ErrorStatus::NO_DATA_PACKETS)));
    this->publish_error_status_(ErrorStatus::NO_DATA_PACKETS);
    this->set_input_baud_(2400);
    return;
  }

  size_t decoded = 0;
  for (size_t packet_idx = 0; packet_idx < packets.size(); packet_idx++) {
    const auto &packet = packets[packet_idx];
    if (this->decode_data_packet_(packet)) {
      decoded++;
    } else {
      ESP_LOGD(TAG, "Packet %u decode failed (size=%u)",
               static_cast<unsigned>(packet_idx + 1U),
               static_cast<unsigned>(packet.size()));
    }
  }

  if (decoded == 0) {
    this->publish_error_status_(ErrorStatus::NO_VALID_PACKET_DECODED);
    ESP_LOGD(TAG, "Read summary: captured_frames=%u decoded_frames=0 error_status=%u",
             static_cast<unsigned>(packets.size()),
             static_cast<unsigned>(static_cast<uint8_t>(
                 ErrorStatus::NO_VALID_PACKET_DECODED)));
  } else {
    this->publish_error_status_(ErrorStatus::OK);
    ESP_LOGD(TAG, "Read summary: captured_frames=%u decoded_frames=%u error_status=%u",
             static_cast<unsigned>(packets.size()), static_cast<unsigned>(decoded),
             static_cast<unsigned>(static_cast<uint8_t>(ErrorStatus::OK)));
  }

  ESP_LOGD(TAG, "Decoded %u/%u T330 packets", static_cast<unsigned>(decoded),
           static_cast<unsigned>(packets.size()));
  this->set_input_baud_(2400);
}

}  // namespace t330reader
}  // namespace esphome
