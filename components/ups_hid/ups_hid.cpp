#include "ups_hid.h"
#include "constants_ups.h"
#include "transport_factory.h"
#include "transport_simulation.h"
#ifdef USE_ESP32
#include "transport_esp32.h"
#endif
#include "protocol_factory.h"
#include "protocol_apc.h"
#include "protocol_cyberpower.h"
#include "protocol_tripplite.h"
#include "protocol_generic.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/time/real_time_clock.h"
#include <functional>
#include <cmath>
#ifdef USE_ESP32
#include "esp_system.h"
#endif

namespace esphome {
namespace ups_hid {

#ifdef USE_ESP32
static const char *get_reset_reason_str() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "Power-on";
    case ESP_RST_EXT:      return "External pin";
    case ESP_RST_SW:       return "Software reset";
    case ESP_RST_PANIC:    return "Exception/panic";
    case ESP_RST_INT_WDT:  return "Interrupt watchdog";
    case ESP_RST_TASK_WDT: return "Task watchdog";
    case ESP_RST_WDT:      return "Other watchdog";
    case ESP_RST_DEEPSLEEP: return "Deep sleep wake";
    case ESP_RST_BROWNOUT: return "Brownout";
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "Unknown";
  }
}
#endif

void UpsHidComponent::setup() {
  ESP_LOGCONFIG(TAG, log_messages::SETTING_UP);

  // Restore event log from previous boot before anything else
  event_log_.load_from_nvs();

#ifdef USE_ESP32
  const char *reason = get_reset_reason_str();
  esp_reset_reason_t reason_code = esp_reset_reason();
  last_reset_reason_ = std::string(reason) + " (" + std::to_string(static_cast<int>(reason_code)) + ")";
  ESP_LOGI(TAG, "Last reset reason: %s", last_reset_reason_.c_str());

  // Record boot event with reset reason (persisted to NVS)
  std::string boot_msg = std::string("Boot: reset=") + reason;
  event_log_.record(format_event_timestamp(), boot_msg);
#endif

  if (!initialize_transport()) {
    ESP_LOGE(TAG, log_messages::TRANSPORT_INIT_FAILED);
    mark_failed();
    return;
  }

  // Launch background task for USB reads so loop() never blocks
  usb_task_running_.store(true);
  xTaskCreatePinnedToCore(
      usb_read_task, "ups_usb_read", 8192, this, 1, &usb_read_task_handle_, 1);
  ESP_LOGI(TAG, "USB read task started on core 1");

  ESP_LOGCONFIG(TAG, log_messages::SETUP_COMPLETE);
}

void UpsHidComponent::update() {
  // update() runs on ESPHome's main loop -- never blocks on USB.
  // The background task does all USB I/O and sets new_data_available_.
  if (new_data_available_.exchange(false)) {
    update_sensors();
    check_state_changes();
    check_and_update_timers();
  }
}

void UpsHidComponent::usb_read_task(void *param) {
  auto *self = static_cast<UpsHidComponent *>(param);
  self->usb_read_loop();
  vTaskDelete(nullptr);
}

void UpsHidComponent::usb_read_loop() {
  while (usb_task_running_.load()) {
    uint32_t interval = get_update_interval();

    if (!transport_ || !transport_->is_connected()) {
      ESP_LOGD(TAG, log_messages::WAITING_FOR_DEVICE);
      vTaskDelay(pdMS_TO_TICKS(interval));
      continue;
    }

    // Protocol detection
    if (!active_protocol_) {
      ESP_LOGI(TAG, log_messages::ATTEMPTING_DETECTION);
      if (detect_protocol()) {
        ESP_LOGI(TAG, log_messages::PROTOCOL_DETECTED);
        consecutive_failures_ = 0;
      } else {
        consecutive_failures_++;
        ESP_LOGW(TAG, log_messages::DETECTION_FAILED, consecutive_failures_);
        if (consecutive_failures_ > max_consecutive_failures_) {
          ESP_LOGE(TAG, log_messages::TOO_MANY_FAILURES);
        }
        vTaskDelay(pdMS_TO_TICKS(interval));
        continue;
      }
    }

    // Read data (this is the slow USB I/O part)
    if (read_ups_data()) {
      new_data_available_.store(true);
      consecutive_failures_ = 0;
      last_successful_read_ = millis();
    } else {
      consecutive_failures_++;
      ESP_LOGW(TAG, log_messages::READ_FAILED, consecutive_failures_);
      if (consecutive_failures_ > max_consecutive_failures_) {
        ESP_LOGW(TAG, log_messages::RESETTING_PROTOCOL);
        active_protocol_.reset();
        report_map_.reset();
        consecutive_failures_ = 0;
      }

      // If no successful read for a long time, reset to avoid serving stale data
      if (last_successful_read_ > 0 &&
          (millis() - last_successful_read_) > DATA_STALE_TIMEOUT_MS) {
        ESP_LOGW(TAG, "No successful read for %us, clearing stale data",
                 DATA_STALE_TIMEOUT_MS / 1000);
        std::lock_guard<std::mutex> lock(data_mutex_);
        DeviceInfo saved_device = ups_data_.device;
        ups_data_.reset();
        ups_data_.device = saved_device;
        last_successful_read_ = 0;
        new_data_available_.store(true);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(interval));
  }
}

void UpsHidComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "UPS HID Component:");
  ESP_LOGCONFIG(TAG, "  Simulation Mode: %s", simulation_mode_ ? status::YES : status::NO);

  if (transport_ && transport_->is_connected()) {
    ESP_LOGCONFIG(TAG, "  USB Vendor ID: 0x%04X", transport_->get_vendor_id());
    ESP_LOGCONFIG(TAG, "  USB Product ID: 0x%04X", transport_->get_product_id());
  }

  ESP_LOGCONFIG(TAG, "  Protocol Timeout: %u ms", protocol_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Protocol Selection: %s", protocol_selection_.c_str());
  ESP_LOGCONFIG(TAG, "  Update Interval: %u ms", get_update_interval());
  ESP_LOGCONFIG(TAG, "  Time Source: %s", time_ != nullptr ? "configured" : "not configured (using uptime)");

  if (transport_ && transport_->is_connected()) {
    ESP_LOGCONFIG(TAG, "  Status: %s", status::CONNECTED);
    if (active_protocol_) {
      ESP_LOGCONFIG(TAG, "  Active Protocol: %s",
                   active_protocol_->get_protocol_name().c_str());
    } else {
      ESP_LOGCONFIG(TAG, "  Protocol Status: %s", status::DETECTION_PENDING);
    }
  } else {
    ESP_LOGCONFIG(TAG, "  Status: %s", status::DISCONNECTED);
  }

#ifdef USE_SENSOR
  ESP_LOGCONFIG(TAG, "  Registered Sensors: %zu", sensors_.size());
#endif
#ifdef USE_BINARY_SENSOR
  ESP_LOGCONFIG(TAG, "  Registered Binary Sensors: %zu", binary_sensors_.size());
#endif
#ifdef USE_TEXT_SENSOR
  ESP_LOGCONFIG(TAG, "  Registered Text Sensors: %zu", text_sensors_.size());
#endif
}

// Transport abstraction methods
esp_err_t UpsHidComponent::hid_get_report(uint8_t report_type, uint8_t report_id,
                                         uint8_t* data, size_t* data_len,
                                         uint32_t timeout_ms) {
  if (!transport_) {
    return ESP_ERR_INVALID_STATE;
  }
  return transport_->hid_get_report(report_type, report_id, data, data_len, timeout_ms);
}

esp_err_t UpsHidComponent::hid_set_report(uint8_t report_type, uint8_t report_id,
                                         const uint8_t* data, size_t data_len,
                                         uint32_t timeout_ms) {
  if (!transport_) {
    return ESP_ERR_INVALID_STATE;
  }
  return transport_->hid_set_report(report_type, report_id, data, data_len, timeout_ms);
}

esp_err_t UpsHidComponent::get_string_descriptor(uint8_t string_index, std::string& result) {
  if (!transport_) {
    return ESP_ERR_INVALID_STATE;
  }
  return transport_->get_string_descriptor(string_index, result);
}

esp_err_t UpsHidComponent::get_hid_report_descriptor(std::vector<uint8_t>& descriptor) {
  if (!transport_) {
    return ESP_ERR_INVALID_STATE;
  }
  return transport_->get_hid_report_descriptor(descriptor);
}

bool UpsHidComponent::fetch_and_parse_report_descriptor() {
  std::vector<uint8_t> raw_descriptor;
  esp_err_t ret = get_hid_report_descriptor(raw_descriptor);
  if (ret != ESP_OK || raw_descriptor.empty()) {
    ESP_LOGW(TAG, "Failed to fetch HID report descriptor: %s",
             ret != ESP_OK ? esp_err_to_name(ret) : "empty descriptor");
    return false;
  }

  ESP_LOGI(TAG, "Fetched HID report descriptor: %zu bytes", raw_descriptor.size());

  // Log raw descriptor bytes for debugging
  std::string hex_dump;
  for (size_t i = 0; i < raw_descriptor.size() && i < 128; i++) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02X ", raw_descriptor[i]);
    hex_dump += buf;
    if ((i + 1) % 16 == 0) {
      ESP_LOGD(TAG, "  Descriptor[%03zu]: %s", i - 15, hex_dump.c_str());
      hex_dump.clear();
    }
  }
  if (!hex_dump.empty()) {
    ESP_LOGD(TAG, "  Descriptor[...]: %s", hex_dump.c_str());
  }
  if (raw_descriptor.size() > 128) {
    ESP_LOGD(TAG, "  ... (%zu more bytes)", raw_descriptor.size() - 128);
  }

  report_map_ = std::make_unique<HidReportMap>();
  if (!report_map_->parse(raw_descriptor.data(), raw_descriptor.size())) {
    ESP_LOGW(TAG, "Failed to parse HID report descriptor");
    report_map_.reset();
    return false;
  }

  report_map_->dump(TAG);
  return true;
}

bool UpsHidComponent::is_connected() const {
  return transport_ && transport_->is_connected();
}

uint16_t UpsHidComponent::get_vendor_id() const {
  return transport_ ? transport_->get_vendor_id() : defaults::AUTO_DETECT_VENDOR_ID;
}

uint16_t UpsHidComponent::get_product_id() const {
  return transport_ ? transport_->get_product_id() : defaults::AUTO_DETECT_PRODUCT_ID;
}

// Core implementation methods
bool UpsHidComponent::initialize_transport() {
  ESP_LOGD(TAG, "Initializing transport layer");

  // Create appropriate transport
  auto transport_type = simulation_mode_ ?
    UsbTransportFactory::SIMULATION :
    UsbTransportFactory::ESP32_HARDWARE;

  transport_ = UsbTransportFactory::create(transport_type, simulation_mode_);

  if (!transport_) {
    ESP_LOGE(TAG, "Failed to create transport instance");
    return false;
  }

  esp_err_t ret = transport_->initialize();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Transport initialization failed: %s", transport_->get_last_error().c_str());
    transport_.reset();
    return false;
  }

  connected_ = transport_->is_connected();

  ESP_LOGI(TAG, "Transport initialized successfully (VID=0x%04X, PID=0x%04X)",
           transport_->get_vendor_id(), transport_->get_product_id());

  return true;
}

bool UpsHidComponent::detect_protocol() {
  if (!transport_ || !transport_->is_connected()) {
    ESP_LOGE(TAG, "Cannot detect protocol - transport not connected");
    return false;
  }

  uint16_t vendor_id = transport_->get_vendor_id();

  if (protocol_selection_ == "auto") {
    // Automatic protocol detection based on vendor ID
    ESP_LOGD(TAG, "Auto-detecting protocol for vendor 0x%04X using factory", vendor_id);
    active_protocol_ = ProtocolFactory::create_for_vendor(vendor_id, this);
  } else {
    // Manual protocol selection via factory
    ESP_LOGD(TAG, "Using manually selected protocol: %s", protocol_selection_.c_str());
    active_protocol_ = ProtocolFactory::create_by_name(protocol_selection_, this);
  }

  if (!active_protocol_) {
    ESP_LOGE(TAG, "Failed to create protocol (selection: %s, vendor: 0x%04X)",
             protocol_selection_.c_str(), vendor_id);
    return false;
  }

  ESP_LOGI(TAG, "Successfully created protocol: %s", active_protocol_->get_protocol_name().c_str());

  // Fetch and parse the HID report descriptor before protocol initialization
  // This makes the parsed descriptor available to protocols during initialize()
  if (!report_map_) {
    if (fetch_and_parse_report_descriptor()) {
      ESP_LOGI(TAG, "HID report descriptor parsed successfully");
    } else {
      ESP_LOGW(TAG, "Could not parse HID report descriptor - protocol will use fallback methods");
    }
  }

  // Initialize the protocol (detection already done by factory)
  if (!active_protocol_->initialize()) {
    ESP_LOGE(TAG, "Protocol initialization failed");
    active_protocol_.reset();
    return false;
  }

  ESP_LOGI(TAG, "Protocol initialized: %s",
           active_protocol_->get_protocol_name().c_str());

  // Set the detected protocol in ups_data_ after successful detection
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ups_data_.device.detected_protocol = active_protocol_->get_protocol_type();
  }

  return true;
}

bool UpsHidComponent::read_ups_data() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for reading data");
    return false;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  // Read into a fresh temporary so partial USB failures produce NAN
  // rather than clobbering previously-good values in ups_data_.
  UpsData new_data;
  new_data.device = ups_data_.device;

  bool success = active_protocol_->read_data(new_data);

  if (success) {
    uint8_t prev_stale = ups_data_.power.status_stale_cycles;
    ups_data_.merge_from(new_data);

    if (ups_data_.power.status_stale_cycles == 0 && prev_stale > 0) {
      ESP_LOGI(TAG, "Status freshly determined after %u stale cycle(s): %s",
               prev_stale, ups_data_.power.status.c_str());
    } else if (ups_data_.power.status_stale_cycles > 0) {
      ESP_LOGW(TAG, "Status not determined this cycle (%u/%u stale), keeping: %s",
               ups_data_.power.status_stale_cycles, PowerData::MAX_STALE_CYCLES,
               ups_data_.power.status.empty() ? "(cleared)" : ups_data_.power.status.c_str());
    }
    ESP_LOGV(TAG, "Successfully read and merged UPS data");
  } else {
    ESP_LOGW(TAG, "Failed to read UPS data via protocol");
  }

  return success;
}

void UpsHidComponent::update_sensors() {
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Check if any sensors are registered - if not, skip sensor updates
  size_t total_sensors = 0;
#ifdef USE_SENSOR
  total_sensors += sensors_.size();
#endif
#ifdef USE_BINARY_SENSOR
  total_sensors += binary_sensors_.size();
#endif
#ifdef USE_TEXT_SENSOR
  total_sensors += text_sensors_.size();
#endif

  if (total_sensors == 0) {
    // Data provider mode - no sensor entities, just providing data for direct access
    ESP_LOGVV(TAG, "Data provider mode: no sensors registered, data available via direct access methods");
    return;
  }

  ESP_LOGVV(TAG, "Updating %zu registered sensor entities", total_sensors);

  // Update all registered sensors with current data
#ifdef USE_SENSOR
  for (auto& sensor_pair : sensors_) {
    const std::string& type = sensor_pair.first;
    sensor::Sensor* sensor = sensor_pair.second;

    // Extract appropriate value based on sensor type
    float value = NAN;

    if (type == sensor_type::BATTERY_LEVEL && ups_data_.battery.is_valid()) {
      value = ups_data_.battery.level;
    } else if (type == sensor_type::BATTERY_VOLTAGE && !std::isnan(ups_data_.battery.voltage)) {
      value = ups_data_.battery.voltage;
    } else if (type == sensor_type::BATTERY_VOLTAGE_NOMINAL && !std::isnan(ups_data_.battery.voltage_nominal)) {
      value = ups_data_.battery.voltage_nominal;
    } else if (type == sensor_type::RUNTIME && !std::isnan(ups_data_.battery.runtime_minutes)) {
      value = ups_data_.battery.runtime_minutes;
    } else if (type == sensor_type::INPUT_VOLTAGE && !std::isnan(ups_data_.power.input_voltage)) {
      value = ups_data_.power.input_voltage;
    } else if (type == sensor_type::INPUT_VOLTAGE_NOMINAL && !std::isnan(ups_data_.power.input_voltage_nominal)) {
      value = ups_data_.power.input_voltage_nominal;
    } else if (type == sensor_type::OUTPUT_VOLTAGE && !std::isnan(ups_data_.power.output_voltage)) {
      value = ups_data_.power.output_voltage;
    } else if (type == sensor_type::LOAD_PERCENT && !std::isnan(ups_data_.power.load_percent)) {
      value = ups_data_.power.load_percent;
    } else if (type == sensor_type::FREQUENCY && !std::isnan(ups_data_.power.frequency)) {
      value = ups_data_.power.frequency;
    } else if (type == sensor_type::INPUT_TRANSFER_LOW && !std::isnan(ups_data_.power.input_transfer_low)) {
      value = ups_data_.power.input_transfer_low;
    } else if (type == sensor_type::INPUT_TRANSFER_HIGH && !std::isnan(ups_data_.power.input_transfer_high)) {
      value = ups_data_.power.input_transfer_high;
    } else if (type == sensor_type::BATTERY_RUNTIME_LOW && !std::isnan(ups_data_.battery.runtime_low)) {
      value = ups_data_.battery.runtime_low;
    } else if (type == sensor_type::OUTPUT_CURRENT && !std::isnan(ups_data_.power.output_current)) {
      value = ups_data_.power.output_current;
    } else if (type == sensor_type::OUTPUT_FREQUENCY && !std::isnan(ups_data_.power.output_frequency)) {
      value = ups_data_.power.output_frequency;
    } else if (type == sensor_type::ACTIVE_POWER && !std::isnan(ups_data_.power.active_power)) {
      value = ups_data_.power.active_power;
    } else if (type == sensor_type::BATTERY_CONFIG_VOLTAGE && !std::isnan(ups_data_.battery.config_voltage)) {
      value = ups_data_.battery.config_voltage;
    } else if (type == sensor_type::BATTERY_FULL_CHARGE_CAPACITY && !std::isnan(ups_data_.battery.full_charge_capacity)) {
      value = ups_data_.battery.full_charge_capacity;
    } else if (type == sensor_type::BATTERY_DESIGN_CAPACITY && !std::isnan(ups_data_.battery.design_capacity)) {
      value = ups_data_.battery.design_capacity;
    } else if (type == sensor_type::UPS_REALPOWER_NOMINAL && !std::isnan(ups_data_.power.realpower_nominal)) {
      value = ups_data_.power.realpower_nominal;
    } else if (type == sensor_type::UPS_DELAY_SHUTDOWN && !std::isnan(ups_data_.config.delay_shutdown)) {
      value = ups_data_.config.delay_shutdown;
    } else if (type == sensor_type::UPS_DELAY_START && !std::isnan(ups_data_.config.delay_start)) {
      value = ups_data_.config.delay_start;
    } else if (type == sensor_type::UPS_DELAY_REBOOT && !std::isnan(ups_data_.config.delay_reboot)) {
      value = ups_data_.config.delay_reboot;
    } else if (type == sensor_type::UPS_TIMER_REBOOT && ups_data_.test.timer_reboot != -1) {
      value = ups_data_.test.timer_reboot;
    } else if (type == sensor_type::UPS_TIMER_SHUTDOWN && ups_data_.test.timer_shutdown != -1) {
      value = ups_data_.test.timer_shutdown;
    } else if (type == sensor_type::UPS_TIMER_START && ups_data_.test.timer_start != -1) {
      value = ups_data_.test.timer_start;
    }

    if (!std::isnan(value)) {
      sensor->publish_state(value);
    }
  }
#endif

  // Update binary sensors
#ifdef USE_BINARY_SENSOR
  for (auto& sensor_pair : binary_sensors_) {
    const std::string& type = sensor_pair.first;
    binary_sensor::BinarySensor* sensor = sensor_pair.second;

    bool state = false;

    if (type == binary_sensor_type::ONLINE) {
      state = ups_data_.power.status == status::ONLINE ||
              ups_data_.power.status == "Online (Boost)" ||
              ups_data_.power.status == "Online (Trim)";
    } else if (type == binary_sensor_type::ON_BATTERY) {
      state = ups_data_.power.status == status::ON_BATTERY;
    } else if (type == binary_sensor_type::LOW_BATTERY) {
      state = ups_data_.battery.is_low();
    } else if (type == binary_sensor_type::OVERLOAD) {
      state = ups_data_.power.is_overloaded() || ups_data_.power.status == "Overload";
    } else if (type == binary_sensor_type::BOOST) {
      state = ups_data_.power.boost_active;
    } else if (type == binary_sensor_type::BUCK) {
      state = ups_data_.power.buck_active;
    } else if (type == binary_sensor_type::CHARGING) {
      state = ups_data_.battery.status == battery_status::CHARGING;
    } else if (type == binary_sensor_type::DISCHARGING) {
      state = ups_data_.battery.status == battery_status::DISCHARGING;
    } else if (type == binary_sensor_type::FULLY_DISCHARGED) {
      state = ups_data_.battery.status == "Depleted";
    } else if (type == binary_sensor_type::OVER_TEMPERATURE) {
      state = ups_data_.power.over_temperature;
    } else if (type == binary_sensor_type::COMMUNICATION_LOST) {
      state = ups_data_.power.communication_lost;
    } else if (type == binary_sensor_type::SHUTDOWN_IMMINENT) {
      state = ups_data_.power.shutdown_imminent;
    } else if (type == binary_sensor_type::AWAITING_POWER) {
      state = ups_data_.power.awaiting_power;
    } else if (type == binary_sensor_type::VOLTAGE_OUT_OF_RANGE) {
      state = ups_data_.power.voltage_out_of_range;
    }

    sensor->publish_state(state);
  }
#endif

  // Update text sensors
#ifdef USE_TEXT_SENSOR
  for (auto& sensor_pair : text_sensors_) {
    const std::string& type = sensor_pair.first;
    text_sensor::TextSensor* sensor = sensor_pair.second;

    std::string value = "";

    if (type == text_sensor_type::MODEL && !ups_data_.device.model.empty()) {
      value = ups_data_.device.model;
    } else if (type == text_sensor_type::MANUFACTURER && !ups_data_.device.manufacturer.empty()) {
      value = ups_data_.device.manufacturer;
    } else if (type == text_sensor_type::SERIAL_NUMBER && !ups_data_.device.serial_number.empty()) {
      value = ups_data_.device.serial_number;
    } else if (type == text_sensor_type::FIRMWARE_VERSION && !ups_data_.device.firmware_version.empty()) {
      value = ups_data_.device.firmware_version;
    } else if (type == text_sensor_type::BATTERY_STATUS && !ups_data_.battery.status.empty()) {
      value = ups_data_.battery.status;
    } else if (type == text_sensor_type::UPS_TEST_RESULT && !ups_data_.test.ups_test_result.empty()) {
      value = ups_data_.test.ups_test_result;
    } else if (type == text_sensor_type::UPS_BEEPER_STATUS && !ups_data_.config.beeper_status.empty()) {
      value = ups_data_.config.beeper_status;
    } else if (type == text_sensor_type::INPUT_SENSITIVITY && !ups_data_.config.input_sensitivity.empty()) {
      value = ups_data_.config.input_sensitivity;
    } else if (type == text_sensor_type::STATUS && !ups_data_.power.status.empty()) {
      value = ups_data_.power.status;
    } else if (type == text_sensor_type::PROTOCOL) {
      value = get_protocol_name();
    } else if (type == text_sensor_type::BATTERY_MFR_DATE && !ups_data_.battery.mfr_date.empty()) {
      value = ups_data_.battery.mfr_date;
    } else if (type == text_sensor_type::UPS_MFR_DATE && !ups_data_.device.mfr_date.empty()) {
      value = ups_data_.device.mfr_date;
    } else if (type == text_sensor_type::BATTERY_TYPE && !ups_data_.battery.type.empty()) {
      value = ups_data_.battery.type;
    } else if (type == text_sensor_type::UPS_FIRMWARE_AUX && !ups_data_.device.firmware_aux.empty()) {
      value = ups_data_.device.firmware_aux;
    }

    if (!value.empty()) {
      sensor->publish_state(value);
    }
  }
#endif

  // Update delay number components
  for (auto* delay_number : delay_numbers_) {
    if (delay_number != nullptr) {
      // Determine which delay value to use based on the delay type
      // This will be handled by the number component itself
      // We just need to trigger an update with the current config values
      // The number component will query the appropriate value
    }
  }

  // Log sensor counts (conditional on platform availability)
#ifdef USE_SENSOR
  size_t sensor_count = sensors_.size();
#else
  size_t sensor_count = 0;
#endif
#ifdef USE_BINARY_SENSOR
  size_t binary_sensor_count = binary_sensors_.size();
#else
  size_t binary_sensor_count = 0;
#endif
#ifdef USE_TEXT_SENSOR
  size_t text_sensor_count = text_sensors_.size();
#else
  size_t text_sensor_count = 0;
#endif

  ESP_LOGV(TAG, "Updated %zu sensors, %zu binary sensors, %zu text sensors",
           sensor_count, binary_sensor_count, text_sensor_count);
}

// Sensor registration methods (conditional on platform availability)
#ifdef USE_SENSOR
void UpsHidComponent::register_sensor(sensor::Sensor *sens, const std::string &type) {
  sensors_[type] = sens;
  ESP_LOGD(TAG, "Registered sensor: %s", type.c_str());
}
#endif

#ifdef USE_BINARY_SENSOR
void UpsHidComponent::register_binary_sensor(binary_sensor::BinarySensor *sens, const std::string &type) {
  binary_sensors_[type] = sens;
  ESP_LOGD(TAG, "Registered binary sensor: %s", type.c_str());
}
#endif

#ifdef USE_TEXT_SENSOR
void UpsHidComponent::register_text_sensor(text_sensor::TextSensor *sens, const std::string &type) {
  text_sensors_[type] = sens;
  ESP_LOGD(TAG, "Registered text sensor: %s", type.c_str());
}
#endif

void UpsHidComponent::register_delay_number(UpsDelayNumber *number) {
  delay_numbers_.push_back(number);
  ESP_LOGD(TAG, "Registered delay number component");
}

// Test control methods
bool UpsHidComponent::start_battery_test_quick() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for battery test");
    return false;
  }
  return active_protocol_->start_battery_test_quick();
}

bool UpsHidComponent::start_battery_test_deep() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for battery test");
    return false;
  }
  return active_protocol_->start_battery_test_deep();
}

bool UpsHidComponent::stop_battery_test() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for battery test");
    return false;
  }
  return active_protocol_->stop_battery_test();
}

bool UpsHidComponent::start_ups_test() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for UPS test");
    return false;
  }
  return active_protocol_->start_ups_test();
}

bool UpsHidComponent::stop_ups_test() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for UPS test");
    return false;
  }
  return active_protocol_->stop_ups_test();
}

// Beeper control methods
bool UpsHidComponent::beeper_enable() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for beeper control");
    return false;
  }
  return active_protocol_->beeper_enable();
}

bool UpsHidComponent::beeper_disable() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for beeper control");
    return false;
  }
  return active_protocol_->beeper_disable();
}

bool UpsHidComponent::beeper_mute() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for beeper control");
    return false;
  }
  return active_protocol_->beeper_mute();
}

bool UpsHidComponent::beeper_test() {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for beeper control");
    return false;
  }
  return active_protocol_->beeper_test();
}

// Delay configuration methods
bool UpsHidComponent::set_shutdown_delay(int seconds) {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for delay configuration");
    return false;
  }
  return active_protocol_->set_shutdown_delay(seconds);
}

bool UpsHidComponent::set_start_delay(int seconds) {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for delay configuration");
    return false;
  }
  return active_protocol_->set_start_delay(seconds);
}

bool UpsHidComponent::set_reboot_delay(int seconds) {
  if (!active_protocol_) {
    ESP_LOGW(TAG, "No active protocol for delay configuration");
    return false;
  }
  return active_protocol_->set_reboot_delay(seconds);
}

// Additional protocol access method
std::string UpsHidComponent::get_protocol_name() const {
  if (active_protocol_) {
    return active_protocol_->get_protocol_name();
  }
  return protocol::NONE;
}


// Error rate limiting helpers
bool UpsHidComponent::should_log_error(ErrorRateLimit& limiter) {
  uint32_t now = millis();

  if (limiter.error_count < ErrorRateLimit::MAX_BURST) {
    limiter.error_count++;
    limiter.last_error_time = now;
    return true;
  }

  if (now - limiter.last_error_time > ErrorRateLimit::RATE_LIMIT_MS) {
    limiter.error_count = 1;
    limiter.suppressed_count = 0;
    limiter.last_error_time = now;
    return true;
  }

  limiter.suppressed_count++;
  return false;
}

void UpsHidComponent::log_suppressed_errors(ErrorRateLimit& limiter) {
  if (limiter.suppressed_count > 0) {
    ESP_LOGW(TAG, "Suppressed %u similar errors in the last %u ms",
             limiter.suppressed_count, ErrorRateLimit::RATE_LIMIT_MS);
    limiter.suppressed_count = 0;
  }
}

void UpsHidComponent::cleanup() {
  // Stop the background task before tearing down transport
  if (usb_task_running_.load()) {
    usb_task_running_.store(false);
    if (usb_read_task_handle_) {
      // Give the task time to exit its loop
      vTaskDelay(pdMS_TO_TICKS(200));
      usb_read_task_handle_ = nullptr;
    }
  }

  if (transport_) {
    transport_->deinitialize();
    transport_.reset();
  }

  active_protocol_.reset();
  report_map_.reset();
  connected_ = false;

  ESP_LOGD(TAG, "Component cleanup completed");
}

// Timer polling implementation
void UpsHidComponent::check_and_update_timers() {
  if (!active_protocol_) return;

  uint32_t now = millis();

  // Check if we need to poll timers
  bool should_poll_timers = false;

  if (fast_polling_mode_) {
    // In fast polling mode, check every 2 seconds
    if (now - last_timer_poll_ >= FAST_POLL_INTERVAL_MS) {
      should_poll_timers = true;
    }
  } else {
    // Check if any timers might be active (less frequent check)
    if (now - last_timer_poll_ >= get_update_interval()) {
      should_poll_timers = true;
    }
  }

  if (should_poll_timers) {
    last_timer_poll_ = now;

    // Try to read timer data
    UpsData timer_data = ups_data_;  // Copy current data
    if (active_protocol_->read_timer_data(timer_data)) {
      // Update only timer-related fields
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ups_data_.test.timer_shutdown = timer_data.test.timer_shutdown;
        ups_data_.test.timer_start = timer_data.test.timer_start;
        ups_data_.test.timer_reboot = timer_data.test.timer_reboot;
      }

      // Update fast polling mode based on timer activity
      bool timers_active = has_active_timers();
      if (timers_active != fast_polling_mode_) {
        set_fast_polling_mode(timers_active);
      }

      // Update timer sensors immediately when values change
      update_sensors();
    }
  }
}

bool UpsHidComponent::has_active_timers() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return (ups_data_.test.timer_shutdown > 0 ||
          ups_data_.test.timer_start > 0 ||
          ups_data_.test.timer_reboot > 0);
}

void UpsHidComponent::set_fast_polling_mode(bool enable) {
  if (enable != fast_polling_mode_) {
    fast_polling_mode_ = enable;
    if (enable) {
      ESP_LOGI(TAG, "Enabled fast polling for timer countdown");
    } else {
      ESP_LOGI(TAG, "Disabled fast polling, returning to normal interval");
    }
  }
}

// Format a timestamp for event log entries.
// Uses wall-clock time if a time source is configured and synced, otherwise uptime.
std::string UpsHidComponent::format_event_timestamp() const {
  if (time_ != nullptr) {
    auto now = time_->now();
    if (now.is_valid()) {
      char buf[24];
      snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
               now.year, now.month, now.day_of_month,
               now.hour, now.minute, now.second);
      return std::string(buf);
    }
    ESP_LOGD(TAG, "Time source configured but not yet valid (year=%d)", now.year);
  } else {
    ESP_LOGD(TAG, "No time source configured, using uptime");
  }
  // Fallback to uptime
  uint32_t ms = millis();
  uint32_t total_s = ms / 1000;
  uint32_t h = total_s / 3600;
  uint32_t m = (total_s % 3600) / 60;
  uint32_t s = total_s % 60;
  char buf[16];
  snprintf(buf, sizeof(buf), "+%02u:%02u:%02u", h, m, s);
  return std::string(buf);
}

// State change detection -- records events to the ring buffer
void UpsHidComponent::check_state_changes() {
  StateSnapshot current;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto& s = ups_data_.power.status;
    if (s == status::ON_BATTERY) {
      current.online = false;
      current.on_battery = true;
    } else if (!s.empty() && s != status::UNKNOWN) {
      current.online = true;
      current.on_battery = false;
    } else {
      current.online = ups_data_.power.input_voltage_valid();
      current.on_battery = !current.online;
    }
    current.low_battery = ups_data_.battery.is_low();
    current.charging = current.online &&
                       ups_data_.battery.is_valid() &&
                       !std::isnan(ups_data_.battery.level) &&
                       ups_data_.battery.level < 100.0f;
    current.overloaded = ups_data_.power.is_overloaded();
    current.fault = ups_data_.power.is_input_out_of_range() ||
                    (!ups_data_.power.is_valid() && !ups_data_.battery.is_valid());

    float level = ups_data_.battery.is_valid() ? ups_data_.battery.level : NAN;
    current.battery_level_bucket = std::isnan(level) ? -1 : static_cast<int>(level) / 5;
    current.valid = true;
  }

  std::string ts = format_event_timestamp();

  if (!last_snapshot_.valid) {
    // First reading -- record initial state
    char buf[128];
    snprintf(buf, sizeof(buf), "Initial state: %s, battery %d%%",
             current.status_string().c_str(),
             current.battery_level_bucket >= 0 ? current.battery_level_bucket * 5 : -1);
    event_log_.record(ts, buf);
    ESP_LOGI(TAG, "Event log: %s", buf);
    last_snapshot_ = current;
    return;
  }

  // Check for status flag changes
  if (current.online != last_snapshot_.online ||
      current.on_battery != last_snapshot_.on_battery ||
      current.low_battery != last_snapshot_.low_battery ||
      current.charging != last_snapshot_.charging ||
      current.overloaded != last_snapshot_.overloaded ||
      current.fault != last_snapshot_.fault) {
    char buf[128];
    snprintf(buf, sizeof(buf), "Status: %s -> %s",
             last_snapshot_.status_string().c_str(),
             current.status_string().c_str());
    event_log_.record(ts, buf);
    ESP_LOGW(TAG, "Event log: %s", buf);
  }

  // Check for battery level bucket changes (5% granularity)
  if (current.battery_level_bucket != last_snapshot_.battery_level_bucket &&
      current.battery_level_bucket >= 0 && last_snapshot_.battery_level_bucket >= 0) {
    char buf[128];
    snprintf(buf, sizeof(buf), "Battery: %d%% -> %d%%",
             last_snapshot_.battery_level_bucket * 5,
             current.battery_level_bucket * 5);
    event_log_.record(ts, buf);
    ESP_LOGI(TAG, "Event log: %s", buf);
  }

  last_snapshot_ = current;
}

// Convenient state getters for lambda expressions (no sensor entities required).
// These use the protocol-reported status string as primary indicator, falling
// back to voltage checks only when status is unknown or not yet determined.
bool UpsHidComponent::is_online() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  const auto& s = ups_data_.power.status;
  if (s == status::ON_BATTERY) return false;
  if (!s.empty() && s != status::UNKNOWN) return true;
  return ups_data_.power.input_voltage_valid();
}

bool UpsHidComponent::is_on_battery() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ups_data_.power.status == status::ON_BATTERY;
}

bool UpsHidComponent::is_low_battery() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ups_data_.battery.is_low();
}

bool UpsHidComponent::is_charging() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (ups_data_.power.status == status::ON_BATTERY) return false;
  return ups_data_.battery.is_valid() &&
         !std::isnan(ups_data_.battery.level) &&
         ups_data_.battery.level < 100.0f;
}

bool UpsHidComponent::has_fault() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ups_data_.power.is_input_out_of_range() ||
         (!ups_data_.power.is_valid() && !ups_data_.battery.is_valid());
}

uint32_t UpsHidComponent::get_data_age_ms() const {
  return last_successful_read_ > 0 ? millis() - last_successful_read_ : 0;
}

bool UpsHidComponent::is_overloaded() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // Use power's built-in overload detection
  return ups_data_.power.is_overloaded();
}

float UpsHidComponent::get_battery_level() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ups_data_.battery.is_valid() ? ups_data_.battery.level : NAN;
}

float UpsHidComponent::get_input_voltage() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ups_data_.power.input_voltage;
}

float UpsHidComponent::get_output_voltage() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ups_data_.power.output_voltage;
}

float UpsHidComponent::get_load_percent() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ups_data_.power.load_percent;
}

float UpsHidComponent::get_runtime_minutes() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return ups_data_.battery.runtime_minutes;
}


} // namespace ups_hid
} // namespace esphome