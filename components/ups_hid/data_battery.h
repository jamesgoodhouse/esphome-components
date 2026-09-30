#pragma once

#include <string>
#include <cmath>

namespace esphome {
namespace ups_hid {

struct BatteryData {
  // Core battery metrics
  float level{NAN};                    // Battery charge level (0-100%)
  float voltage{NAN};                  // Current battery voltage (V)
  float voltage_nominal{NAN};          // Nominal battery voltage (V)
  float runtime_minutes{NAN};          // Estimated runtime (minutes)

  // Battery capacity
  float config_voltage{NAN};           // Nominal / config battery voltage (V)
  float full_charge_capacity{NAN};     // Full charge capacity (Ah or %)
  float design_capacity{NAN};          // Design capacity (Ah or %)

  // Battery thresholds
  float charge_low{NAN};               // Low battery threshold (%)
  float charge_warning{NAN};           // Warning battery threshold (%)
  float runtime_low{NAN};              // Low runtime threshold (minutes)

  // Battery status information
  std::string status{};                // Battery status text
  std::string type{};                  // Battery chemistry type
  std::string mfr_date{};              // Battery manufacture date
  bool needs_replacement{false};       // Battery replacement needed flag
  bool low_battery{false};             // Device-reported low battery (BelowRemainingCapacityLimit / time limit expired)
  bool flags_valid{false};             // needs_replacement/low_battery were read this cycle

  // Validation and utility methods
  bool is_valid() const {
    return !std::isnan(level) || !std::isnan(voltage) || !std::isnan(runtime_minutes);
  }

  // Low battery: the device's own judgement, or our thresholds when known.
  bool is_low() const {
    if (low_battery) return true;
    if (!std::isnan(level) && !std::isnan(charge_low) && level <= charge_low) return true;
    return !std::isnan(runtime_minutes) && !std::isnan(runtime_low) && runtime_minutes <= runtime_low;
  }

  bool is_warning() const {
    return !std::isnan(level) && !std::isnan(charge_warning) && level <= charge_warning;
  }

  bool has_runtime_estimate() const {
    return !std::isnan(runtime_minutes) && runtime_minutes > 0;
  }

  // Merge valid fields from a fresh read, keeping old values where new data is NAN/empty.
  void merge_from(const BatteryData& other) {
    if (!std::isnan(other.level)) level = other.level;
    if (!std::isnan(other.voltage)) voltage = other.voltage;
    if (!std::isnan(other.voltage_nominal)) voltage_nominal = other.voltage_nominal;
    if (!std::isnan(other.runtime_minutes)) runtime_minutes = other.runtime_minutes;
    if (!std::isnan(other.config_voltage)) config_voltage = other.config_voltage;
    if (!std::isnan(other.full_charge_capacity)) full_charge_capacity = other.full_charge_capacity;
    if (!std::isnan(other.design_capacity)) design_capacity = other.design_capacity;
    if (!std::isnan(other.charge_low)) charge_low = other.charge_low;
    if (!std::isnan(other.charge_warning)) charge_warning = other.charge_warning;
    if (!std::isnan(other.runtime_low)) runtime_low = other.runtime_low;
    if (!other.status.empty()) status = other.status;
    if (!other.type.empty()) type = other.type;
    if (!other.mfr_date.empty()) mfr_date = other.mfr_date;
    if (other.flags_valid) {
      low_battery = other.low_battery;
      needs_replacement = other.needs_replacement;
      flags_valid = true;
    } else if (other.is_valid()) {
      needs_replacement = other.needs_replacement;
    }
  }

  void reset() {
    *this = BatteryData{};
  }

  // Copy constructor and assignment for safe copying
  BatteryData() = default;
  BatteryData(const BatteryData&) = default;
  BatteryData& operator=(const BatteryData&) = default;
  BatteryData(BatteryData&&) = default;
  BatteryData& operator=(BatteryData&&) = default;
};

}  // namespace ups_hid
}  // namespace esphome