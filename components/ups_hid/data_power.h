#pragma once

#include <string>
#include <cmath>
#include "constants_hid.h"

namespace esphome {
namespace ups_hid {

struct PowerData {
  // Input power metrics
  float input_voltage{NAN};            // Current input voltage (V)
  float input_voltage_nominal{NAN};    // Nominal input voltage (V)
  float input_transfer_low{NAN};       // Low transfer voltage threshold (V)
  float input_transfer_high{NAN};      // High transfer voltage threshold (V)
  float frequency{NAN};                // Input frequency (Hz)

  // Input frequency nominal
  float input_frequency_nominal{NAN};  // Nominal input frequency (Hz)

  // Output power metrics
  float output_voltage{NAN};           // Current output voltage (V)
  float output_voltage_nominal{NAN};   // Nominal output voltage (V)
  float output_current{NAN};           // Current output current (A)
  float output_frequency{NAN};         // Current output frequency (Hz)
  float active_power{NAN};             // Live active power draw (W)
  float load_percent{NAN};             // Current load percentage (0-100%)

  // Power ratings and capabilities
  float realpower_nominal{NAN};        // Nominal real power rating (W)
  float apparent_power_nominal{NAN};   // Nominal apparent power rating (VA)

  // Power status information
  std::string status{};                // Power status text (Online, On Battery, etc.)

  // AVR / power conditioning flags
  bool boost_active{false};            // AVR boost mode active
  bool buck_active{false};             // AVR buck mode active
  bool over_temperature{false};        // Over temperature alarm
  bool communication_lost{false};      // Communication lost flag
  bool shutdown_imminent{false};       // UPS will shutdown very soon
  bool awaiting_power{false};          // UPS waiting for AC power to return
  bool voltage_out_of_range{false};    // Input voltage out of range

  // How many consecutive read cycles the status was not freshly determined.
  // After MAX_STALE_CYCLES, status is forced to empty so downstream doesn't
  // keep acting on a state that may no longer be true.
  uint8_t status_stale_cycles{0};
  static constexpr uint8_t MAX_STALE_CYCLES = 3;

  // Power quality indicators
  bool input_voltage_valid() const {
    return !std::isnan(input_voltage) && input_voltage > 50.0f && input_voltage < 300.0f;
  }

  bool output_voltage_valid() const {
    return !std::isnan(output_voltage) && output_voltage > 50.0f && output_voltage < 300.0f;
  }

  bool frequency_valid() const {
    return !std::isnan(frequency) && frequency >= FREQUENCY_MIN_VALID && frequency <= FREQUENCY_MAX_VALID;
  }

  bool is_input_out_of_range() const {
    if (!input_voltage_valid()) return false;
    return (!std::isnan(input_transfer_low) && input_voltage < input_transfer_low) ||
           (!std::isnan(input_transfer_high) && input_voltage > input_transfer_high);
  }

  bool is_overloaded() const {
    return !std::isnan(load_percent) && load_percent > 95.0f;
  }

  bool has_load_info() const {
    return !std::isnan(load_percent);
  }

  // Validation and utility methods
  bool is_valid() const {
    return input_voltage_valid() || output_voltage_valid() || has_load_info();
  }

  // Merge valid fields from a fresh read, keeping old values where new data is NAN/empty.
  // Prevents transient USB read failures from clobbering good data with NAN.
  void merge_from(const PowerData& other) {
    if (!std::isnan(other.input_voltage)) input_voltage = other.input_voltage;
    if (!std::isnan(other.input_voltage_nominal)) input_voltage_nominal = other.input_voltage_nominal;
    if (!std::isnan(other.input_transfer_low)) input_transfer_low = other.input_transfer_low;
    if (!std::isnan(other.input_transfer_high)) input_transfer_high = other.input_transfer_high;
    if (!std::isnan(other.frequency)) frequency = other.frequency;
    if (!std::isnan(other.input_frequency_nominal)) input_frequency_nominal = other.input_frequency_nominal;
    if (!std::isnan(other.output_voltage)) output_voltage = other.output_voltage;
    if (!std::isnan(other.output_voltage_nominal)) output_voltage_nominal = other.output_voltage_nominal;
    if (!std::isnan(other.output_current)) output_current = other.output_current;
    if (!std::isnan(other.output_frequency)) output_frequency = other.output_frequency;
    if (!std::isnan(other.active_power)) active_power = other.active_power;
    if (!std::isnan(other.load_percent)) load_percent = other.load_percent;
    if (!std::isnan(other.realpower_nominal)) realpower_nominal = other.realpower_nominal;
    if (!std::isnan(other.apparent_power_nominal)) apparent_power_nominal = other.apparent_power_nominal;

    // Status and derived bools: only trust when protocol could determine state.
    // "Unknown" means the key HID reports failed to read; keep previous state
    // for a few cycles to ride out transient glitches.
    if (!other.status.empty() && other.status != "Unknown") {
      status = other.status;
      boost_active = other.boost_active;
      buck_active = other.buck_active;
      over_temperature = other.over_temperature;
      communication_lost = other.communication_lost;
      shutdown_imminent = other.shutdown_imminent;
      awaiting_power = other.awaiting_power;
      voltage_out_of_range = other.voltage_out_of_range;
      status_stale_cycles = 0;
    } else {
      status_stale_cycles++;
      if (status_stale_cycles >= MAX_STALE_CYCLES) {
        status.clear();
        boost_active = false;
        buck_active = false;
        over_temperature = false;
        communication_lost = false;
        shutdown_imminent = false;
        awaiting_power = false;
        voltage_out_of_range = false;
      }
    }
  }

  void reset() {
    *this = PowerData{};
  }

  // Copy constructor and assignment for safe copying
  PowerData() = default;
  PowerData(const PowerData&) = default;
  PowerData& operator=(const PowerData&) = default;
  PowerData(PowerData&&) = default;
  PowerData& operator=(PowerData&&) = default;
};

}  // namespace ups_hid
}  // namespace esphome