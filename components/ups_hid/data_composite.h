#pragma once

#include "data_battery.h"
#include "data_power.h"
#include "data_device.h"
#include "data_test.h"
#include "data_config.h"
#include <cstdint>
#include <cmath>
#include <string>

namespace esphome {
namespace ups_hid {

/**
 * Clean UPS Composite Data Structure
 * 
 * Contains all UPS data organized into logical components following
 * Single Responsibility Principle. No legacy compatibility code.
 */
struct UpsCompositeData {
  // Specialized data components
  BatteryData battery;
  PowerData power;
  DeviceInfo device;
  TestStatus test;
  ConfigData config;
  
  // Validation and utility methods
  bool is_valid() const {
    return battery.is_valid() || power.is_valid() || device.is_valid() || 
           test.is_valid() || config.is_valid();
  }
  
  bool has_core_data() const {
    return battery.is_valid() && power.is_valid();
  }

  // Derived state, shared by the NUT server, binary sensors and event log so
  // every consumer agrees on what the UPS is doing. Token semantics follow
  // NUT's usbhid-ups ups_status_set().
  bool is_online() const { return power.status.compare(0, 6, "Online") == 0; }
  bool is_on_battery() const { return power.status.compare(0, 10, "On Battery") == 0; }
  bool has_power_status() const { return is_online() || is_on_battery(); }
  bool is_discharging() const { return battery.status == "Discharging"; }
  bool is_charging() const {
    if (battery.status == "Charging") return true;
    // Protocols that do not report charge flags: assume charging while online below 100%.
    return battery.status.empty() && is_online() && battery.is_valid() &&
           !std::isnan(battery.level) && battery.level < 100.0f;
  }
  bool is_low_battery() const { return battery.is_low() || power.shutdown_imminent; }
  bool has_fault() const {
    return power.internal_failure || power.over_temperature || power.is_input_out_of_range();
  }

  // NUT ups.alarm text; empty when no alarm is active.
  std::string nut_alarm() const {
    std::string a;
    auto add = [&](const char *msg) { if (!a.empty()) a += ' '; a += msg; };
    if (battery.needs_replacement) add("Replace battery!");
    if (power.shutdown_imminent) add("Shutdown imminent!");
    if (power.over_temperature) add("Temperature too high!");
    if (power.internal_failure) add("Internal UPS fault!");
    if (power.awaiting_power) add("Awaiting power!");
    return a;
  }

  // NUT ups.status tokens, e.g. "OL CHRG" or "OB DISCHRG LB". Empty when the
  // power state is unknown (caller should report stale data).
  std::string nut_status() const {
    if (!has_power_status()) return "";
    std::string s;
    auto add = [&](const char *token) { if (!s.empty()) s += ' '; s += token; };
    if (!nut_alarm().empty()) add("ALARM");
    add(is_online() ? "OL" : "OB");
    if (is_discharging()) add("DISCHRG");
    if (is_charging()) add("CHRG");
    if (is_low_battery()) add("LB");
    if (battery.needs_replacement) add("RB");
    if (power.is_overloaded()) add("OVER");
    if (power.buck_active) add("TRIM");
    if (power.boost_active) add("BOOST");
    return s;
  }
  
  // Merge valid fields from a fresh read into the persistent data.
  // Only overwrites fields where the new read produced a valid value.
  void merge_from(const UpsCompositeData& other) {
    battery.merge_from(other.battery);
    power.merge_from(other.power);
    if (other.device.is_valid()) device = other.device;
    if (other.test.is_valid()) test = other.test;
    if (other.config.is_valid()) config = other.config;
  }

  // Clean reset without legacy flags
  void reset() {
    battery.reset();
    power.reset();
    device.reset();
    test.reset();
    config.reset();
  }
  
  // Copy constructor and assignment for safe copying
  UpsCompositeData() = default;
  UpsCompositeData(const UpsCompositeData&) = default;
  UpsCompositeData& operator=(const UpsCompositeData&) = default;
  UpsCompositeData(UpsCompositeData&&) = default;
  UpsCompositeData& operator=(UpsCompositeData&&) = default;
};

// Type alias for cleaner naming
using UpsData = UpsCompositeData;

}  // namespace ups_hid
}  // namespace esphome