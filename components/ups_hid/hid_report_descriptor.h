#pragma once

/**
 * @file hid_report_descriptor.h
 * @brief HID Report Descriptor Parser for UPS Power Device Class
 *
 * Parses USB HID report descriptors to extract field definitions (voltage,
 * battery charge, runtime, etc.) for use with USB UPS devices.
 *
 * References:
 * - USB HID Device Class Definition v1.11
 * - USB HID Usage Tables
 * - USB Power Device Class v1.1
 */

#include "constants_hid.h"
#include <cstdint>
#include <vector>
#include <set>
#include <map>

namespace esphome {
namespace ups_hid {

/**
 * @brief Describes a single data field within a HID report
 */
struct HidField {
  uint8_t report_id;
  uint8_t report_type;  // Input=1, Output=2, Feature=3
  uint32_t usage;       // Full usage (page << 16 | usage_id)
  std::vector<uint32_t> usage_path;  // Collection nesting path
  uint16_t bit_offset;
  uint16_t bit_size;
  int32_t logical_min;
  int32_t logical_max;
  int32_t physical_min;
  int32_t physical_max;
  int8_t unit_exponent;
  uint32_t unit;
};

/**
 * @brief Parsed HID report descriptor providing field lookup and value extraction
 */
class HidReportMap {
 public:
  /** Parse a HID report descriptor. Returns true on success. */
  bool parse(const uint8_t* descriptor, size_t length);

  /** Find field by full usage (e.g., Voltage in Power Device page) */
  const HidField* find_field_by_usage(uint32_t usage) const;

  /** Find field by usage within a specific collection path */
  const HidField* find_field_by_usage_path(const std::vector<uint32_t>& path, uint32_t usage) const;

  /** Get all fields for a report ID and type */
  std::vector<const HidField*> get_fields_for_report(uint8_t report_id, uint8_t report_type) const;

  /** Get all known report IDs */
  std::set<uint8_t> get_report_ids() const;

  /** Get total report size in bytes for a report ID (for validation) */
  size_t get_report_size_bytes(uint8_t report_id, uint8_t report_type) const;

  /**
   * Extract a field's value from raw report data.
   * Applies logical-to-physical conversion and unit exponent.
   * Returns NAN on error or invalid data.
   */
  float extract_field_value(const HidField& field, const uint8_t* report_data, size_t report_len) const;

  /** Get all fields (for debugging/logging) */
  const std::vector<HidField>& get_all_fields() const { return fields_; }

  /** Log the parsed descriptor for debugging */
  void dump(const char* tag) const;

 private:
  std::vector<HidField> fields_;
};

}  // namespace ups_hid
}  // namespace esphome
