#include "protocol_tripplite.h"
#include "ups_hid.h"
#include "constants_hid.h"
#include "constants_ups.h"
#include "esphome/core/log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include <set>
#include <map>
#include <algorithm>
#include <cmath>

namespace esphome {
namespace ups_hid {

static const char *const TL_TAG = "ups_hid.tripplite";

// ============================================================================
// Common HID report IDs observed on Tripp Lite devices
// These are discovered at runtime but we try these first for faster detection.
// Based on analysis of NUT debug logs and HID report descriptors from
// ECO850LCD, OMNI1000LCD, SMART1000LCD, and similar models.
// ============================================================================

// Report IDs to try during detection (most commonly found on Tripp Lite devices)
static const uint8_t TL_DETECTION_REPORT_IDS[] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
};

// Extended report IDs for full enumeration
static const uint8_t TL_EXTENDED_REPORT_IDS[] = {
    0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
    0x50, 0x51, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x5A,
};


// ============================================================================
// Protocol Detection
// ============================================================================

bool TrippLiteProtocol::detect() {
    ESP_LOGD(TL_TAG, "Detecting Tripp Lite HID protocol...");

    if (!parent_->is_connected()) {
        ESP_LOGD(TL_TAG, "Device not connected, skipping protocol detection");
        return false;
    }

    // Verify this is a Tripp Lite device
    uint16_t vid = parent_->get_vendor_id();
    if (vid != usb::VENDOR_ID_TRIPPLITE) {
        ESP_LOGD(TL_TAG, "Not a Tripp Lite device (VID=0x%04X)", vid);
        return false;
    }

    // Reject non-HID Tripp Lite devices (PID 0x0001 uses serial-over-USB protocol)
    uint16_t pid = parent_->get_product_id();
    if (pid == 0x0001) {
        ESP_LOGW(TL_TAG, "Tripp Lite device PID 0x0001 uses serial protocol, not HID. "
                 "This device is not supported by the HID driver.");
        return false;
    }

    ESP_LOGI(TL_TAG, "Tripp Lite device detected (VID=0x%04X, PID=0x%04X)", vid, pid);

    // Give device time to initialize after connection
    vTaskDelay(pdMS_TO_TICKS(timing::USB_INITIALIZATION_DELAY_MS));

    // Try to read any HID report to confirm HID communication works
    HidReport test_report;
    for (uint8_t report_id : TL_DETECTION_REPORT_IDS) {
        if (!parent_->is_connected()) {
            ESP_LOGD(TL_TAG, "Device disconnected during protocol detection");
            return false;
        }

        if (read_hid_report(report_id, test_report)) {
            ESP_LOGI(TL_TAG, "Tripp Lite HID protocol confirmed via report 0x%02X (%zu bytes)",
                     report_id, test_report.data.size());
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(timing::REPORT_RETRY_DELAY_MS));
    }

    ESP_LOGW(TL_TAG, "Failed to read any HID reports from Tripp Lite device");
    return false;
}


// ============================================================================
// Protocol Initialization
// ============================================================================

bool TrippLiteProtocol::initialize() {
    ESP_LOGI(TL_TAG, "Initializing Tripp Lite HID protocol...");

    // Clear previous state
    available_input_reports_.clear();
    available_feature_reports_.clear();
    report_sizes_.clear();
    device_info_read_ = false;
    use_descriptor_ = false;
    descriptor_needs_raw_extraction_ = false;
    descriptor_voltage_scale_ = 1.0;
    descriptor_frequency_scale_ = 1.0;
    queried_usages_.clear();
    // Always determine scaling factors (needed for both modes)
    determine_scaling_factors();

    // Check if a parsed HID report descriptor is available
    const HidReportMap* map = parent_->get_report_map();
    if (map && !map->get_all_fields().empty()) {
        use_descriptor_ = true;
        ESP_LOGI(TL_TAG, "HID report descriptor available (%zu fields) - using descriptor-based reading",
                 map->get_all_fields().size());
        enumerate_reports_from_descriptor();
    } else {
        ESP_LOGW(TL_TAG, "No HID report descriptor - falling back to heuristic discovery");
        enumerate_reports();
    }

    if (available_input_reports_.empty() && available_feature_reports_.empty()) {
        ESP_LOGE(TL_TAG, "No HID reports found during initialization");
        return false;
    }

    ESP_LOGI(TL_TAG, "Tripp Lite HID initialized (%s mode): %zu input reports, %zu feature reports",
             use_descriptor_ ? "descriptor" : "heuristic",
             available_input_reports_.size(), available_feature_reports_.size());

    for (uint8_t id : available_feature_reports_) {
        ESP_LOGD(TL_TAG, "  Feature report 0x%02X: %zu bytes", id, report_sizes_[id]);
    }
    for (uint8_t id : available_input_reports_) {
        ESP_LOGD(TL_TAG, "  Input report 0x%02X: %zu bytes", id, report_sizes_[id]);
    }

    return true;
}

void TrippLiteProtocol::enumerate_reports_from_descriptor() {
    const HidReportMap* map = parent_->get_report_map();
    if (!map) return;

    auto ids = map->get_report_ids();
    ESP_LOGD(TL_TAG, "Descriptor contains %zu report IDs", ids.size());

    for (uint8_t id : ids) {
        size_t feat_sz = map->get_report_size_bytes(id, HID_REPORT_TYPE_FEATURE);
        if (feat_sz > 0) {
            available_feature_reports_.insert(id);
            report_sizes_[id] = feat_sz;
        }
        size_t inp_sz = map->get_report_size_bytes(id, HID_REPORT_TYPE_INPUT);
        if (inp_sz > 0) {
            available_input_reports_.insert(id);
            if (report_sizes_.find(id) == report_sizes_.end()) {
                report_sizes_[id] = inp_sz;
            }
        }
    }
}


// ============================================================================
// Scaling Factor Determination (based on NUT tripplite-hid.c)
// ============================================================================

void TrippLiteProtocol::determine_scaling_factors() {
    uint16_t pid = parent_->get_product_id();

    // Default scales
    battery_scale_ = 1.0;
    io_voltage_scale_ = 1.0;
    io_frequency_scale_ = 1.0;
    io_current_scale_ = 1.0;

    // Product-specific scaling based on NUT tripplite-hid.c device table
    // PID 0x1xxx series (AVR/ECO models) - battery voltage needs 0.1 scaling
    if (pid == 0x1003 || pid == 0x1007 || pid == 0x1008 ||
        pid == 0x1009 || pid == 0x1010) {
        battery_scale_ = 0.1;
    }
    // PID 0x2xxx series (ECO/OMNI/SMART LCD models) - battery voltage needs 0.1 scaling
    else if (pid >= 0x2000 && pid <= 0x2FFF) {
        battery_scale_ = 0.1;
    }
    // PID 0x3016 (SMART1500LCDT newer) and 0x3024 (AVR750U newer / ECO850LCD)
    // These devices have HID descriptors with incorrect unit exponents.
    // Heuristic scaling stays at 1.0 (auto-detection handles ranges).
    // Descriptor mode uses raw extraction with device-specific scaling.
    else if (pid == 0x3016 || pid == 0x3024) {
        battery_scale_ = 1.0;
        io_voltage_scale_ = 1.0;
        io_frequency_scale_ = 1.0;
        io_current_scale_ = 1.0;
        // Descriptor-specific: raw logical values are in decivolts/decihertz
        descriptor_needs_raw_extraction_ = true;
        descriptor_voltage_scale_ = 0.1;    // 1207 → 120.7V
        descriptor_frequency_scale_ = 0.1;  // 602 → 60.2Hz
    }
    // PID 0x3xxx series (SMART models, newer) - no battery scaling
    else if (pid >= 0x3000 && pid <= 0x3FFF) {
        battery_scale_ = 1.0;
    }
    // PID 0x4xxx series (SmartOnline models) - no battery scaling
    else if (pid >= 0x4000 && pid <= 0x4FFF) {
        battery_scale_ = 1.0;
    }

    ESP_LOGI(TL_TAG, "Scaling factors for PID 0x%04X: battery=%.4f, voltage=%.4f, freq=%.4f",
             pid, battery_scale_, io_voltage_scale_, io_frequency_scale_);
}


// ============================================================================
// Report Enumeration
// ============================================================================

void TrippLiteProtocol::enumerate_reports() {
    ESP_LOGD(TL_TAG, "Enumerating Tripp Lite HID reports...");

    uint8_t buffer[limits::MAX_HID_REPORT_SIZE];
    size_t buffer_len;
    int discovered_count = 0;

    // Try primary detection report IDs first
    for (uint8_t id : TL_DETECTION_REPORT_IDS) {
        if (!parent_->is_connected()) {
            ESP_LOGD(TL_TAG, "Device disconnected during enumeration");
            return;
        }

        // Try Feature report first (most Tripp Lite data is in Feature reports)
        buffer_len = sizeof(buffer);
        esp_err_t ret = parent_->hid_get_report(HID_REPORT_TYPE_FEATURE, id,
                                                 buffer, &buffer_len,
                                                 parent_->get_protocol_timeout());
        if (ret == ESP_OK && buffer_len > 0) {
            available_feature_reports_.insert(id);
            report_sizes_[id] = buffer_len;
            discovered_count++;
            ESP_LOGV(TL_TAG, "Found Feature report 0x%02X (%zu bytes)", id, buffer_len);
        }

        if (!parent_->is_connected()) return;

        // Also try Input report
        buffer_len = sizeof(buffer);
        ret = parent_->hid_get_report(HID_REPORT_TYPE_INPUT, id,
                                       buffer, &buffer_len,
                                       parent_->get_protocol_timeout());
        if (ret == ESP_OK && buffer_len > 0) {
            available_input_reports_.insert(id);
            if (report_sizes_.find(id) == report_sizes_.end()) {
                report_sizes_[id] = buffer_len;
            }
            discovered_count++;
            ESP_LOGV(TL_TAG, "Found Input report 0x%02X (%zu bytes)", id, buffer_len);
        }

        vTaskDelay(pdMS_TO_TICKS(timing::REPORT_DISCOVERY_DELAY_MS));
    }

    // Extended search for additional reports
    ESP_LOGD(TL_TAG, "Found %d reports in primary scan, performing extended search...", discovered_count);

    for (uint8_t id : TL_EXTENDED_REPORT_IDS) {
        if (!parent_->is_connected()) return;
        if (discovered_count >= static_cast<int>(limits::MAX_EXTENDED_DISCOVERY_ATTEMPTS)) break;

        // Try Feature report
        buffer_len = sizeof(buffer);
        esp_err_t ret = parent_->hid_get_report(HID_REPORT_TYPE_FEATURE, id,
                                                 buffer, &buffer_len,
                                                 parent_->get_protocol_timeout());
        if (ret == ESP_OK && buffer_len > 0) {
            available_feature_reports_.insert(id);
            if (report_sizes_.find(id) == report_sizes_.end()) {
                report_sizes_[id] = buffer_len;
            }
            discovered_count++;
            ESP_LOGV(TL_TAG, "Found Feature report 0x%02X (%zu bytes) [extended]", id, buffer_len);
        }

        if (!parent_->is_connected()) return;

        // Try Input report
        buffer_len = sizeof(buffer);
        ret = parent_->hid_get_report(HID_REPORT_TYPE_INPUT, id,
                                       buffer, &buffer_len,
                                       parent_->get_protocol_timeout());
        if (ret == ESP_OK && buffer_len > 0) {
            available_input_reports_.insert(id);
            if (report_sizes_.find(id) == report_sizes_.end()) {
                report_sizes_[id] = buffer_len;
            }
            discovered_count++;
            ESP_LOGV(TL_TAG, "Found Input report 0x%02X (%zu bytes) [extended]", id, buffer_len);
        }

        vTaskDelay(pdMS_TO_TICKS(timing::REPORT_DISCOVERY_DELAY_MS));
    }

    ESP_LOGD(TL_TAG, "Report enumeration complete: %d total reports discovered", discovered_count);
}


// ============================================================================
// HID Report I/O
// ============================================================================

bool TrippLiteProtocol::read_hid_report(uint8_t report_id, HidReport &report) {
    if (!parent_->is_connected()) {
        return false;
    }

    uint8_t buffer[limits::MAX_HID_REPORT_SIZE];
    size_t buffer_len;
    esp_err_t ret;

    // Try Feature report first (Tripp Lite primarily uses Feature reports)
    if (available_feature_reports_.empty() || available_feature_reports_.count(report_id)) {
        buffer_len = sizeof(buffer);
        ret = parent_->hid_get_report(HID_REPORT_TYPE_FEATURE, report_id,
                                       buffer, &buffer_len,
                                       parent_->get_protocol_timeout());
        if (ret == ESP_OK && buffer_len > 0) {
            report.report_id = report_id;
            report.data.assign(buffer, buffer + buffer_len);
            ESP_LOGV(TL_TAG, "Read Feature report 0x%02X: %zu bytes", report_id, buffer_len);
            return true;
        }
    }

    // Fallback to Input report
    if (available_input_reports_.empty() || available_input_reports_.count(report_id)) {
        buffer_len = sizeof(buffer);
        ret = parent_->hid_get_report(HID_REPORT_TYPE_INPUT, report_id,
                                       buffer, &buffer_len,
                                       parent_->get_protocol_timeout());
        if (ret == ESP_OK && buffer_len > 0) {
            report.report_id = report_id;
            report.data.assign(buffer, buffer + buffer_len);
            ESP_LOGV(TL_TAG, "Read Input report 0x%02X: %zu bytes", report_id, buffer_len);
            return true;
        }
    }

    return false;
}

bool TrippLiteProtocol::write_hid_feature_report(uint8_t report_id, const uint8_t* data, size_t len) {
    if (!parent_->is_connected()) {
        return false;
    }

    esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE, report_id,
                                             data, len, parent_->get_protocol_timeout());
    if (ret == ESP_OK) {
        ESP_LOGD(TL_TAG, "Wrote Feature report 0x%02X: %zu bytes", report_id, len);
        return true;
    }

    ESP_LOGD(TL_TAG, "Failed to write Feature report 0x%02X: %s", report_id, esp_err_to_name(ret));
    return false;
}


// ============================================================================
// Value Extraction Helpers
// ============================================================================

float TrippLiteProtocol::read_single_byte_value(const HidReport &report, uint8_t byte_index) {
    if (report.data.size() > byte_index) {
        return static_cast<float>(report.data[byte_index]);
    }
    return NAN;
}

uint16_t TrippLiteProtocol::read_16bit_le_value(const HidReport &report, uint8_t start_index) {
    if (report.data.size() > static_cast<size_t>(start_index + 1)) {
        return report.data[start_index] | (report.data[start_index + 1] << 8);
    }
    return 0xFFFF;
}

float TrippLiteProtocol::apply_battery_voltage_scale(float raw_value) {
    return static_cast<float>(battery_scale_ * raw_value);
}

float TrippLiteProtocol::apply_io_voltage_scale(float raw_value) {
    return static_cast<float>(io_voltage_scale_ * raw_value);
}

float TrippLiteProtocol::apply_io_frequency_scale(float raw_value) {
    return static_cast<float>(io_frequency_scale_ * raw_value);
}


// ============================================================================
// Main Data Reading - dispatch between descriptor and heuristic modes
// ============================================================================

bool TrippLiteProtocol::read_data(UpsData &data) {
    // Read device information (once, on first successful read)
    if (!device_info_read_) {
        read_device_information(data);
    }

    if (use_descriptor_) {
        return read_data_descriptor(data);
    }
    return read_data_heuristic(data);
}


// ============================================================================
// Descriptor-based Data Reading (preferred)
// ============================================================================
//
// Uses the parsed HID report descriptor to know exactly which report ID
// contains which data field. The descriptor provides:
// - Report ID for each HID usage (Voltage, Frequency, RemainingCapacity, etc.)
// - Bit offset and size within the report
// - Logical/physical conversion factors
// - Unit exponents (e.g., 10^-1 for decivolts)
//
// This eliminates guesswork and handles all unit conversions correctly.
// ============================================================================

float TrippLiteProtocol::read_usage_value(
    const HidReportMap* map,
    const std::map<uint8_t, std::vector<uint8_t>>& cache,
    uint32_t usage, const char* name) {

    queried_usages_.insert(usage);
    const HidField* field = map->find_field_by_usage(usage);
    if (!field) {
        ESP_LOGV(TL_TAG, "No descriptor field for %s (usage 0x%08lX)", name, (unsigned long)usage);
        return NAN;
    }

    auto it = cache.find(field->report_id);
    if (it == cache.end()) {
        ESP_LOGV(TL_TAG, "No data for report 0x%02X (%s)", field->report_id, name);
        return NAN;
    }

    float val;
    if (descriptor_needs_raw_extraction_) {
        // Device has incorrect descriptor exponents; use raw logical value
        val = map->extract_raw_value(*field, it->second.data(), it->second.size());
    } else {
        // Device has correct descriptor; use full conversion (physical + exponent)
        val = map->extract_field_value(*field, it->second.data(), it->second.size());
    }
    if (!std::isnan(val)) {
        ESP_LOGD(TL_TAG, "%s = %.2f (report 0x%02X, bits %u@%u, %s)",
                 name, val, field->report_id, field->bit_size, field->bit_offset,
                 descriptor_needs_raw_extraction_ ? "raw" : "converted");
    }
    return val;
}

float TrippLiteProtocol::read_usage_in_collection(
    const HidReportMap* map,
    const std::map<uint8_t, std::vector<uint8_t>>& cache,
    uint32_t usage, uint32_t collection_usage,
    const char* name) {

    queried_usages_.insert(usage);
    // Search for a field with the given usage that is nested under
    // a collection with the given collection_usage in its path
    const HidField* field = nullptr;
    for (const auto& f : map->get_all_fields()) {
        if (f.usage != usage) continue;
        for (auto path_u : f.usage_path) {
            if (path_u == collection_usage) {
                field = &f;
                break;
            }
        }
        if (field) break;
    }

    if (!field) {
        ESP_LOGV(TL_TAG, "No field for %s (usage 0x%08lX in collection 0x%08lX)",
                 name, (unsigned long)usage, (unsigned long)collection_usage);
        return NAN;
    }

    auto it = cache.find(field->report_id);
    if (it == cache.end()) {
        ESP_LOGV(TL_TAG, "No data for report 0x%02X (%s)", field->report_id, name);
        return NAN;
    }

    float val;
    if (descriptor_needs_raw_extraction_) {
        val = map->extract_raw_value(*field, it->second.data(), it->second.size());
    } else {
        val = map->extract_field_value(*field, it->second.data(), it->second.size());
    }
    if (!std::isnan(val)) {
        ESP_LOGD(TL_TAG, "%s = %.2f (report 0x%02X, collection 0x%08lX, %s)",
                 name, val, field->report_id, (unsigned long)collection_usage,
                 descriptor_needs_raw_extraction_ ? "raw" : "converted");
    }
    return val;
}

uint8_t TrippLiteProtocol::find_report_id_for_usage(uint32_t usage) const {
    const HidReportMap* map = parent_->get_report_map();
    if (!map) return 0;

    const HidField* field = map->find_field_by_usage(usage);
    return field ? field->report_id : 0;
}

bool TrippLiteProtocol::read_data_descriptor(UpsData &data) {
    const HidReportMap* map = parent_->get_report_map();
    if (!map) {
        ESP_LOGW(TL_TAG, "Report map no longer available, switching to heuristic mode");
        use_descriptor_ = false;
        return read_data_heuristic(data);
    }

    ESP_LOGV(TL_TAG, "Reading Tripp Lite HID data (descriptor mode)...");

    // Step 1: Read ALL available feature reports into a cache.
    // Each report is a separate USB HID GET_REPORT request.
    // Individual reports can fail while others succeed.
    std::map<uint8_t, std::vector<uint8_t>> report_cache;
    int reports_read = 0;
    int reports_failed = 0;

    for (uint8_t rid : available_feature_reports_) {
        HidReport report;
        if (read_hid_report(rid, report) && !report.data.empty()) {
            report_cache[rid] = std::move(report.data);
            reports_read++;
            report_fail_count_[rid] = 0;
        } else {
            reports_failed++;
            uint8_t &count = report_fail_count_[rid];
            if (count < 255) count++;
            if (count == REPORT_FAIL_THRESHOLD) {
                ESP_LOGW(TL_TAG, "Report 0x%02X has failed %u consecutive reads (%zu bytes expected)",
                         rid, REPORT_FAIL_THRESHOLD,
                         report_sizes_.count(rid) ? report_sizes_[rid] : 0);
                // Log what usages the descriptor says are in this report
                auto fields = map->get_fields_for_report(rid, HID_REPORT_TYPE_FEATURE);
                for (const auto *f : fields) {
                    uint16_t page = (f->usage >> 16) & 0xFFFF;
                    uint16_t id = f->usage & 0xFFFF;
                    ESP_LOGW(TL_TAG, "  -> usage 0x%04X:0x%04X (%s page), %u bits @ offset %u",
                             page, id,
                             page == 0x0084 ? "Power Device" :
                             page == 0x0085 ? "Battery System" : "other",
                             f->bit_size, f->bit_offset);
                }
            }
        }
    }

    if (reports_read == 0) {
        ESP_LOGW(TL_TAG, "All %zu report reads failed", available_feature_reports_.size());
        return false;
    }

    if (reports_failed > 0) {
        ESP_LOGD(TL_TAG, "Read %d/%zu reports (%d failed)",
                 reports_read, available_feature_reports_.size(), reports_failed);
    } else {
        ESP_LOGD(TL_TAG, "Read all %d reports", reports_read);
    }

    // Step 2: Extract values using the parsed descriptor
    // Full 32-bit usages: (page << 16) | usage_id

    // --- Battery data ---
    data.battery.level = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_REMAINING_CAPACITY), "battery.charge");

    float runtime_sec = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_RUN_TIME_TO_EMPTY), "battery.runtime");
    if (!std::isnan(runtime_sec) && runtime_sec > 0) {
        data.battery.runtime_minutes = runtime_sec / 60.0f;
    }

    // Battery voltage - look in BatterySystem.Battery collection first
    // Battery voltage uses a different range (12V/24V) than AC voltage.
    // First try in Battery collection, then scan all Voltage fields for one
    // in battery range that isn't already claimed as input/output.
    float bat_voltage = read_usage_in_collection(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_VOLTAGE),
        HID_USAGE_BAT(0x0012),  // Battery collection
        "battery.voltage");
    if (!std::isnan(bat_voltage)) {
        if (bat_voltage >= 1.0f && bat_voltage <= 60.0f) {
            data.battery.voltage = bat_voltage;
        } else if (descriptor_needs_raw_extraction_ &&
                   bat_voltage * descriptor_voltage_scale_ >= 1.0f &&
                   bat_voltage * descriptor_voltage_scale_ <= 60.0f) {
            data.battery.voltage = bat_voltage * descriptor_voltage_scale_;
        }
    }
    // Fallback: scan all Voltage fields for one in DC battery range
    if (std::isnan(data.battery.voltage)) {
        for (const auto& f : map->get_all_fields()) {
            if (f.usage != HID_USAGE_POW(HID_USAGE_POW_VOLTAGE)) continue;
            auto it = report_cache.find(f.report_id);
            if (it == report_cache.end()) continue;
            float v = descriptor_needs_raw_extraction_
                ? map->extract_raw_value(f, it->second.data(), it->second.size())
                : map->extract_field_value(f, it->second.data(), it->second.size());
            if (std::isnan(v)) continue;
            float scaled = descriptor_needs_raw_extraction_ ? v * descriptor_voltage_scale_ : v;
            if (scaled >= 1.0f && scaled <= 60.0f) {
                data.battery.voltage = scaled;
                ESP_LOGD(TL_TAG, "battery.voltage fallback = %.1f (report 0x%02X, raw=%g)",
                         scaled, f.report_id, v);
                break;
            }
        }
    }

    // Battery voltage nominal (ConfigVoltage in Battery collection)
    // Config values are already in correct units, no scaling needed
    float bat_voltage_nom = read_usage_in_collection(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_CONFIG_VOLTAGE),
        HID_USAGE_BAT(0x0012),  // Battery collection
        "battery.voltage.nominal");
    if (!std::isnan(bat_voltage_nom) && bat_voltage_nom >= 6.0f && bat_voltage_nom <= 60.0f) {
        data.battery.voltage_nominal = bat_voltage_nom;
    }

    // Battery config voltage: look for PowerDevice:ConfigVoltage (0x0040) scoped
    // to the BatterySystem collection, like NUT does with path
    // "UPS.BatterySystem.Battery.ConfigVoltage". This is NOT 0x85:0x008B
    // (which is actually Rechargeable, a boolean flag).
    float bat_config_voltage = read_usage_in_collection(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_CONFIG_VOLTAGE),
        HID_USAGE_POW(HID_USAGE_POW_BATTERY_SYSTEM),
        "battery.config_voltage");
    if (std::isnan(bat_config_voltage)) {
        // Also try Battery (0x0012) collection
        bat_config_voltage = read_usage_in_collection(map, report_cache,
            HID_USAGE_POW(HID_USAGE_POW_CONFIG_VOLTAGE),
            HID_USAGE_POW(HID_USAGE_POW_BATTERY),
            "battery.config_voltage.alt");
    }
    if (!std::isnan(bat_config_voltage) && bat_config_voltage >= 6.0f && bat_config_voltage <= 60.0f) {
        data.battery.config_voltage = bat_config_voltage;
        if (std::isnan(data.battery.voltage_nominal)) {
            data.battery.voltage_nominal = bat_config_voltage;
        }
    }

    // Battery full charge capacity
    float full_charge_cap = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_FULL_CHARGE_CAPACITY), "battery.full_charge_capacity");
    if (!std::isnan(full_charge_cap)) {
        data.battery.full_charge_capacity = full_charge_cap;
    }

    // Battery design capacity
    float design_cap = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_DESIGN_CAPACITY), "battery.design_capacity");
    if (!std::isnan(design_cap)) {
        data.battery.design_capacity = design_cap;
    }

    // Charging/Discharging/FullyCharged status flags
    // Standard location: BatterySystem page (0x85)
    float charging = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_CHARGING), "battery.charging");
    float discharging = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_DISCHARGING), "battery.discharging");
    float fully_charged = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_FULLY_CHARGED), "battery.fully_charged");

    // Tripp Lite page-confusion fallback: some TL devices put these on page 0x84
    if (std::isnan(charging)) {
        charging = read_usage_value(map, report_cache,
            HID_USAGE_POW(HID_USAGE_TL_CHARGING), "battery.charging.p84");
    }
    if (std::isnan(discharging)) {
        discharging = read_usage_value(map, report_cache,
            HID_USAGE_POW(HID_USAGE_TL_DISCHARGING), "battery.discharging.p84");
    }

    // AC Present flag (standard on 0x85, TL also puts it on 0x84)
    float ac_present = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_AC_PRESENT), "ups.status.ac_present");
    if (std::isnan(ac_present)) {
        ac_present = read_usage_value(map, report_cache,
            HID_USAGE_POW(HID_USAGE_TL_AC_PRESENT), "ups.status.ac_present.p84");
    }

    if (!std::isnan(discharging) && discharging > 0) {
        data.battery.status = battery_status::DISCHARGING;
        data.power.status = status::ON_BATTERY;
    } else if (!std::isnan(fully_charged) && fully_charged > 0) {
        data.battery.status = battery_status::FULLY_CHARGED;
    } else if (!std::isnan(charging) && charging > 0) {
        data.battery.status = battery_status::CHARGING;
    }

    // Fully discharged flag
    float fully_discharged = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_FULLY_DISCHARGED), "battery.fully_discharged");

    // Need replacement flag (standard on 0x85, TL also puts on 0x84)
    float need_replace = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_NEED_REPLACEMENT), "battery.need_replacement");
    if (std::isnan(need_replace)) {
        need_replace = read_usage_value(map, report_cache,
            HID_USAGE_POW(HID_USAGE_TL_NEED_REPLACEMENT), "battery.need_replacement.p84");
    }
    if (!std::isnan(need_replace) && need_replace > 0) {
        data.battery.needs_replacement = true;
    }

    // --- Input data ---
    // Input voltage (Voltage in Input collection)
    data.power.input_voltage = read_usage_in_collection(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_VOLTAGE),
        HID_USAGE_POW(HID_USAGE_POW_INPUT),
        "input.voltage");

    // If not found in Input collection, try PowerSummary or first Voltage field
    if (std::isnan(data.power.input_voltage)) {
        data.power.input_voltage = read_usage_in_collection(map, report_cache,
            HID_USAGE_POW(HID_USAGE_POW_VOLTAGE),
            HID_USAGE_POW(HID_USAGE_POW_POWER_SUMMARY),
            "input.voltage.powersummary");
    }
    // Apply descriptor-specific voltage scaling for devices with bad exponents
    if (descriptor_needs_raw_extraction_ && !std::isnan(data.power.input_voltage)) {
        data.power.input_voltage *= descriptor_voltage_scale_;
    }
    // Treat near-zero input voltage as absent (on battery, no grid)
    if (!std::isnan(data.power.input_voltage) && data.power.input_voltage < 1.0f) {
        data.power.input_voltage = NAN;
    }

    // Input frequency (in Input collection)
    data.power.frequency = read_usage_in_collection(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_FREQUENCY),
        HID_USAGE_POW(HID_USAGE_POW_INPUT),
        "input.frequency");
    if (std::isnan(data.power.frequency)) {
        data.power.frequency = read_usage_value(map, report_cache,
            HID_USAGE_POW(HID_USAGE_POW_FREQUENCY), "input.frequency.global");
    }
    if (descriptor_needs_raw_extraction_ && !std::isnan(data.power.frequency)) {
        data.power.frequency *= descriptor_frequency_scale_;
    }
    // Treat near-zero frequency as absent (on battery, no grid)
    if (!std::isnan(data.power.frequency) && data.power.frequency < 1.0f) {
        data.power.frequency = NAN;
    }

    // --- Output data ---
    // Output voltage (Voltage in Output collection)
    data.power.output_voltage = read_usage_in_collection(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_VOLTAGE),
        HID_USAGE_POW(HID_USAGE_POW_OUTPUT),
        "output.voltage");
    if (descriptor_needs_raw_extraction_ && !std::isnan(data.power.output_voltage)) {
        data.power.output_voltage *= descriptor_voltage_scale_;
    }

    // Output current (Current in Output collection)
    data.power.output_current = read_usage_in_collection(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_CURRENT),
        HID_USAGE_POW(HID_USAGE_POW_OUTPUT),
        "output.current");
    if (std::isnan(data.power.output_current)) {
        data.power.output_current = read_usage_value(map, report_cache,
            HID_USAGE_POW(HID_USAGE_POW_CURRENT), "output.current.global");
    }
    if (descriptor_needs_raw_extraction_ && !std::isnan(data.power.output_current)) {
        data.power.output_current *= 0.1f;  // raw value is in deci-amps
    }

    // Output frequency (Frequency in Output collection)
    data.power.output_frequency = read_usage_in_collection(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_FREQUENCY),
        HID_USAGE_POW(HID_USAGE_POW_OUTPUT),
        "output.frequency");
    if (descriptor_needs_raw_extraction_ && !std::isnan(data.power.output_frequency)) {
        data.power.output_frequency *= descriptor_frequency_scale_;
    }

    // Active power (live watts, in Output collection or global)
    data.power.active_power = read_usage_in_collection(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_ACTIVE_POWER),
        HID_USAGE_POW(HID_USAGE_POW_OUTPUT),
        "output.active_power");
    if (std::isnan(data.power.active_power)) {
        data.power.active_power = read_usage_value(map, report_cache,
            HID_USAGE_POW(HID_USAGE_POW_ACTIVE_POWER), "active_power.global");
    }

    // --- Load ---
    data.power.load_percent = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_PERCENT_LOAD), "ups.load");

    // --- Nominal / configuration values ---
    // Config voltage (nominal input/output)
    float config_voltage = read_usage_in_collection(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_CONFIG_VOLTAGE),
        HID_USAGE_POW(HID_USAGE_POW_INPUT),
        "input.voltage.nominal");
    if (std::isnan(config_voltage)) {
        config_voltage = read_usage_value(map, report_cache,
            HID_USAGE_POW(HID_USAGE_POW_CONFIG_VOLTAGE), "voltage.nominal.global");
    }
    if (!std::isnan(config_voltage) && config_voltage >= 80.0f && config_voltage <= 260.0f) {
        data.power.input_voltage_nominal = config_voltage;
        if (std::isnan(data.power.output_voltage_nominal)) {
            data.power.output_voltage_nominal = config_voltage;
        }
    }

    // Config frequency
    float config_freq = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_CONFIG_FREQUENCY), "input.frequency.nominal");
    if (!std::isnan(config_freq) && config_freq >= 45.0f && config_freq <= 65.0f) {
        data.power.input_frequency_nominal = config_freq;
    }

    // Apparent power nominal (VA)
    data.power.apparent_power_nominal = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_CONFIG_APPARENT_POWER), "ups.power.nominal");

    // Active power nominal (W)
    data.power.realpower_nominal = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_CONFIG_ACTIVE_POWER), "ups.realpower.nominal");

    // Estimate real power if only apparent is available
    if (std::isnan(data.power.realpower_nominal) && !std::isnan(data.power.apparent_power_nominal)) {
        data.power.realpower_nominal = data.power.apparent_power_nominal * 0.6f;
    }

    // --- Transfer limits ---
    float low_transfer = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_LOW_VOLTAGE_TRANSFER), "input.transfer.low");
    if (!std::isnan(low_transfer)) {
        data.power.input_transfer_low = low_transfer;
    }

    float high_transfer = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_HIGH_VOLTAGE_TRANSFER), "input.transfer.high");
    if (!std::isnan(high_transfer)) {
        data.power.input_transfer_high = high_transfer;
    }

    // --- Status flags ---
    // AC Present / Good / Overload / etc.
    float present = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_PRESENT), "ups.status.present");
    float good = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_GOOD), "ups.status.good");
    float overload = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_OVERLOAD), "ups.status.overload");
    float internal_failure = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_INTERNAL_FAILURE), "ups.status.internal_failure");
    float voltage_oor = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_VOLTAGE_OUT_OF_RANGE), "ups.status.voltage_oor");
    float shutdown_imminent_val = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_SHUTDOWN_IMMINENT), "ups.status.shutdown_imminent");
    float awaiting_power = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_AWAITING_POWER), "ups.status.awaiting_power");

    // AVR status flags
    float boost_val = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_BOOST), "ups.status.boost");
    float buck_val = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_BUCK), "ups.status.buck");
    float overtemp_val = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_OVER_TEMPERATURE), "ups.status.overtemp");
    float commlost_val = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_COMMUNICATION_LOST), "ups.status.commlost");

    data.power.boost_active = (!std::isnan(boost_val) && boost_val > 0);
    data.power.buck_active = (!std::isnan(buck_val) && buck_val > 0);
    data.power.over_temperature = (!std::isnan(overtemp_val) && overtemp_val > 0);
    data.power.communication_lost = (!std::isnan(commlost_val) && commlost_val > 0);
    data.power.shutdown_imminent = (!std::isnan(shutdown_imminent_val) && shutdown_imminent_val > 0);
    data.power.awaiting_power = (!std::isnan(awaiting_power) && awaiting_power > 0);
    data.power.voltage_out_of_range = (!std::isnan(voltage_oor) && voltage_oor > 0);

    // Determine power status (input_voltage is already scaled/converted at this point).
    // If none of the indicators yield a definitive answer, leave status empty
    // so the merge layer keeps the previous known-good value.
    if (data.power.status.empty()) {
        if (!std::isnan(discharging) && discharging > 0) {
            data.power.status = status::ON_BATTERY;
        } else if (!std::isnan(ac_present) && ac_present > 0) {
            data.power.status = status::ONLINE;
        } else if (!std::isnan(data.power.input_voltage) && data.power.input_voltage > 10.0f) {
            data.power.status = status::ONLINE;
        } else if (!std::isnan(present) && present > 0) {
            data.power.status = status::ONLINE;
        } else if (!std::isnan(data.power.output_voltage) && data.power.output_voltage > 10.0f) {
            data.power.status = status::ONLINE;
        } else {
            ESP_LOGW(TL_TAG,
                "Status undetermined: discharging=%s, ac_present=%s, "
                "input_v=%s, present=%s, output_v=%s",
                std::isnan(discharging) ? "NaN" : (discharging > 0 ? "1" : "0"),
                std::isnan(ac_present) ? "NaN" : (ac_present > 0 ? "1" : "0"),
                std::isnan(data.power.input_voltage) ? "NaN" :
                    std::to_string(data.power.input_voltage).c_str(),
                std::isnan(present) ? "NaN" : (present > 0 ? "1" : "0"),
                std::isnan(data.power.output_voltage) ? "NaN" :
                    std::to_string(data.power.output_voltage).c_str());
        }
    }

    if (!std::isnan(overload) && overload > 0) {
        data.power.status = "Overload";
    }

    // Append boost/buck to status for AVR UPS
    if (data.power.boost_active && data.power.status == status::ONLINE) {
        data.power.status = "Online (Boost)";
    } else if (data.power.buck_active && data.power.status == status::ONLINE) {
        data.power.status = "Online (Trim)";
    }

    // If fully discharged, note it in battery status
    if (!std::isnan(fully_discharged) && fully_discharged > 0) {
        data.battery.status = "Depleted";
    }

    // If we found input voltage but not output, assume output = input when online
    if (!std::isnan(data.power.input_voltage) && std::isnan(data.power.output_voltage) &&
        (data.power.status == status::ONLINE || data.power.status == "Online (Boost)" || data.power.status == "Online (Trim)")) {
        data.power.output_voltage = data.power.input_voltage;
    }

    // --- Beeper status ---
    float beeper_val = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_AUDIBLE_ALARM_CONTROL), "ups.beeper");
    if (!std::isnan(beeper_val)) {
        int beeper_int = static_cast<int>(beeper_val);
        switch (beeper_int) {
            case 1: data.config.beeper_status = "disabled"; data.config.beeper_state = ConfigData::BEEPER_DISABLED; break;
            case 2: data.config.beeper_status = "enabled"; data.config.beeper_state = ConfigData::BEEPER_ENABLED; break;
            case 3: data.config.beeper_status = "muted"; data.config.beeper_state = ConfigData::BEEPER_MUTED; break;
        }
    }

    // --- Delay/timer values ---
    float shutdown_delay = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_DELAY_BEFORE_SHUTDOWN), "ups.delay.shutdown");
    if (!std::isnan(shutdown_delay)) {
        int16_t sd = static_cast<int16_t>(shutdown_delay);
        if (sd == static_cast<int16_t>(TIMER_INACTIVE) || sd < 0) {
            data.test.timer_shutdown = -1;
        } else {
            data.config.delay_shutdown = sd;
            data.test.timer_shutdown = sd;
        }
    }

    float start_delay = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_DELAY_BEFORE_STARTUP), "ups.delay.start");
    if (!std::isnan(start_delay)) {
        int16_t sd = static_cast<int16_t>(start_delay);
        if (sd == static_cast<int16_t>(TIMER_INACTIVE) || sd < 0) {
            data.test.timer_start = -1;
        } else {
            data.config.delay_start = sd;
            data.test.timer_start = sd;
        }
    }

    float reboot_delay = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_DELAY_BEFORE_REBOOT), "ups.delay.reboot");
    if (!std::isnan(reboot_delay)) {
        int16_t rd = static_cast<int16_t>(reboot_delay);
        if (rd == static_cast<int16_t>(TIMER_INACTIVE) || rd < 0) {
            data.test.timer_reboot = -1;
        } else {
            data.config.delay_reboot = rd;
            data.test.timer_reboot = rd;
        }
    }

    // --- Test result ---
    float test_val = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_TEST), "ups.test.result");
    if (!std::isnan(test_val)) {
        int test_int = static_cast<int>(test_val);
        switch (test_int) {
            case 1: data.test.ups_test_result = test::RESULT_DONE_PASSED; break;
            case 2: data.test.ups_test_result = test::RESULT_DONE_WARNING; break;
            case 3: data.test.ups_test_result = test::RESULT_DONE_ERROR; break;
            case 4: data.test.ups_test_result = test::RESULT_ABORTED; break;
            case 5: data.test.ups_test_result = test::RESULT_IN_PROGRESS; break;
            case 6: data.test.ups_test_result = test::RESULT_NO_TEST; break;
        }
    }
    if (data.test.ups_test_result.empty()) {
        data.test.ups_test_result = test::RESULT_NO_TEST;
    }

    // --- Warning/low battery thresholds ---
    float warning_cap = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_WARNING_CAPACITY_LIMIT), "battery.charge.warning");
    if (!std::isnan(warning_cap)) {
        data.battery.charge_warning = warning_cap;
    }

    // --- Tripp Lite vendor-specific fields (page 0xFFFF) ---
    // UPS Firmware version (FFFF:007C) - value is a firmware revision number
    float fw_version_raw = read_usage_value(map, report_cache,
        HID_USAGE_TL(HID_USAGE_TL_UPS_FIRMWARE_VERSION), "ups.firmware.version");
    if (!std::isnan(fw_version_raw) && fw_version_raw > 0) {
        int fw_int = static_cast<int>(fw_version_raw);
        // Tripp Lite firmware version is typically a small integer (e.g. 161)
        // Format it with a decimal point: 161 -> "1.61", 200 -> "2.00"
        if (fw_int >= 100 && fw_int < 10000) {
            char fw_buf[16];
            snprintf(fw_buf, sizeof(fw_buf), "%d.%02d", fw_int / 100, fw_int % 100);
            data.device.firmware_version = fw_buf;
        } else {
            data.device.firmware_version = std::to_string(fw_int);
        }
        ESP_LOGD(TL_TAG, "Firmware version: %s (raw: %d)", data.device.firmware_version.c_str(), fw_int);
    }

    // Communication protocol version (FFFF:007D) - matches product ID
    float comm_proto = read_usage_value(map, report_cache,
        HID_USAGE_TL(HID_USAGE_TL_COMM_PROTOCOL_VERSION), "ups.comm.protocol");
    if (!std::isnan(comm_proto) && comm_proto > 0) {
        ESP_LOGD(TL_TAG, "Comm protocol version: 0x%04X (PID: 0x%04X)",
                 static_cast<int>(comm_proto), parent_->get_product_id());
    }

    // Watchdog timer (FFFF:0092) - read for logging
    float watchdog_val = read_usage_value(map, report_cache,
        HID_USAGE_TL(HID_USAGE_TL_WATCHDOG), "ups.watchdog");
    if (!std::isnan(watchdog_val)) {
        ESP_LOGD(TL_TAG, "Watchdog timer: %d", static_cast<int>(watchdog_val));
    }

    // --- Manufacture date (USB HID packed format) ---
    float mfr_date_raw = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_MANUFACTURE_DATE), "battery.mfr_date");
    if (!std::isnan(mfr_date_raw) && mfr_date_raw > 0) {
        // USB HID ManufactureDate is packed: (year-1980)*512 + month*32 + day
        uint16_t date_packed = static_cast<uint16_t>(mfr_date_raw);
        int year = ((date_packed >> 9) & 0x7F) + 1980;
        int month = (date_packed >> 5) & 0x0F;
        int day = date_packed & 0x1F;
        if (year >= 1980 && year <= 2100 && month >= 1 && month <= 12 && day >= 1 && day <= 31) {
            char date_buf[16];
            snprintf(date_buf, sizeof(date_buf), "%04d-%02d-%02d", year, month, day);
            data.battery.mfr_date = date_buf;
            ESP_LOGD(TL_TAG, "Manufacture date: %s (packed: 0x%04X)", date_buf, date_packed);
        } else {
            ESP_LOGD(TL_TAG, "Manufacture date packed value 0x%04X decodes to invalid date %d-%d-%d",
                     date_packed, year, month, day);
        }
    }

    // Set USB identification
    data.device.usb_vendor_id = parent_->get_vendor_id();
    data.device.usb_product_id = parent_->get_product_id();

    // Determine success
    bool success = !std::isnan(data.power.input_voltage) ||
                   !std::isnan(data.battery.level) ||
                   !std::isnan(data.power.load_percent);

    if (success) {
        char cur_buf[8] = "?";
        if (!std::isnan(data.power.output_current)) {
            snprintf(cur_buf, sizeof(cur_buf), "%.1f", data.power.output_current);
        }
        ESP_LOGI(TL_TAG, "Data read OK (descriptor): bat=%s%%, in=%sV, out=%sV, load=%s%%, freq=%sHz, cur=%sA, pwr=%sW",
                 !std::isnan(data.battery.level) ? std::to_string(static_cast<int>(data.battery.level)).c_str() : "?",
                 !std::isnan(data.power.input_voltage) ? std::to_string(static_cast<int>(data.power.input_voltage)).c_str() : "?",
                 !std::isnan(data.power.output_voltage) ? std::to_string(static_cast<int>(data.power.output_voltage)).c_str() : "?",
                 !std::isnan(data.power.load_percent) ? std::to_string(static_cast<int>(data.power.load_percent)).c_str() : "?",
                 !std::isnan(data.power.frequency) ? std::to_string(static_cast<int>(data.power.frequency)).c_str() : "?",
                 cur_buf,
                 !std::isnan(data.power.active_power) ? std::to_string(static_cast<int>(data.power.active_power)).c_str() : "?");

        // Log unused descriptor fields summary and their values
        map->log_field_summary(TL_TAG, queried_usages_);
        map->log_unused_field_values(TL_TAG, queried_usages_, report_cache);
    } else {
        ESP_LOGW(TL_TAG, "Descriptor-based reading produced no usable data, falling back to heuristic");
        use_descriptor_ = false;
        return read_data_heuristic(data);
    }

    return success;
}


// ============================================================================
// Heuristic Data Reading (fallback)
// ============================================================================
//
// When the HID report descriptor is not available, reads ALL reports and
// classifies values by range (voltage, frequency, percentage, etc.)
// ============================================================================

bool TrippLiteProtocol::read_data_heuristic(UpsData &data) {
    ESP_LOGV(TL_TAG, "Reading Tripp Lite HID data (heuristic mode)...");

    // Step 1: Read ALL available feature reports
    std::map<uint8_t, HidReport> all_reports;
    int reports_read = 0;

    for (uint8_t rid : available_feature_reports_) {
        HidReport report;
        if (read_hid_report(rid, report) && !report.data.empty()) {
            all_reports[rid] = report;
            reports_read++;
        }
    }

    if (reports_read == 0) {
        ESP_LOGW(TL_TAG, "No reports could be read");
        return false;
    }

    // Step 2: Log ALL raw report data for debugging/mapping
    ESP_LOGI(TL_TAG, "Read %d reports from device:", reports_read);
    for (const auto &pair : all_reports) {
        uint8_t rid = pair.first;
        const auto &rpt = pair.second;
        if (rpt.data.size() == 2) {
            ESP_LOGI(TL_TAG, "  Report 0x%02X (%zuB): [0x%02X] = %d",
                     rid, rpt.data.size(), rpt.data[1], rpt.data[1]);
        } else if (rpt.data.size() == 3) {
            uint16_t val16 = rpt.data[1] | (rpt.data[2] << 8);
            ESP_LOGI(TL_TAG, "  Report 0x%02X (%zuB): [0x%02X 0x%02X] = %d (16-bit LE)",
                     rid, rpt.data.size(), rpt.data[1], rpt.data[2], val16);
        } else if (rpt.data.size() > 3) {
            std::string hex;
            for (size_t i = 1; i < rpt.data.size() && i < 8; i++) {
                char buf[8];
                snprintf(buf, sizeof(buf), "0x%02X ", rpt.data[i]);
                hex += buf;
            }
            ESP_LOGI(TL_TAG, "  Report 0x%02X (%zuB): %s", rid, rpt.data.size(), hex.c_str());
        }
    }

    // Step 3: Classify reports by value range
    //
    // Classification strategy (informed by ECO850LCD PID 0x3024 data):
    //   - 1-byte reports: voltage (90-140/200-260), frequency (45-70), load (0-100)
    //   - 3-byte reports: battery charge (0-100), battery voltage, runtime,
    //                     nominal power, timers (0xFFFF = inactive)
    //   - Small values (0-10) in 1-byte reports are likely thresholds/config, not charge
    //   - Battery charge is more reliably found in 3-byte (16-bit) reports
    //
    // Track what we've assigned to avoid double-assignment
    std::set<uint8_t> classified_rids;
    bool found_input_voltage = false;
    bool found_output_voltage = false;
    bool found_frequency = false;
    bool found_battery_charge = false;
    bool found_load = false;
    bool found_battery_voltage = false;
    bool found_runtime = false;
    bool found_nominal_power = false;

    // === Pass 1: 1-byte reports - high-confidence classifications ===
    for (const auto &pair : all_reports) {
        uint8_t rid = pair.first;
        const auto &rpt = pair.second;
        if (rpt.data.size() != 2) continue;

        uint8_t val = rpt.data[1];

        // Input voltage: 90-140V (US) or 200-260V (EU)
        if (!found_input_voltage && val >= 90 && val <= 140) {
            data.power.input_voltage = static_cast<float>(val);
            ESP_LOGI(TL_TAG, "Classified report 0x%02X = %d as input.voltage", rid, val);
            found_input_voltage = true;
            classified_rids.insert(rid);
            continue;
        }
        if (!found_input_voltage && val >= 200 && val <= 260) {
            data.power.input_voltage = static_cast<float>(val);
            ESP_LOGI(TL_TAG, "Classified report 0x%02X = %d as input.voltage (EU)", rid, val);
            found_input_voltage = true;
            classified_rids.insert(rid);
            continue;
        }

        // Frequency: 45-70Hz (distinctive range, unlikely to conflict)
        if (!found_frequency && val >= 45 && val <= 70) {
            data.power.frequency = static_cast<float>(val);
            ESP_LOGI(TL_TAG, "Classified report 0x%02X = %d as input.frequency", rid, val);
            found_frequency = true;
            classified_rids.insert(rid);
            continue;
        }
    }

    // === Pass 2: 3-byte reports - battery charge, voltage, runtime, power ===
    for (const auto &pair : all_reports) {
        uint8_t rid = pair.first;
        const auto &rpt = pair.second;
        if (rpt.data.size() != 3) continue;

        uint16_t val16 = rpt.data[1] | (rpt.data[2] << 8);

        // Skip timer values (0xFFFF = inactive)
        if (val16 == 0xFFFF) {
            ESP_LOGD(TL_TAG, "Report 0x%02X = 65535 (timer inactive)", rid);
            classified_rids.insert(rid);
            continue;
        }

        // Battery charge: 0-100 in 16-bit (more reliable than 1-byte small values)
        if (!found_battery_charge && val16 >= 0 && val16 <= 100) {
            data.battery.level = static_cast<float>(val16);
            ESP_LOGI(TL_TAG, "Classified report 0x%02X = %d as battery.charge", rid, val16);
            found_battery_charge = true;
            classified_rids.insert(rid);
            continue;
        }

        // Battery voltage: typically 6-60V range after scaling
        if (!found_battery_voltage && val16 > 0 && val16 < 1000) {
            float voltage = apply_battery_voltage_scale(static_cast<float>(val16));
            if (voltage >= 3.0f && voltage <= 60.0f) {
                data.battery.voltage = voltage;
                ESP_LOGI(TL_TAG, "Classified report 0x%02X = %d (scaled %.1f) as battery.voltage",
                         rid, val16, voltage);
                found_battery_voltage = true;
                classified_rids.insert(rid);
                continue;
            }
        }

        // Nominal power (VA rating): typically 300-5000, matches model number
        if (!found_nominal_power && val16 >= 300 && val16 <= 10000) {
            data.power.apparent_power_nominal = static_cast<float>(val16);
            // Estimate real power at ~60% of apparent power (typical for consumer UPS)
            data.power.realpower_nominal = static_cast<float>(val16) * 0.6f;
            ESP_LOGI(TL_TAG, "Classified report 0x%02X = %d as ups.power.nominal (VA)", rid, val16);
            found_nominal_power = true;
            classified_rids.insert(rid);
            continue;
        }

        // Runtime in seconds: >100 and <86400 (remaining after charge, voltage, power)
        if (!found_runtime && val16 > 100 && val16 < 86400) {
            data.battery.runtime_minutes = static_cast<float>(val16) / 60.0f;
            ESP_LOGI(TL_TAG, "Classified report 0x%02X = %d sec (%.1f min) as battery.runtime",
                     rid, val16, data.battery.runtime_minutes);
            found_runtime = true;
            classified_rids.insert(rid);
            continue;
        }
    }

    // === Pass 3: 1-byte reports - load percentage (skip thresholds) ===
    for (const auto &pair : all_reports) {
        uint8_t rid = pair.first;
        const auto &rpt = pair.second;
        if (rpt.data.size() != 2 || classified_rids.count(rid)) continue;

        uint8_t val = rpt.data[1];

        // Load percentage: 0-100, but skip very small values (<= 10) that
        // are likely config thresholds (charge.low, warning levels)
        if (!found_load && val > 0 && val <= 100) {
            data.power.load_percent = static_cast<float>(val);
            ESP_LOGI(TL_TAG, "Classified report 0x%02X = %d as ups.load", rid, val);
            found_load = true;
            classified_rids.insert(rid);
            continue;
        }
    }

    // Log unclassified reports for future mapping
    for (const auto &pair : all_reports) {
        if (!classified_rids.count(pair.first)) {
            uint8_t rid = pair.first;
            const auto &rpt = pair.second;
            if (rpt.data.size() == 2) {
                ESP_LOGD(TL_TAG, "Unclassified report 0x%02X (1B): %d", rid, rpt.data[1]);
            } else if (rpt.data.size() == 3) {
                uint16_t v = rpt.data[1] | (rpt.data[2] << 8);
                ESP_LOGD(TL_TAG, "Unclassified report 0x%02X (2B): %d", rid, v);
            }
        }
    }

    // Step 4: Determine power status from available data
    if (found_input_voltage && data.power.input_voltage > 0) {
        data.power.status = status::ONLINE;
    } else {
        data.power.status = status::ON_BATTERY;
    }

    // Set nominals based on detected input voltage
    if (found_input_voltage) {
        if (data.power.input_voltage >= 90.0f && data.power.input_voltage <= 140.0f) {
            if (std::isnan(data.power.input_voltage_nominal)) data.power.input_voltage_nominal = 120.0f;
            if (std::isnan(data.power.output_voltage_nominal)) data.power.output_voltage_nominal = 120.0f;
        } else if (data.power.input_voltage >= 200.0f && data.power.input_voltage <= 260.0f) {
            if (std::isnan(data.power.input_voltage_nominal)) data.power.input_voltage_nominal = 230.0f;
            if (std::isnan(data.power.output_voltage_nominal)) data.power.output_voltage_nominal = 230.0f;
        }
    }

    // If we found input voltage but not output, assume output = input when online
    if (found_input_voltage && !found_output_voltage && data.power.status == status::ONLINE) {
        data.power.output_voltage = data.power.input_voltage;
    }

    // Set default test result
    if (data.test.ups_test_result.empty()) {
        data.test.ups_test_result = test::RESULT_NO_TEST;
    }

    // Determine success: we need at least some basic data
    bool success = found_input_voltage || found_battery_charge || found_load;

    if (success) {
        ESP_LOGI(TL_TAG, "Data read OK: battery=%s%%, input=%sV, output=%sV, load=%s%%, freq=%sHz",
                 found_battery_charge ? std::to_string(static_cast<int>(data.battery.level)).c_str() : "?",
                 found_input_voltage ? std::to_string(static_cast<int>(data.power.input_voltage)).c_str() : "?",
                 found_output_voltage ? std::to_string(static_cast<int>(data.power.output_voltage)).c_str() : "?",
                 found_load ? std::to_string(static_cast<int>(data.power.load_percent)).c_str() : "?",
                 found_frequency ? std::to_string(static_cast<int>(data.power.frequency)).c_str() : "?");
    } else {
        ESP_LOGW(TL_TAG, "Could not classify any reports into useful UPS data");
        ESP_LOGW(TL_TAG, "This device may need a custom report mapping - please share the report dump above");
    }

    return success;
}


// ============================================================================
// Device Information Reading
// ============================================================================

void TrippLiteProtocol::read_device_information(UpsData &data) {
    ESP_LOGD(TL_TAG, "Reading Tripp Lite device information...");

    // The HID report descriptor contains iManufacturer, iProduct, and iSerialNumber
    // fields whose VALUES are USB string descriptor indices. We read those HID reports
    // to discover the correct string descriptor indices, rather than hardcoding them.
    //
    // Fallback indices (common for Tripp Lite):
    //   Index 1: Product name (e.g., "ECO850LCD")
    //   Index 2: Manufacturer (e.g., "Tripp Lite")
    //   Index 3: Serial number (may be empty on some models)

    int mfr_idx = 2;      // Default manufacturer string descriptor index
    int product_idx = 1;   // Default product string descriptor index
    int serial_idx = 3;    // Default serial string descriptor index
    int chemistry_idx = -1; // No default; discovered from HID descriptor

    // Try to read the actual string descriptor indices from HID reports
    const HidReportMap* map = parent_->get_report_map();
    if (map && use_descriptor_) {
        for (const auto& f : map->get_all_fields()) {
            if (f.usage == HID_USAGE_POW(HID_USAGE_POW_I_MANUFACTURER)) {
                HidReport report;
                if (read_hid_report(f.report_id, report) && !report.data.empty()) {
                    int idx = static_cast<int>(map->extract_raw_value(f, report.data.data(), report.data.size()));
                    if (idx > 0 && idx < 256) {
                        mfr_idx = idx;
                        ESP_LOGD(TL_TAG, "HID iManufacturer index: %d (report 0x%02X)", idx, f.report_id);
                    }
                }
            } else if (f.usage == HID_USAGE_POW(HID_USAGE_POW_I_PRODUCT)) {
                HidReport report;
                if (read_hid_report(f.report_id, report) && !report.data.empty()) {
                    int idx = static_cast<int>(map->extract_raw_value(f, report.data.data(), report.data.size()));
                    if (idx > 0 && idx < 256) {
                        product_idx = idx;
                        ESP_LOGD(TL_TAG, "HID iProduct index: %d (report 0x%02X)", idx, f.report_id);
                    }
                }
            } else if (f.usage == HID_USAGE_POW(HID_USAGE_POW_I_SERIAL_NUMBER)) {
                HidReport report;
                if (read_hid_report(f.report_id, report) && !report.data.empty()) {
                    int idx = static_cast<int>(map->extract_raw_value(f, report.data.data(), report.data.size()));
                    if (idx > 0 && idx < 256) {
                        serial_idx = idx;
                        ESP_LOGD(TL_TAG, "HID iSerialNumber index: %d (report 0x%02X)", idx, f.report_id);
                    }
                }
            } else if (f.usage == HID_USAGE_BAT(HID_USAGE_BAT_I_DEVICE_CHEMISTRY)) {
                HidReport report;
                if (read_hid_report(f.report_id, report) && !report.data.empty()) {
                    int idx = static_cast<int>(map->extract_raw_value(f, report.data.data(), report.data.size()));
                    if (idx > 0 && idx < 256) {
                        chemistry_idx = idx;
                        ESP_LOGD(TL_TAG, "HID iDeviceChemistry index: %d (report 0x%02X)", idx, f.report_id);
                    }
                }
            }
        }
    }

    ESP_LOGI(TL_TAG, "String descriptor indices: mfr=%d, product=%d, serial=%d, chemistry=%d",
             mfr_idx, product_idx, serial_idx, chemistry_idx);

    std::string str_val;
    esp_err_t ret;

    // Manufacturer
    ret = parent_->get_string_descriptor(mfr_idx, str_val);
    if (ret == ESP_OK && !str_val.empty()) {
        data.device.manufacturer = str_val;
        ESP_LOGI(TL_TAG, "Manufacturer: \"%s\"", data.device.manufacturer.c_str());
    } else {
        data.device.manufacturer = "Tripp Lite";
        ESP_LOGD(TL_TAG, "Using default manufacturer: Tripp Lite");
    }

    // Model/product
    ret = parent_->get_string_descriptor(product_idx, str_val);
    if (ret == ESP_OK && !str_val.empty()) {
        data.device.model = str_val;
        ESP_LOGI(TL_TAG, "Model: \"%s\"", data.device.model.c_str());
    } else {
        char model_str[32];
        snprintf(model_str, sizeof(model_str), "Tripp Lite UPS %04X", parent_->get_product_id());
        data.device.model = model_str;
    }

    // Serial number
    ret = parent_->get_string_descriptor(serial_idx, str_val);
    if (ret == ESP_OK && !str_val.empty()) {
        // Validate: serial should not match manufacturer or product name
        if (str_val != data.device.manufacturer && str_val != data.device.model) {
            data.device.serial_number = str_val;
            ESP_LOGI(TL_TAG, "Serial: \"%s\"", data.device.serial_number.c_str());
        } else {
            ESP_LOGW(TL_TAG, "Serial at index %d matches manufacturer/model (\"%s\"), scanning for actual serial...",
                     serial_idx, str_val.c_str());
            // Scan nearby indices for a real serial number
            bool found_serial = false;
            for (int try_idx = 1; try_idx <= 8 && !found_serial; try_idx++) {
                if (try_idx == mfr_idx || try_idx == product_idx || try_idx == serial_idx) continue;
                ret = parent_->get_string_descriptor(try_idx, str_val);
                if (ret == ESP_OK && !str_val.empty() &&
                    str_val != data.device.manufacturer && str_val != data.device.model) {
                    data.device.serial_number = str_val;
                    ESP_LOGI(TL_TAG, "Serial found at index %d: \"%s\"", try_idx, str_val.c_str());
                    found_serial = true;
                }
            }
            if (!found_serial) {
                ESP_LOGW(TL_TAG, "No unique serial number found in string descriptors");
            }
        }
    } else {
        ESP_LOGD(TL_TAG, "No serial number at index %d", serial_idx);
    }

    // Battery chemistry
    if (chemistry_idx > 0) {
        ret = parent_->get_string_descriptor(chemistry_idx, str_val);
    } else {
        // Fallback: try index 4 (common Tripp Lite convention)
        ret = parent_->get_string_descriptor(4, str_val);
    }
    if (ret == ESP_OK && !str_val.empty()) {
        data.battery.type = str_val;
        ESP_LOGI(TL_TAG, "Battery type: \"%s\"", data.battery.type.c_str());
    }

    // Set USB identification info
    data.device.usb_vendor_id = parent_->get_vendor_id();
    data.device.usb_product_id = parent_->get_product_id();

    device_info_read_ = true;
    ESP_LOGD(TL_TAG, "Device info reading complete");
}


// ============================================================================
// Data Parsers
// ============================================================================

void TrippLiteProtocol::parse_battery_data(UpsData &data) {
    // Try to read battery-related reports
    // NUT maps: battery.charge -> UPS.PowerSummary.RemainingCapacity
    //           battery.runtime -> UPS.PowerSummary.RunTimeToEmpty
    //           battery.voltage -> UPS.BatterySystem.Battery.Voltage
    //           battery.voltage.nominal -> UPS.BatterySystem.Battery.ConfigVoltage

    // Try multiple report IDs that commonly contain battery data
    HidReport report;

    // Try standard Power Summary reports for battery % and runtime
    // These report IDs are commonly used but device-specific - try several
    const uint8_t battery_report_ids[] = {
        HID_USAGE_POW_POWER_SUMMARY,       // 0x24
        0x0C,                               // Common power summary
        0x08,                               // Battery runtime (CyberPower-style)
        0x07,                               // Battery capacity
    };

    for (uint8_t rid : battery_report_ids) {
        if (read_hid_report(rid, report) && report.data.size() >= 2) {
            // Try to extract battery percentage
            if (std::isnan(data.battery.level)) {
                uint8_t battery_pct = report.data[1];
                if (battery_pct <= 100) {
                    data.battery.level = static_cast<float>(battery_pct);
                    ESP_LOGD(TL_TAG, "Battery level: %d%% (report 0x%02X)", battery_pct, rid);
                }
            }

            // Try to extract runtime (16-bit LE at bytes 2-3, in seconds)
            if (std::isnan(data.battery.runtime_minutes) && report.data.size() >= 4) {
                uint16_t runtime_raw = read_16bit_le_value(report, 2);
                if (runtime_raw > 0 && runtime_raw < TIMER_INACTIVE) {
                    // NUT reports runtime in seconds for Tripp Lite
                    data.battery.runtime_minutes = static_cast<float>(runtime_raw) / 60.0f;
                    ESP_LOGD(TL_TAG, "Battery runtime: %.1f min (%d sec, report 0x%02X)",
                             data.battery.runtime_minutes, runtime_raw, rid);
                }
            }
        }
    }

    // Try to read battery voltage
    const uint8_t voltage_report_ids[] = {
        HID_USAGE_POW_VOLTAGE,              // 0x30 (may be battery voltage in some contexts)
        0x0A,                               // CyberPower-style battery voltage
        0x40,                               // Battery system
    };

    for (uint8_t rid : voltage_report_ids) {
        if (std::isnan(data.battery.voltage) && read_hid_report(rid, report) && report.data.size() >= 3) {
            uint16_t voltage_raw = read_16bit_le_value(report, 1);
            if (voltage_raw > 0 && voltage_raw < 0xFFFF) {
                float voltage = apply_battery_voltage_scale(static_cast<float>(voltage_raw));
                // Validate battery voltage (typical range 6V-60V for UPS batteries)
                if (voltage >= 3.0f && voltage <= 60.0f) {
                    data.battery.voltage = voltage;
                    ESP_LOGD(TL_TAG, "Battery voltage: %.1fV (raw=%d, scale=%.4f, report 0x%02X)",
                             voltage, voltage_raw, battery_scale_, rid);
                    break;
                }
            }
        }
    }

    // Try to read battery voltage nominal (ConfigVoltage)
    const uint8_t nominal_voltage_report_ids[] = {
        HID_USAGE_POW_CONFIG_VOLTAGE,       // 0x40
        0x09,                               // CyberPower-style nominal
    };

    for (uint8_t rid : nominal_voltage_report_ids) {
        if (std::isnan(data.battery.voltage_nominal) && read_hid_report(rid, report) && report.data.size() >= 3) {
            uint16_t nominal_raw = read_16bit_le_value(report, 1);
            if (nominal_raw > 0 && nominal_raw < 0xFFFF) {
                // Nominal voltage typically doesn't need battery_scale_
                // but may need it on some models
                float nominal = static_cast<float>(nominal_raw);
                // Try direct value first
                if (nominal >= 6.0f && nominal <= 60.0f) {
                    data.battery.voltage_nominal = nominal;
                    ESP_LOGD(TL_TAG, "Battery voltage nominal: %.1fV (report 0x%02X)", nominal, rid);
                    break;
                }
                // Try with battery scale
                float scaled = apply_battery_voltage_scale(nominal);
                if (scaled >= 6.0f && scaled <= 60.0f) {
                    data.battery.voltage_nominal = scaled;
                    ESP_LOGD(TL_TAG, "Battery voltage nominal: %.1fV (scaled, report 0x%02X)", scaled, rid);
                    break;
                }
            }
        }
    }
}

void TrippLiteProtocol::parse_power_summary(UpsData &data) {
    // NUT: UPS.PowerSummary contains RemainingCapacity, RunTimeToEmpty,
    //      WarningCapacityLimit, RemainingCapacityLimit, and PresentStatus
    // These are often in reports we've already tried in parse_battery_data()
    // This method handles any remaining power summary data

    // Check for WarningCapacityLimit and RemainingCapacityLimit
    HidReport report;

    // Try to find charge warning and low thresholds
    const uint8_t threshold_report_ids[] = {0x07, 0x24};
    for (uint8_t rid : threshold_report_ids) {
        if (read_hid_report(rid, report) && report.data.size() >= 4) {
            // Some reports contain: [id, remaining_capacity, warning_limit, ...]
            // or [id, full_charge_capacity, remaining_capacity_limit, warning_capacity_limit]
            if (report.data.size() >= 3) {
                uint8_t val = report.data[2];
                if (val > 0 && val <= 100 && std::isnan(data.battery.charge_low)) {
                    data.battery.charge_low = static_cast<float>(val);
                    ESP_LOGD(TL_TAG, "Battery charge low threshold: %d%% (report 0x%02X)", val, rid);
                }
            }
            if (report.data.size() >= 4) {
                uint8_t val = report.data[3];
                if (val > 0 && val <= 100 && std::isnan(data.battery.charge_warning)) {
                    data.battery.charge_warning = static_cast<float>(val);
                    ESP_LOGD(TL_TAG, "Battery charge warning: %d%% (report 0x%02X)", val, rid);
                }
            }
        }
    }
}

void TrippLiteProtocol::parse_status_flags(UpsData &data) {
    // NUT maps multiple PresentStatus boolean values from HID reports:
    // ACPresent -> Online
    // Charging -> Charging
    // Discharging -> On Battery
    // BelowRemainingCapacityLimit -> Low Battery
    // etc.

    HidReport report;

    // Try various report IDs that commonly contain status flags
    const uint8_t status_report_ids[] = {
        HID_USAGE_POW_PRESENT_STATUS,       // 0x02
        0x0B,                               // CyberPower-style present status
        0x16,                               // Generic present status
        0x01,                               // General status
    };

    for (uint8_t rid : status_report_ids) {
        if (read_hid_report(rid, report) && report.data.size() >= 2) {
            // Status flags are typically bit-packed
            // The exact bit layout depends on the HID report descriptor
            // Try to interpret common patterns

            uint8_t status_byte = report.data[1];

            // Skip invalid values
            if (status_byte == 0xFF || status_byte == 0x00) continue;

            // Log raw status for debugging
            ESP_LOGD(TL_TAG, "Status report 0x%02X: 0x%02X", rid, status_byte);

            // For 2-byte status, check second byte too
            if (report.data.size() >= 3) {
                uint8_t status_byte2 = report.data[2];
                ESP_LOGD(TL_TAG, "  Second status byte: 0x%02X", status_byte2);

                // Some Tripp Lite devices use multi-byte status flags
                // Try interpreting as individual boolean fields
                // NUT's HID parser extracts individual bits from specific offsets
            }

            // Common Tripp Lite status interpretations:
            // Bit patterns vary by model, but these are common across many devices

            // Check for AC Present (online) indication
            if (status_byte & 0x01) {
                data.power.status = status::ONLINE;
                // Don't override existing input voltage if it's already set
                if (std::isnan(data.power.input_voltage)) {
                    data.power.input_voltage = parent_->get_fallback_nominal_voltage();
                }
            }

            // Check for Discharging (on battery)
            if (status_byte & 0x02) {
                data.power.status = status::ON_BATTERY;
                data.power.input_voltage = NAN;
            }

            // Check for Charging
            if (status_byte & 0x04) {
                if (data.battery.status.empty()) {
                    data.battery.status = battery_status::CHARGING;
                }
            }

            // Check for Low Battery
            if (status_byte & 0x08) {
                data.battery.charge_low = battery::LOW_THRESHOLD_PERCENT;
                if (data.battery.status.empty()) {
                    data.battery.status = battery_status::LOW;
                }
            }

            // Check for Fully Charged
            if (status_byte & 0x10) {
                if (data.battery.status.empty() || data.battery.status == battery_status::CHARGING) {
                    data.battery.status = battery_status::FULLY_CHARGED;
                }
            }

            // Found valid status, break
            if (!data.power.status.empty()) {
                ESP_LOGD(TL_TAG, "Status: power=%s, battery=%s (report 0x%02X)",
                         data.power.status.c_str(), data.battery.status.c_str(), rid);
                break;
            }
        }
    }
}

void TrippLiteProtocol::parse_input_data(UpsData &data) {
    HidReport report;

    // Input voltage
    // NUT: input.voltage -> UPS.PowerSummary.Input.Voltage or UPS.PowerConverter.Input.Voltage
    const uint8_t input_voltage_ids[] = {
        HID_USAGE_POW_VOLTAGE,              // 0x30
        0x0F,                               // CyberPower-style input voltage
        HID_USAGE_POW_INPUT,                // 0x1A - Input collection
    };

    for (uint8_t rid : input_voltage_ids) {
        if (read_hid_report(rid, report) && report.data.size() >= 3) {
            uint16_t voltage_raw = read_16bit_le_value(report, 1);
            if (voltage_raw > 0 && voltage_raw != 0xFFFF) {
                float voltage = apply_io_voltage_scale(static_cast<float>(voltage_raw));

                // Auto-detect if value needs additional scaling
                if (voltage > 1000.0f) {
                    voltage /= 10.0f;  // Tenths of volts
                }

                if (voltage >= voltage::MIN_VALID_VOLTAGE && voltage <= voltage::MAX_VALID_VOLTAGE) {
                    data.power.input_voltage = voltage;
                    ESP_LOGD(TL_TAG, "Input voltage: %.1fV (raw=%d, report 0x%02X)", voltage, voltage_raw, rid);
                    break;
                }
            }
        }
    }

    // Input voltage nominal
    const uint8_t input_nominal_ids[] = {
        HID_USAGE_POW_CONFIG_VOLTAGE,       // 0x40
        0x0E,                               // CyberPower-style nominal
    };

    for (uint8_t rid : input_nominal_ids) {
        if (std::isnan(data.power.input_voltage_nominal) && read_hid_report(rid, report) && report.data.size() >= 3) {
            uint16_t nominal_raw = read_16bit_le_value(report, 1);
            if (nominal_raw > 0 && nominal_raw != 0xFFFF) {
                float nominal = apply_io_voltage_scale(static_cast<float>(nominal_raw));
                if (nominal > 1000.0f) nominal /= 10.0f;

                if (nominal >= voltage::MIN_VALID_VOLTAGE && nominal <= voltage::MAX_VALID_VOLTAGE) {
                    data.power.input_voltage_nominal = nominal;
                    ESP_LOGD(TL_TAG, "Input voltage nominal: %.0fV (report 0x%02X)", nominal, rid);
                    break;
                }
            }
        }
    }
}

void TrippLiteProtocol::parse_output_data(UpsData &data) {
    HidReport report;

    // Output voltage
    // NUT: output.voltage -> UPS.PowerConverter.Output.Voltage or UPS.PowerSummary.Voltage
    const uint8_t output_voltage_ids[] = {
        HID_USAGE_POW_CURRENT,              // 0x31 (often output voltage on some devices)
        0x12,                               // CyberPower-style output voltage
        HID_USAGE_POW_OUTPUT,               // 0x1C - Output collection
    };

    for (uint8_t rid : output_voltage_ids) {
        if (read_hid_report(rid, report) && report.data.size() >= 3) {
            uint16_t voltage_raw = read_16bit_le_value(report, 1);
            if (voltage_raw > 0 && voltage_raw != 0xFFFF) {
                float voltage = apply_io_voltage_scale(static_cast<float>(voltage_raw));
                if (voltage > 1000.0f) voltage /= 10.0f;

                if (voltage >= voltage::MIN_VALID_VOLTAGE && voltage <= voltage::MAX_VALID_VOLTAGE) {
                    data.power.output_voltage = voltage;
                    ESP_LOGD(TL_TAG, "Output voltage: %.1fV (raw=%d, report 0x%02X)", voltage, voltage_raw, rid);
                    break;
                }
            }
        }
    }

    // Output voltage nominal
    // NUT: output.voltage.nominal -> UPS.Flow.ConfigVoltage
    const uint8_t output_nominal_ids[] = {
        HID_USAGE_POW_CONFIG_VOLTAGE,       // 0x40 (also used for input nominal, context-dependent)
    };

    for (uint8_t rid : output_nominal_ids) {
        if (std::isnan(data.power.output_voltage_nominal) && read_hid_report(rid, report) && report.data.size() >= 3) {
            uint16_t nominal_raw = read_16bit_le_value(report, 1);
            if (nominal_raw > 0 && nominal_raw != 0xFFFF) {
                float nominal = apply_io_voltage_scale(static_cast<float>(nominal_raw));
                if (nominal > 1000.0f) nominal /= 10.0f;

                if (nominal >= voltage::MIN_VALID_VOLTAGE && nominal <= voltage::MAX_VALID_VOLTAGE) {
                    // If input nominal is already set to the same value, this is likely
                    // the output nominal from a different context
                    if (std::isnan(data.power.input_voltage_nominal) ||
                        data.power.input_voltage_nominal == nominal) {
                        data.power.output_voltage_nominal = nominal;
                        // Also set input nominal if not already set
                        if (std::isnan(data.power.input_voltage_nominal)) {
                            data.power.input_voltage_nominal = nominal;
                        }
                        ESP_LOGD(TL_TAG, "Output voltage nominal: %.0fV (report 0x%02X)", nominal, rid);
                    }
                    break;
                }
            }
        }
    }
}

void TrippLiteProtocol::parse_load_data(UpsData &data) {
    HidReport report;

    // UPS load percentage
    // NUT: ups.load -> UPS.OutletSystem.Outlet.PercentLoad
    const uint8_t load_report_ids[] = {
        HID_USAGE_POW_PERCENT_LOAD,         // 0x35
        0x13,                               // CyberPower-style load
        HID_USAGE_POW_CONFIG_PERCENT_LOAD,  // 0x45
        0x50,                               // Common load report
    };

    for (uint8_t rid : load_report_ids) {
        if (read_hid_report(rid, report) && report.data.size() >= 2) {
            uint8_t load_raw = report.data[1];

            // Skip invalid values
            if (load_raw == 0xFF) continue;

            if (load_raw <= 100) {
                data.power.load_percent = static_cast<float>(load_raw);
                ESP_LOGD(TL_TAG, "Load: %d%% (report 0x%02X)", load_raw, rid);
                break;
            }

            // Try 16-bit value
            if (report.data.size() >= 3) {
                uint16_t load_16 = read_16bit_le_value(report, 1);
                if (load_16 <= 100) {
                    data.power.load_percent = static_cast<float>(load_16);
                    ESP_LOGD(TL_TAG, "Load: %d%% (16-bit, report 0x%02X)", load_16, rid);
                    break;
                }
            }
        }
    }
}

void TrippLiteProtocol::parse_frequency_data(UpsData &data) {
    HidReport report;

    // Input frequency
    // NUT: input.frequency -> UPS.PowerConverter.Input.Frequency
    const uint8_t freq_report_ids[] = {
        HID_USAGE_POW_FREQUENCY,            // 0x32
        HID_USAGE_POW_CONFIG_FREQUENCY,     // 0x42 (nominal frequency)
    };

    for (uint8_t rid : freq_report_ids) {
        if (read_hid_report(rid, report) && report.data.size() >= 2) {
            // Try single byte first
            float freq = read_single_byte_value(report, 1);
            freq = apply_io_frequency_scale(freq);

            if (freq >= FREQUENCY_MIN_VALID && freq <= FREQUENCY_MAX_VALID) {
                data.power.frequency = freq;
                ESP_LOGD(TL_TAG, "Frequency: %.1f Hz (report 0x%02X)", freq, rid);
                return;
            }

            // Try 16-bit value
            if (report.data.size() >= 3) {
                uint16_t freq_raw = read_16bit_le_value(report, 1);
                float freq16 = apply_io_frequency_scale(static_cast<float>(freq_raw));

                if (freq16 >= FREQUENCY_MIN_VALID && freq16 <= FREQUENCY_MAX_VALID) {
                    data.power.frequency = freq16;
                    ESP_LOGD(TL_TAG, "Frequency: %.1f Hz (16-bit, report 0x%02X)", freq16, rid);
                    return;
                }

                // Try dividing by 10 (tenths of Hz)
                float freq_tenths = static_cast<float>(freq_raw) / 10.0f;
                freq_tenths = apply_io_frequency_scale(freq_tenths);
                if (freq_tenths >= FREQUENCY_MIN_VALID && freq_tenths <= FREQUENCY_MAX_VALID) {
                    data.power.frequency = freq_tenths;
                    ESP_LOGD(TL_TAG, "Frequency: %.1f Hz (tenths, report 0x%02X)", freq_tenths, rid);
                    return;
                }
            }
        }
    }
}

void TrippLiteProtocol::parse_transfer_limits(UpsData &data) {
    HidReport report;

    // Low voltage transfer
    // NUT: input.transfer.low -> UPS.PowerConverter.Output.LowVoltageTransfer
    if (read_hid_report(HID_USAGE_POW_LOW_VOLTAGE_TRANSFER, report) && report.data.size() >= 3) {
        uint16_t low_raw = read_16bit_le_value(report, 1);
        if (low_raw > 0 && low_raw != 0xFFFF) {
            float low_voltage = apply_io_voltage_scale(static_cast<float>(low_raw));
            if (low_voltage > 1000.0f) low_voltage /= 10.0f;
            if (low_voltage >= 50.0f && low_voltage <= 200.0f) {
                data.power.input_transfer_low = low_voltage;
                ESP_LOGD(TL_TAG, "Transfer low: %.1fV", low_voltage);
            }
        }
    }

    // High voltage transfer
    // NUT: input.transfer.high -> UPS.PowerConverter.Output.HighVoltageTransfer
    if (read_hid_report(HID_USAGE_POW_HIGH_VOLTAGE_TRANSFER, report) && report.data.size() >= 3) {
        uint16_t high_raw = read_16bit_le_value(report, 1);
        if (high_raw > 0 && high_raw != 0xFFFF) {
            float high_voltage = apply_io_voltage_scale(static_cast<float>(high_raw));
            if (high_voltage > 1000.0f) high_voltage /= 10.0f;
            if (high_voltage >= 100.0f && high_voltage <= 300.0f) {
                data.power.input_transfer_high = high_voltage;
                ESP_LOGD(TL_TAG, "Transfer high: %.1fV", high_voltage);
            }
        }
    }
}

void TrippLiteProtocol::parse_power_nominal(UpsData &data) {
    HidReport report;

    // Apparent power nominal (VA rating)
    // NUT: ups.power.nominal -> UPS.Flow.ConfigApparentPower
    if (read_hid_report(HID_USAGE_POW_CONFIG_APPARENT_POWER, report) && report.data.size() >= 3) {
        uint16_t power_raw = read_16bit_le_value(report, 1);
        if (power_raw > 0 && power_raw != 0xFFFF && power_raw <= 20000) {
            data.power.apparent_power_nominal = static_cast<float>(power_raw);
            ESP_LOGD(TL_TAG, "Apparent power nominal: %.0f VA", data.power.apparent_power_nominal);
        }
    }

    // Active power nominal (W rating)
    // NUT: ups.realpower.nominal -> UPS.Flow.ConfigActivePower
    if (read_hid_report(HID_USAGE_POW_CONFIG_ACTIVE_POWER, report) && report.data.size() >= 3) {
        uint16_t power_raw = read_16bit_le_value(report, 1);
        if (power_raw > 0 && power_raw != 0xFFFF && power_raw <= 20000) {
            data.power.realpower_nominal = static_cast<float>(power_raw);
            ESP_LOGD(TL_TAG, "Real power nominal: %.0f W", data.power.realpower_nominal);
        }
    }
}

void TrippLiteProtocol::parse_beeper_status(UpsData &data) {
    HidReport report;

    // Beeper status
    // NUT: ups.beeper.status -> UPS.PowerSummary.AudibleAlarmControl
    // Tripp Lite values: 1=disabled, 2=enabled, 3=muted
    if (read_hid_report(HID_USAGE_POW_AUDIBLE_ALARM_CONTROL, report) && report.data.size() >= 2) {
        uint8_t beeper_val = report.data[1];

        switch (beeper_val) {
            case 1:
                data.config.beeper_status = "disabled";
                data.config.beeper_state = ConfigData::BEEPER_DISABLED;
                break;
            case 2:
                data.config.beeper_status = "enabled";
                data.config.beeper_state = ConfigData::BEEPER_ENABLED;
                break;
            case 3:
                data.config.beeper_status = "muted";
                data.config.beeper_state = ConfigData::BEEPER_MUTED;
                break;
            default:
                ESP_LOGD(TL_TAG, "Unknown beeper value: %d", beeper_val);
                break;
        }

        if (!data.config.beeper_status.empty()) {
            ESP_LOGD(TL_TAG, "Beeper status: %s (raw=%d)", data.config.beeper_status.c_str(), beeper_val);
        }
    }
}

void TrippLiteProtocol::parse_delay_configuration(UpsData &data) {
    HidReport report;

    // Shutdown delay
    // NUT: ups.delay.shutdown -> UPS.OutletSystem.Outlet.DelayBeforeShutdown
    if (read_hid_report(HID_USAGE_POW_DELAY_BEFORE_SHUTDOWN, report) && report.data.size() >= 3) {
        uint16_t delay_raw = read_16bit_le_value(report, 1);
        if (delay_raw != 0xFFFF && delay_raw < 7200) {
            data.config.delay_shutdown = static_cast<int16_t>(delay_raw);
            ESP_LOGD(TL_TAG, "Shutdown delay: %d sec", data.config.delay_shutdown);
        }
    }

    // Startup delay
    // NUT: ups.delay.start -> UPS.OutletSystem.Outlet.DelayBeforeStartup
    if (read_hid_report(HID_USAGE_POW_DELAY_BEFORE_STARTUP, report) && report.data.size() >= 3) {
        uint16_t delay_raw = read_16bit_le_value(report, 1);
        if (delay_raw != 0xFFFF && delay_raw < 7200) {
            data.config.delay_start = static_cast<int16_t>(delay_raw);
            ESP_LOGD(TL_TAG, "Startup delay: %d sec", data.config.delay_start);
        }
    }

    // Reboot delay
    if (read_hid_report(HID_USAGE_POW_DELAY_BEFORE_REBOOT, report) && report.data.size() >= 3) {
        uint16_t delay_raw = read_16bit_le_value(report, 1);
        if (delay_raw != 0xFFFF && delay_raw < 7200) {
            data.config.delay_reboot = static_cast<int16_t>(delay_raw);
            ESP_LOGD(TL_TAG, "Reboot delay: %d sec", data.config.delay_reboot);
        }
    }
}

void TrippLiteProtocol::parse_timer_data(UpsData &data) {
    HidReport report;

    // Timer values: 65535 (0xFFFF) means "inactive" on Tripp Lite
    // NUT: ups.timer.shutdown, ups.timer.reboot, ups.timer.start

    // Shutdown timer
    if (read_hid_report(HID_USAGE_POW_DELAY_BEFORE_SHUTDOWN, report) && report.data.size() >= 3) {
        uint16_t timer_raw = read_16bit_le_value(report, 1);
        if (timer_raw == TIMER_INACTIVE) {
            data.test.timer_shutdown = -1;  // Inactive
        } else {
            data.test.timer_shutdown = static_cast<int16_t>(timer_raw);
        }
    }

    // Reboot timer
    if (read_hid_report(HID_USAGE_POW_DELAY_BEFORE_REBOOT, report) && report.data.size() >= 3) {
        uint16_t timer_raw = read_16bit_le_value(report, 1);
        if (timer_raw == TIMER_INACTIVE) {
            data.test.timer_reboot = -1;  // Inactive
        } else {
            data.test.timer_reboot = static_cast<int16_t>(timer_raw);
        }
    }

    // Start timer
    if (read_hid_report(HID_USAGE_POW_DELAY_BEFORE_STARTUP, report) && report.data.size() >= 3) {
        uint16_t timer_raw = read_16bit_le_value(report, 1);
        if (timer_raw == TIMER_INACTIVE) {
            data.test.timer_start = -1;  // Inactive
        } else {
            data.test.timer_start = static_cast<int16_t>(timer_raw);
        }
    }
}

void TrippLiteProtocol::parse_test_result(UpsData &data) {
    HidReport report;

    // Test result
    // NUT: ups.test.result -> UPS.BatterySystem.Test
    if (read_hid_report(HID_USAGE_POW_TEST, report) && report.data.size() >= 2) {
        uint8_t test_val = report.data[1];

        // NUT test_read_info mapping:
        // 1 = Done and passed
        // 2 = Done and warning
        // 3 = Done and error
        // 4 = Aborted
        // 5 = In progress
        // 6 = No test initiated
        switch (test_val) {
            case 1:
                data.test.ups_test_result = test::RESULT_DONE_PASSED;
                break;
            case 2:
                data.test.ups_test_result = test::RESULT_DONE_WARNING;
                break;
            case 3:
                data.test.ups_test_result = test::RESULT_DONE_ERROR;
                break;
            case 4:
                data.test.ups_test_result = test::RESULT_ABORTED;
                break;
            case 5:
                data.test.ups_test_result = test::RESULT_IN_PROGRESS;
                break;
            case 6:
                data.test.ups_test_result = test::RESULT_NO_TEST;
                break;
            default:
                ESP_LOGD(TL_TAG, "Unknown test result value: %d", test_val);
                break;
        }

        if (!data.test.ups_test_result.empty()) {
            ESP_LOGD(TL_TAG, "Test result: %s (raw=%d)", data.test.ups_test_result.c_str(), test_val);
        }
    }
}


// ============================================================================
// Timer Polling (for real-time countdown updates)
// ============================================================================

bool TrippLiteProtocol::read_timer_data(UpsData &data) {
    parse_timer_data(data);
    return true;
}


// ============================================================================
// Beeper Control
// NUT mapping: UPS.PowerSummary.AudibleAlarmControl
// Values: 1=disabled, 2=enabled, 3=muted
// ============================================================================

bool TrippLiteProtocol::beeper_enable() {
    ESP_LOGI(TL_TAG, "Enabling beeper");
    uint8_t rid = use_descriptor_ ? find_report_id_for_usage(HID_USAGE_POW(HID_USAGE_POW_AUDIBLE_ALARM_CONTROL)) : 0;
    if (rid == 0) rid = HID_USAGE_POW_AUDIBLE_ALARM_CONTROL;  // Fallback to usage ID
    uint8_t data[2] = {rid, 2};  // 2 = enable
    return write_hid_feature_report(rid, data, 2);
}

bool TrippLiteProtocol::beeper_disable() {
    ESP_LOGI(TL_TAG, "Disabling beeper");
    uint8_t rid = use_descriptor_ ? find_report_id_for_usage(HID_USAGE_POW(HID_USAGE_POW_AUDIBLE_ALARM_CONTROL)) : 0;
    if (rid == 0) rid = HID_USAGE_POW_AUDIBLE_ALARM_CONTROL;
    uint8_t data[2] = {rid, 1};  // 1 = disable
    return write_hid_feature_report(rid, data, 2);
}

bool TrippLiteProtocol::beeper_mute() {
    ESP_LOGI(TL_TAG, "Muting beeper");
    uint8_t rid = use_descriptor_ ? find_report_id_for_usage(HID_USAGE_POW(HID_USAGE_POW_AUDIBLE_ALARM_CONTROL)) : 0;
    if (rid == 0) rid = HID_USAGE_POW_AUDIBLE_ALARM_CONTROL;
    uint8_t data[2] = {rid, 3};  // 3 = mute
    return write_hid_feature_report(rid, data, 2);
}

bool TrippLiteProtocol::beeper_test() {
    // Tripp Lite doesn't have a dedicated beeper test
    // Toggle beeper on briefly as a test
    ESP_LOGI(TL_TAG, "Beeper test (enable momentarily)");
    return beeper_enable();
}


// ============================================================================
// Battery Test Control
// NUT mapping: UPS.BatterySystem.Test
// Values: 1=quick test, 2=deep test, 3=abort
// ============================================================================

bool TrippLiteProtocol::start_battery_test_quick() {
    ESP_LOGI(TL_TAG, "Starting quick battery test");
    uint8_t rid = use_descriptor_ ? find_report_id_for_usage(HID_USAGE_POW(HID_USAGE_POW_TEST)) : 0;
    if (rid == 0) rid = HID_USAGE_POW_TEST;
    uint8_t data[2] = {rid, test::COMMAND_QUICK};
    return write_hid_feature_report(rid, data, 2);
}

bool TrippLiteProtocol::start_battery_test_deep() {
    ESP_LOGI(TL_TAG, "Starting deep battery test");
    uint8_t rid = use_descriptor_ ? find_report_id_for_usage(HID_USAGE_POW(HID_USAGE_POW_TEST)) : 0;
    if (rid == 0) rid = HID_USAGE_POW_TEST;
    uint8_t data[2] = {rid, test::COMMAND_DEEP};
    return write_hid_feature_report(rid, data, 2);
}

bool TrippLiteProtocol::stop_battery_test() {
    ESP_LOGI(TL_TAG, "Stopping battery test");
    uint8_t rid = use_descriptor_ ? find_report_id_for_usage(HID_USAGE_POW(HID_USAGE_POW_TEST)) : 0;
    if (rid == 0) rid = HID_USAGE_POW_TEST;
    uint8_t data[2] = {rid, test::COMMAND_ABORT};
    return write_hid_feature_report(rid, data, 2);
}

bool TrippLiteProtocol::start_ups_test() {
    // Tripp Lite uses the same Test usage for UPS self-test
    ESP_LOGI(TL_TAG, "Starting UPS self-test");
    return start_battery_test_quick();
}

bool TrippLiteProtocol::stop_ups_test() {
    ESP_LOGI(TL_TAG, "Stopping UPS self-test");
    return stop_battery_test();
}


// ============================================================================
// Delay Configuration
// ============================================================================

bool TrippLiteProtocol::set_shutdown_delay(int seconds) {
    ESP_LOGI(TL_TAG, "Setting shutdown delay to %d seconds", seconds);

    if (seconds < -1 || seconds > 7200) {
        ESP_LOGW(TL_TAG, "Shutdown delay %d out of range (-1 to 7200)", seconds);
        return false;
    }

    uint8_t rid = use_descriptor_ ? find_report_id_for_usage(HID_USAGE_POW(HID_USAGE_POW_DELAY_BEFORE_SHUTDOWN)) : 0;
    if (rid == 0) rid = HID_USAGE_POW_DELAY_BEFORE_SHUTDOWN;

    uint16_t value = (seconds < 0) ? 0xFFFF : static_cast<uint16_t>(seconds);
    uint8_t data[2] = {
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF)
    };

    return write_hid_feature_report(rid, data, 2);
}

bool TrippLiteProtocol::set_start_delay(int seconds) {
    ESP_LOGI(TL_TAG, "Setting start delay to %d seconds", seconds);

    if (seconds < 0 || seconds > 7200) {
        ESP_LOGW(TL_TAG, "Start delay %d out of range (0-7200)", seconds);
        return false;
    }

    uint8_t rid = use_descriptor_ ? find_report_id_for_usage(HID_USAGE_POW(HID_USAGE_POW_DELAY_BEFORE_STARTUP)) : 0;
    if (rid == 0) rid = HID_USAGE_POW_DELAY_BEFORE_STARTUP;

    uint8_t data[2] = {
        static_cast<uint8_t>(seconds & 0xFF),
        static_cast<uint8_t>((seconds >> 8) & 0xFF)
    };

    return write_hid_feature_report(rid, data, 2);
}

bool TrippLiteProtocol::set_reboot_delay(int seconds) {
    ESP_LOGI(TL_TAG, "Setting reboot delay to %d seconds", seconds);

    if (seconds < 0 || seconds > 7200) {
        ESP_LOGW(TL_TAG, "Reboot delay %d out of range (0-7200)", seconds);
        return false;
    }

    uint8_t rid = use_descriptor_ ? find_report_id_for_usage(HID_USAGE_POW(HID_USAGE_POW_DELAY_BEFORE_REBOOT)) : 0;
    if (rid == 0) rid = HID_USAGE_POW_DELAY_BEFORE_REBOOT;

    uint8_t data[2] = {
        static_cast<uint8_t>(seconds & 0xFF),
        static_cast<uint8_t>((seconds >> 8) & 0xFF)
    };

    return write_hid_feature_report(rid, data, 2);
}


}  // namespace ups_hid
}  // namespace esphome

// ============================================================================
// Protocol Factory Self-Registration
// ============================================================================
#include "protocol_factory.h"

namespace esphome {
namespace ups_hid {

// Creator function for Tripp Lite protocol
std::unique_ptr<UpsProtocolBase> create_tripplite_protocol(UpsHidComponent* parent) {
    return std::make_unique<TrippLiteProtocol>(parent);
}

} // namespace ups_hid
} // namespace esphome

// Register Tripp Lite protocol for vendor ID 0x09AE
REGISTER_UPS_PROTOCOL_FOR_VENDOR(
    0x09AE,
    tripplite_hid_protocol,
    esphome::ups_hid::create_tripplite_protocol,
    "Tripp Lite HID Protocol",
    "Tripp Lite USB HID UPS protocol with vendor-specific scaling and quirk handling",
    100
)
