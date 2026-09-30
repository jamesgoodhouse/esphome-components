#include "protocol_tripplite.h"
#include "ups_hid.h"
#include "constants_hid.h"
#include "constants_ups.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include <set>
#include <map>
#include <algorithm>
#include <cinttypes>
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

    std::string last_err = parent_->get_transport_error();
    ESP_LOGW(TL_TAG, "Failed to read any HID reports from Tripp Lite device%s%s",
             last_err.empty() ? "" : " (last transport error: ",
             last_err.empty() ? "" : (last_err + ")").c_str());
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
    report_fail_count_.clear();
    dynamic_reports_.clear();
    last_full_read_ms_ = 0;
    force_full_read_ = true;
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
        classify_reports_from_descriptor();
    } else {
        ESP_LOGW(TL_TAG, "No HID report descriptor - falling back to heuristic discovery");
        enumerate_reports();
    }

    if (available_input_reports_.empty() && available_feature_reports_.empty()) {
        ESP_LOGE(TL_TAG, "No HID reports found during initialization");
        return false;
    }

    ESP_LOGI(TL_TAG, "Tripp Lite HID initialized (%s mode): %zu input reports, %zu feature reports"
             " (%zu polled every cycle, rest every %" PRIu32 "s)",
             use_descriptor_ ? "descriptor" : "heuristic",
             available_input_reports_.size(), available_feature_reports_.size(),
             use_descriptor_ ? dynamic_reports_.size() : available_feature_reports_.size(),
             FULL_REFRESH_INTERVAL_MS / 1000);

    for (uint8_t id : available_feature_reports_) {
        ESP_LOGD(TL_TAG, "  Feature report 0x%02X: %zu bytes%s", id, report_sizes_[id],
                 dynamic_reports_.count(id) ? " [every cycle]" : "");
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

// Usages whose values change at runtime (measurements, status flags, timers).
// Reports without any of these hold identity/configuration data and are only
// re-read on a full refresh, mirroring NUT's quick-poll vs full-update split.
static bool is_dynamic_usage(uint32_t usage) {
    uint16_t page = (usage >> 16) & 0xFFFF;
    uint16_t id = usage & 0xFFFF;
    if (page == HID_USAGE_PAGE_POWER_DEVICE) {
        if (id >= HID_USAGE_POW_VOLTAGE && id <= HID_USAGE_POW_TEMPERATURE) return true;       // measurements
        if (id >= HID_USAGE_POW_DELAY_BEFORE_REBOOT && id <= HID_USAGE_POW_TEST) return true;  // timers, test
        if (id >= HID_USAGE_POW_PRESENT && id <= HID_USAGE_POW_COMMUNICATION_LOST) return true; // status bits
        // Tripp Lite puts some Battery System status bits on this page
        return id == HID_USAGE_TL_CHARGING || id == HID_USAGE_TL_DISCHARGING ||
               id == HID_USAGE_TL_NEED_REPLACEMENT || id == HID_USAGE_TL_AC_PRESENT;
    }
    if (page == HID_USAGE_PAGE_BATTERY_SYSTEM) {
        if (id >= HID_USAGE_BAT_BELOW_REMAINING_CAPACITY_LIMIT && id <= HID_USAGE_BAT_FULLY_DISCHARGED) return true;
        return id == HID_USAGE_BAT_NEED_REPLACEMENT ||
               id == HID_USAGE_BAT_REMAINING_CAPACITY ||
               id == HID_USAGE_BAT_RUN_TIME_TO_EMPTY ||
               id == HID_USAGE_BAT_AVERAGE_TIME_TO_EMPTY ||
               id == HID_USAGE_BAT_AVERAGE_TIME_TO_FULL ||
               id == HID_USAGE_BAT_AC_PRESENT ||
               id == HID_USAGE_BAT_BATTERY_PRESENT;
    }
    return false;
}

void TrippLiteProtocol::classify_reports_from_descriptor() {
    const HidReportMap* map = parent_->get_report_map();
    if (!map) return;

    dynamic_reports_.clear();
    for (const auto& f : map->get_all_fields()) {
        if (f.report_type != HID_REPORT_TYPE_FEATURE) continue;
        if (!available_feature_reports_.count(f.report_id)) continue;
        if (is_dynamic_usage(f.usage)) {
            dynamic_reports_.insert(f.report_id);
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
                                                 parent_->get_report_timeout());
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
                                       parent_->get_report_timeout());
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
                                                 parent_->get_report_timeout());
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
                                       parent_->get_report_timeout());
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
                                       parent_->get_report_timeout());
        if (ret == ESP_OK && buffer_len > 0) {
            report.report_id = report_id;
            report.data.assign(buffer, buffer + buffer_len);
            ESP_LOGV(TL_TAG, "Read Feature report 0x%02X: %zu bytes", report_id, buffer_len);
            return true;
        }
    }

    // Fallback to Input report (skip when the device just went away; a stalled
    // transfer triggers a recovery and every further attempt would only wait)
    if (!parent_->is_connected()) {
        return false;
    }
    if (available_input_reports_.empty() || available_input_reports_.count(report_id)) {
        buffer_len = sizeof(buffer);
        ret = parent_->hid_get_report(HID_REPORT_TYPE_INPUT, report_id,
                                       buffer, &buffer_len,
                                       parent_->get_report_timeout());
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
                                             data, len, parent_->get_report_timeout());
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

float TrippLiteProtocol::apply_battery_voltage_scale(float raw_value) {
    return static_cast<float>(battery_scale_ * raw_value);
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
        ESP_LOGV(TL_TAG, "%s = %.2f (report 0x%02X, bits %u@%u, %s)",
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
        ESP_LOGV(TL_TAG, "%s = %.2f (report 0x%02X, collection 0x%08lX, %s)",
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

    // Full refresh: every feature report. Quick cycle: only the reports that
    // carry measurements/status; values not read this cycle stay NaN and the
    // component keeps the previous ones.
    uint32_t now = millis();
    bool full = force_full_read_ || last_full_read_ms_ == 0 || dynamic_reports_.empty() ||
                (now - last_full_read_ms_) >= FULL_REFRESH_INTERVAL_MS;
    const std::set<uint8_t>& to_read = full ? available_feature_reports_ : dynamic_reports_;

    ESP_LOGV(TL_TAG, "Reading Tripp Lite HID data (descriptor mode, %s, %zu reports)...",
             full ? "full" : "quick", to_read.size());

    // Each report is a separate GET_REPORT; individual reports may fail while
    // others succeed. Abort the cycle after several consecutive failures so an
    // unresponsive device does not hold the read task for long.
    static constexpr int MAX_CONSECUTIVE_FAILURES = 5;
    std::map<uint8_t, std::vector<uint8_t>> report_cache;
    int reports_read = 0;
    int consecutive_failures = 0;
    bool aborted = false;
    std::vector<uint8_t> failed;

    for (uint8_t rid : to_read) {
        if (!available_feature_reports_.count(rid)) continue;
        HidReport report;
        if (read_hid_report(rid, report) && !report.data.empty()) {
            report_cache[rid] = std::move(report.data);
            reports_read++;
            report_fail_count_[rid] = 0;
            consecutive_failures = 0;
        } else {
            failed.push_back(rid);
            if (++consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
                ESP_LOGW(TL_TAG, "Aborting read cycle: %d consecutive report failures (%d read so far)",
                         consecutive_failures, reports_read);
                aborted = true;
                break;
            }
        }
    }

    if (reports_read == 0) {
        ESP_LOGW(TL_TAG, "All %zu report reads failed", failed.size());
        return false;
    }

    // Exclude reports that keep failing while the rest of the device answers.
    // A cycle that had to be aborted means the device stopped responding, not
    // that those particular reports are bad, so it is not counted.
    if (aborted) failed.clear();
    for (uint8_t rid : failed) {
        uint8_t &count = report_fail_count_[rid];
        if (count < 255) count++;
        if (count == REPORT_FAIL_THRESHOLD) {
            ESP_LOGW(TL_TAG, "Report 0x%02X failed %u consecutive reads, excluding (%zu bytes expected)",
                     rid, REPORT_FAIL_THRESHOLD,
                     report_sizes_.count(rid) ? report_sizes_[rid] : 0);
            for (const auto *f : map->get_fields_for_report(rid, HID_REPORT_TYPE_FEATURE)) {
                uint16_t page = (f->usage >> 16) & 0xFFFF;
                ESP_LOGW(TL_TAG, "  -> usage 0x%04X:0x%04X (%s), %u bits @ offset %u",
                         page, f->usage & 0xFFFF,
                         page == 0x0084 ? "Power Device" :
                         page == 0x0085 ? "Battery System" :
                         page == 0xFFFF ? "Vendor-specific" : "other",
                         f->bit_size, f->bit_offset);
            }
            available_feature_reports_.erase(rid);
            dynamic_reports_.erase(rid);
        }
    }

    if (!failed.empty()) {
        ESP_LOGD(TL_TAG, "Read %d/%zu reports (%zu failed)", reports_read, to_read.size(), failed.size());
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
    if (!std::isnan(ac_present)) {
        data.power.ac_present = ac_present > 0 ? 1 : 0;
    }

    if (!std::isnan(discharging) && discharging > 0) {
        data.battery.status = battery_status::DISCHARGING;
    } else if (!std::isnan(fully_charged) && fully_charged > 0) {
        data.battery.status = battery_status::FULLY_CHARGED;
    } else if (!std::isnan(charging) && charging > 0) {
        data.battery.status = battery_status::CHARGING;
    } else if (!std::isnan(charging) || !std::isnan(discharging)) {
        data.battery.status = battery_status::NOT_CHARGING;
    }

    // Fully discharged flag
    float fully_discharged = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_FULLY_DISCHARGED), "battery.fully_discharged");

    // Low battery as judged by the UPS itself (NUT: lowbatt_info -> "LB")
    float below_limit = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_BELOW_REMAINING_CAPACITY_LIMIT), "battery.below_capacity_limit");
    float time_limit_expired = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_REMAINING_TIME_LIMIT_EXPIRED), "battery.time_limit_expired");

    // Need replacement flag (standard on 0x85, TL also puts on 0x84)
    float need_replace = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_NEED_REPLACEMENT), "battery.need_replacement");
    if (std::isnan(need_replace)) {
        need_replace = read_usage_value(map, report_cache,
            HID_USAGE_POW(HID_USAGE_TL_NEED_REPLACEMENT), "battery.need_replacement.p84");
    }

    // The battery flags share the PresentStatus report; if any of them was read
    // this cycle the others are authoritative too.
    if (!std::isnan(charging) || !std::isnan(discharging) || !std::isnan(below_limit) ||
        !std::isnan(need_replace)) {
        data.battery.flags_valid = true;
        data.battery.low_battery = (!std::isnan(below_limit) && below_limit > 0) ||
                                   (!std::isnan(time_limit_expired) && time_limit_expired > 0);
        data.battery.needs_replacement = !std::isnan(need_replace) && need_replace > 0;
    }

    // UPS-configured low battery thresholds (NUT battery.charge.low / battery.runtime.low)
    float charge_low = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_REMAINING_CAPACITY_LIMIT), "battery.charge.low");
    if (!std::isnan(charge_low) && charge_low >= 0 && charge_low <= 100) {
        data.battery.charge_low = charge_low;
    }
    float runtime_low_sec = read_usage_value(map, report_cache,
        HID_USAGE_BAT(HID_USAGE_BAT_REMAINING_TIME_LIMIT), "battery.runtime.low");
    if (!std::isnan(runtime_low_sec) && runtime_low_sec > 0) {
        data.battery.runtime_low = runtime_low_sec / 60.0f;
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
    // Apply descriptor-specific voltage scaling for devices with bad exponents.
    // 0 V while on battery is a real reading (NUT reports input.voltage: 0.0).
    if (descriptor_needs_raw_extraction_ && !std::isnan(data.power.input_voltage)) {
        data.power.input_voltage *= descriptor_voltage_scale_;
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

    // Active power nominal (W); only reported when the device provides it
    data.power.realpower_nominal = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_CONFIG_ACTIVE_POWER), "ups.realpower.nominal");

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
    float boost_val = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_BOOST), "ups.status.boost");
    float buck_val = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_BUCK), "ups.status.buck");
    float overtemp_val = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_OVER_TEMPERATURE), "ups.status.overtemp");
    float commlost_val = read_usage_value(map, report_cache,
        HID_USAGE_POW(HID_USAGE_POW_COMMUNICATION_LOST), "ups.status.commlost");

    data.power.overload = (!std::isnan(overload) && overload > 0);
    data.power.internal_failure = (!std::isnan(internal_failure) && internal_failure > 0);
    data.power.boost_active = (!std::isnan(boost_val) && boost_val > 0);
    data.power.buck_active = (!std::isnan(buck_val) && buck_val > 0);
    data.power.over_temperature = (!std::isnan(overtemp_val) && overtemp_val > 0);
    data.power.communication_lost = (!std::isnan(commlost_val) && commlost_val > 0);
    data.power.shutdown_imminent = (!std::isnan(shutdown_imminent_val) && shutdown_imminent_val > 0);
    data.power.awaiting_power = (!std::isnan(awaiting_power) && awaiting_power > 0);
    data.power.voltage_out_of_range = (!std::isnan(voltage_oor) && voltage_oor > 0);

    // Power state, same rules as NUT's ups_status_set(): ACPresent decides
    // online/on-battery; without it, Discharging means on battery. Fall back to
    // the measured input voltage only when the device reports neither. Leave
    // the status empty when undetermined so the previous value is kept.
    if (data.power.ac_present == 1) {
        data.power.status = status::ONLINE;
    } else if (data.power.ac_present == 0) {
        data.power.status = status::ON_BATTERY;
    } else if (!std::isnan(discharging) && discharging > 0) {
        data.power.status = status::ON_BATTERY;
    } else if (data.power.input_voltage_valid()) {
        data.power.status = status::ONLINE;
    } else if (report_cache.size() == to_read.size()) {
        ESP_LOGW(TL_TAG, "Status undetermined: no ACPresent/Discharging flag and input voltage %s",
                 std::isnan(data.power.input_voltage) ? "unknown"
                                                     : std::to_string(data.power.input_voltage).c_str());
    }

    if (!std::isnan(fully_discharged) && fully_discharged > 0) {
        data.battery.status = "Depleted";
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
                   !std::isnan(data.power.load_percent) ||
                   !data.power.status.empty();

    if (!success) {
        if (full) {
            ESP_LOGW(TL_TAG, "Descriptor-based reading produced no usable data, falling back to heuristic");
            use_descriptor_ = false;
            return read_data_heuristic(data);
        }
        ESP_LOGW(TL_TAG, "Quick read produced no usable data");
        return false;
    }

    auto fmt_val = [](float v, const char *fmt, char *buf, size_t n) -> const char * {
        if (std::isnan(v)) return "?";
        snprintf(buf, n, fmt, v);
        return buf;
    };
    char b_bat[8], b_in[8], b_out[8], b_load[8], b_freq[8], b_cur[8], b_pwr[8];
    char summary[160];
    snprintf(summary, sizeof(summary),
             "Data read OK (%s): %s, bat=%s%%, in=%sV, out=%sV, load=%s%%, freq=%sHz, cur=%sA, pwr=%sW",
             full ? "full" : "quick",
             data.power.status.empty() ? "status unchanged" : data.power.status.c_str(),
             fmt_val(data.battery.level, "%.0f", b_bat, sizeof(b_bat)),
             fmt_val(data.power.input_voltage, "%.0f", b_in, sizeof(b_in)),
             fmt_val(data.power.output_voltage, "%.0f", b_out, sizeof(b_out)),
             fmt_val(data.power.load_percent, "%.0f", b_load, sizeof(b_load)),
             fmt_val(data.power.frequency, "%.0f", b_freq, sizeof(b_freq)),
             fmt_val(data.power.output_current, "%.1f", b_cur, sizeof(b_cur)),
             fmt_val(data.power.active_power, "%.0f", b_pwr, sizeof(b_pwr)));

    if (full) {
        ESP_LOGI(TL_TAG, "%s", summary);
        // Descriptor coverage and the values of fields we do not use, for
        // reverse-engineering vendor reports.
        map->log_field_summary(TL_TAG, queried_usages_);
        map->log_unused_field_values(TL_TAG, queried_usages_, report_cache);
        last_full_read_ms_ = now;
        force_full_read_ = false;
    } else {
        ESP_LOGD(TL_TAG, "%s", summary);
    }

    return true;
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

    static constexpr int MAX_CONSECUTIVE_TIMEOUTS = 5;
    std::map<uint8_t, HidReport> all_reports;
    int reports_read = 0;
    int consecutive_timeouts = 0;

    for (uint8_t rid : available_feature_reports_) {
        HidReport report;
        if (read_hid_report(rid, report) && !report.data.empty()) {
            all_reports[rid] = report;
            reports_read++;
            consecutive_timeouts = 0;
        } else {
            consecutive_timeouts++;
            if (consecutive_timeouts >= MAX_CONSECUTIVE_TIMEOUTS) {
                ESP_LOGW(TL_TAG, "Aborting heuristic read: %d consecutive failures "
                         "(%d read so far)", consecutive_timeouts, reports_read);
                break;
            }
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
// Timer Polling
// ============================================================================

// Timers are read as part of the regular cycle (descriptor mode), so there is
// nothing extra to poll here; the component derives fast polling from the data.
bool TrippLiteProtocol::read_timer_data(UpsData &data) {
    return false;
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
