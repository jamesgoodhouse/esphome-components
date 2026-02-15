#pragma once

#include "ups_hid.h"
#include "data_composite.h"
#include "data_device.h"
#include "hid_report_descriptor.h"
#include <set>
#include <map>

namespace esphome {
namespace ups_hid {

/**
 * Tripp Lite HID Protocol Implementation
 *
 * Supports two data reading strategies:
 * 1. **Descriptor-based** (preferred): Parses the device's HID report descriptor
 *    to know exactly which report ID contains which data field. Handles unit
 *    exponents and physical conversion automatically.
 * 2. **Heuristic** (fallback): Reads all reports and classifies values by range.
 *    Used when the report descriptor cannot be fetched or parsed.
 *
 * Key Tripp Lite quirks handled:
 * - Battery voltage scaling: 0.1 factor for product IDs in 0x2xxx range (heuristic mode only)
 * - Timer values of 65535 (0xFFFF) mean "inactive/disabled"
 * - Beeper control: AudibleAlarmControl with 1=disable, 2=enable, 3=mute
 *
 * Reference: https://github.com/networkupstools/nut/blob/master/drivers/tripplite-hid.c
 */
class TrippLiteProtocol : public UpsProtocolBase {
public:
    explicit TrippLiteProtocol(UpsHidComponent* parent) : UpsProtocolBase(parent) {}
    ~TrippLiteProtocol() override = default;

    // Protocol identification
    DeviceInfo::DetectedProtocol get_protocol_type() const override {
        return DeviceInfo::PROTOCOL_TRIPPLITE_HID;
    }
    std::string get_protocol_name() const override {
        return "Tripp Lite HID";
    }

    // Core protocol interface
    bool detect() override;
    bool initialize() override;
    bool read_data(UpsData &data) override;

    // Timer polling for real-time countdown
    bool read_timer_data(UpsData &data) override;

    // Beeper control methods (AudibleAlarmControl standard)
    bool beeper_enable() override;
    bool beeper_disable() override;
    bool beeper_mute() override;
    bool beeper_test() override;

    // Battery test methods
    bool start_battery_test_quick() override;
    bool start_battery_test_deep() override;
    bool stop_battery_test() override;
    bool start_ups_test() override;
    bool stop_ups_test() override;

    // Delay configuration methods
    bool set_shutdown_delay(int seconds) override;
    bool set_start_delay(int seconds) override;
    bool set_reboot_delay(int seconds) override;

private:
    // HID Report structure (matches pattern from other protocols)
    struct HidReport {
        uint8_t report_id;
        std::vector<uint8_t> data;

        HidReport() : report_id(0) {}
    };

    // === Strategy selection ===
    bool use_descriptor_{false};  // true = descriptor-based, false = heuristic

    // When true, descriptor mode uses raw logical value extraction (skipping
    // the descriptor's unit exponent and physical conversion) because the
    // device's descriptor has incorrect exponents. Device-specific scaling
    // (descriptor_voltage_scale_ / descriptor_frequency_scale_) is applied instead.
    bool descriptor_needs_raw_extraction_{false};
    double descriptor_voltage_scale_{1.0};   // Applied to measured voltages in descriptor mode
    double descriptor_frequency_scale_{1.0}; // Applied to measured frequencies in descriptor mode

    // === Heuristic mode state ===
    // Scaling factors (determined by product ID, matching NUT tripplite-hid.c)
    double battery_scale_{0.1};
    double io_voltage_scale_{1.0};
    double io_frequency_scale_{1.0};
    double io_current_scale_{1.0};

    // Report discovery state (used by both modes)
    std::set<uint8_t> available_input_reports_;
    std::set<uint8_t> available_feature_reports_;
    std::map<uint8_t, size_t> report_sizes_;
    bool device_info_read_{false};

    // === HID communication ===
    bool read_hid_report(uint8_t report_id, HidReport &report);
    bool write_hid_feature_report(uint8_t report_id, const uint8_t* data, size_t len);

    // === Initialization helpers ===
    void enumerate_reports();              // Brute-force enumeration (heuristic mode)
    void enumerate_reports_from_descriptor(); // Descriptor-based enumeration
    void determine_scaling_factors();      // Heuristic mode only

    // === Device information ===
    void read_device_information(UpsData &data);

    // === Descriptor-based data reading (preferred) ===
    bool read_data_descriptor(UpsData &data);

    // Helper: extract a single usage value from cached report data
    float read_usage_value(const HidReportMap* map,
                          const std::map<uint8_t, std::vector<uint8_t>>& cache,
                          uint32_t usage, const char* name);

    // Helper: extract a usage value scoped to a specific collection
    float read_usage_in_collection(const HidReportMap* map,
                                   const std::map<uint8_t, std::vector<uint8_t>>& cache,
                                   uint32_t usage, uint32_t collection_usage,
                                   const char* name);

    // Helper: find the report ID for a given usage (for control commands)
    uint8_t find_report_id_for_usage(uint32_t usage) const;

    // === Heuristic data reading (fallback) ===
    bool read_data_heuristic(UpsData &data);

    // Heuristic parser methods
    void parse_battery_data(UpsData &data);
    void parse_power_summary(UpsData &data);
    void parse_status_flags(UpsData &data);
    void parse_input_data(UpsData &data);
    void parse_output_data(UpsData &data);
    void parse_load_data(UpsData &data);
    void parse_beeper_status(UpsData &data);
    void parse_delay_configuration(UpsData &data);
    void parse_timer_data(UpsData &data);
    void parse_test_result(UpsData &data);
    void parse_power_nominal(UpsData &data);
    void parse_frequency_data(UpsData &data);
    void parse_transfer_limits(UpsData &data);

    // Value extraction helpers (heuristic mode)
    float read_single_byte_value(const HidReport &report, uint8_t byte_index = 1);
    uint16_t read_16bit_le_value(const HidReport &report, uint8_t start_index = 1);
    float apply_battery_voltage_scale(float raw_value);
    float apply_io_voltage_scale(float raw_value);
    float apply_io_frequency_scale(float raw_value);

    // Timer sentinel value
    static constexpr uint16_t TIMER_INACTIVE = 0xFFFF;
};

// Factory creator function declaration
std::unique_ptr<UpsProtocolBase> create_tripplite_protocol(UpsHidComponent* parent);

} // namespace ups_hid
} // namespace esphome
