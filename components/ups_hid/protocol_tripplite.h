#pragma once

#include "ups_hid.h"
#include "data_composite.h"
#include "data_device.h"
#include <set>
#include <map>

namespace esphome {
namespace ups_hid {

/**
 * Tripp Lite HID Protocol Implementation
 * 
 * Based on NUT tripplite-hid.c subdriver analysis and the USB HID Power Device
 * Class specification. Supports Tripp Lite UPS devices that use standard HID
 * Power Device class (vendor ID 0x09AE).
 * 
 * Key Tripp Lite quirks handled:
 * - Battery voltage scaling: 0.1 factor for product IDs in 0x2xxx range
 * - Some models use page 0x84 instead of 0x85 for charging/discharging status
 * - Timer values of 65535 (0xFFFF) mean "inactive/disabled"
 * - Beeper control: AudibleAlarmControl with 1=disable, 2=enable, 3=mute
 * - Some newer models (PID 0x3016, 0x3024) need additional voltage/frequency scaling
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

    // Scaling factors (determined by product ID, matching NUT tripplite-hid.c)
    double battery_scale_{0.1};        // Default for 0x2xxx PIDs (ECO/OMNI series)
    double io_voltage_scale_{1.0};     // Default: no I/O voltage scaling
    double io_frequency_scale_{1.0};   // Default: no frequency scaling
    double io_current_scale_{1.0};     // Default: no current scaling
    
    // Report discovery state
    std::set<uint8_t> available_input_reports_;
    std::set<uint8_t> available_feature_reports_;
    std::map<uint8_t, size_t> report_sizes_;
    bool device_info_read_{false};
    
    // HID communication methods
    bool read_hid_report(uint8_t report_id, HidReport &report);
    bool write_hid_feature_report(uint8_t report_id, const uint8_t* data, size_t len);
    
    // Report discovery
    void enumerate_reports();
    void determine_scaling_factors();
    
    // Device information reading
    void read_device_information(UpsData &data);
    
    // Parser methods for Tripp Lite report formats
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
    
    // Value extraction helpers
    float read_single_byte_value(const HidReport &report, uint8_t byte_index = 1);
    uint16_t read_16bit_le_value(const HidReport &report, uint8_t start_index = 1);
    float apply_battery_voltage_scale(float raw_value);
    float apply_io_voltage_scale(float raw_value);
    float apply_io_frequency_scale(float raw_value);
    
    // Timer sentinel value
    static constexpr uint16_t TIMER_INACTIVE = 0xFFFF;  // 65535 = inactive on Tripp Lite
};

// Factory creator function declaration
std::unique_ptr<UpsProtocolBase> create_tripplite_protocol(UpsHidComponent* parent);

} // namespace ups_hid  
} // namespace esphome
