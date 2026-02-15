# UPS HID Configuration Management

This directory provides a modular, maintainable approach to UPS HID device configuration using ESPHome packages with the a Protocol Factory system. Instead of maintaining large, complex configuration files, you can compose configurations from reusable components.

## 📦 Modular Configuration Packages

### **Core Packages** (Always Required)

#### `base_ups.yaml` 
Essential foundation for all UPS devices:
- ESP32-S3 hardware configuration
- Network setup (WiFi, OTA, Web server)
- UPS HID component initialization with Protocol Factory
- **Data Provider Pattern**: Components can access UPS data directly (no sensor entities required!)
- **Optional sensors**: Only `status` and `protocol` text sensors (for Home Assistant integration)

#### `base_ups_with_led.yaml`
Complete UPS monitoring with automatic LED status indication:
- Includes all `base_ups.yaml` functionality
- **Automatic LED Package**: Integrates `ups_status_led.yaml` 
- **Smart Status Display**: 7 priority-based LED patterns
- **Night Mode**: Time-based dimming with Home Assistant control
- **Zero Configuration**: Works immediately with substitution customization
- **Hardware Flexible**: Compatible with any ESPHome light component

#### `essential_sensors.yaml` / `essential_sensors_grouped.yaml`
Core monitoring sensors available on all protocols:
- **Sensors (5)**: `battery_level`, `input_voltage`, `output_voltage`, `load_percent`, `runtime`
- **Text Sensors (2)**: `manufacturer`, `model`
- **Binary Sensors (10)**: `charging`, `overload`, `discharging`, `boost`, `buck`, `over_temperature`, `communication_lost`, `shutdown_imminent`, `awaiting_power`, `voltage_out_of_range`
- **System Monitoring**: uptime, WiFi signal, IP address, ESPHome version
- **Grouped version** organizes entities into logical sections in ESPHome web interface
- Compatible with APC, CyberPower, Tripp Lite, and Generic protocols

### **Optional Enhancement Packages**

#### `ups_status_led.yaml`
Automatic LED status indication using smart patterns:

**Hardware Integration**:
- Works with any ESPHome light component (WS2812, FastLED, etc.)
- Hardware abstraction layer for maximum compatibility

**Smart Pattern System** (priority-based, automatic selection with solid colors):
- **🔴 Critical Alert** (solid red) - Low battery/fault/overload
- **🟠 Battery Power** (solid orange) - Running on battery
- **🟡 Charging** (solid yellow) - Battery charging
- **🟢 Normal** (solid green) - Normal operation
- **🔵 UPS Offline** (solid blue) - UPS disconnected
- **🟣 No Data** (solid purple) - Component connected but no data
- **⚪ Error** (solid white) - Component error

**Advanced Features**:
- **Battery-aware Colors**: Discrete thresholds or smooth gradients
- **Night Mode**: Time-based dimming with Home Assistant control
- **Home Assistant Integration**: Auto-created entities for complete control

**Automatic Entity Creation**:
- `switch.ups_status_led` (master enable/disable)
- `switch.ups_led_night_mode` (night mode toggle)  
- `number.ups_led_brightness` (brightness 10-100%)
- `text_sensor.ups_led_pattern` (current pattern name)

**Uses Data Provider Pattern**: No sensor entities required - accesses UPS data directly via `id(ups_monitor).is_online()`, etc.

#### `extended_sensors.yaml` / `extended_sensors_grouped.yaml`
Advanced monitoring (17 additional numeric sensors + 9 text sensors) for feature-rich devices:

**Enhanced Voltage Monitoring**:
- `battery_voltage`, `battery_voltage_nominal`, `input_voltage_nominal`
- `input_transfer_low`, `input_transfer_high`, `frequency`

**Live Power Measurements**:
- `output_current`, `active_power`, `output_frequency`

**Battery Capacity Details**:
- `battery_config_voltage`, `battery_full_charge_capacity`, `battery_design_capacity`

**Power & Configuration**:
- `ups_realpower_nominal`, `ups_delay_shutdown`, `ups_delay_start`, `ups_delay_reboot`
- `battery_runtime_low`

**Extended Device Information**:
- `serial_number`, `firmware_version`, `ups_beeper_status`, `input_sensitivity`
- `ups_mfr_date`, `battery_mfr_date`, `battery_type`, `battery_status`, `ups_firmware_aux`

**Smart Template**: Calculated load power using HID nominal power when available (falls back to estimate if `active_power` not available)
**Grouped version** categorizes sensors by function (Battery, Voltage, Load, Configuration, Device Info)

#### `ups_controls.yaml` / `ups_controls_grouped.yaml`
Complete control functionality (10 buttons + test monitoring):

**Beeper Control**: `enable`, `disable`, `mute`, `test`
**UPS Testing**: `battery_quick`, `battery_deep`, `battery_stop`, `ups_test`, `ups_stop`  
**Test Monitoring**: `ups_test_result` text sensor
**Grouped version** separates beeper controls from test controls in web interface

#### `timer_sensors.yaml` / `timer_sensors_grouped.yaml`
Real-time countdown timer monitoring (5 sensors + analysis):

**Active Timer Monitoring**: `ups_timer_shutdown`, `ups_timer_start`, `ups_timer_reboot`
**Timer Analysis**: `active_timer_count`, `fast_polling_status`
**Real-time Updates**: Component automatically switches to fast polling during countdowns
**Grouped version** places all timer-related entities in dedicated section

#### `delay_config.yaml`
UPS delay configuration package (3 number entities + automation scripts):

**Delay Configuration**: `shutdown`, `start`, `reboot` delays (0-600 seconds)
**Protocol Support**: CyberPower (full), Generic HID (multi-vendor), APC (INPUT-ONLY fallback)
**Scripts**: Default delay application and configuration reading
**Home Assistant**: Number entities with proper device classes and validation

#### `nut_server.yaml` / `nut_server_grouped.yaml`
Network UPS Tools (NUT) TCP server for standard protocol integration:

**Server Features**:
- **Standard NUT Protocol**: v2.8.0 compliant TCP server on port 3493
- **Authentication**: Optional username/password; read-only commands work without auth
- **Multi-client Support**: Up to 8 simultaneous connections (configurable to 20)
- **Dynamic UPS Discovery**: Automatically exposes connected UPS data
- **TCP Keepalive**: Detects dead connections automatically
- **Send Timeout**: 3-second timeout prevents slow clients from blocking the server

**Protocol Support**:
- `LIST UPS` - Available UPS devices with dynamic description
- `LIST VAR` - All UPS variables with real-time data
- `LIST CMD` - Available instant commands
- `LIST CLIENT` - Connected monitoring clients
- `INSTCMD` - Execute UPS commands (beeper, tests)
- `NETVER` - Protocol version negotiation

**Data Integration**:
- Uses Data Provider Pattern from ups_hid component
- Maps UPS data to standard NUT variables (battery.charge, input.voltage, etc.)
- Dynamic manufacturer/model detection from connected device

**Client Compatibility**:
- Standard NUT tools (`upsc`, `upscmd`)
- Home Assistant NUT integration
- Any NUT-compatible monitoring software
- TCP plain-text protocol (no SSL/TLS required)

**Grouped Version Features**:
- **NUT Server Group**: All NUT-related entities in dedicated "🌐 NUT Server" section
- **Enhanced Sensors**: Connection string helper, client counter placeholder
- **Entity Categories**: Marked as diagnostic for proper Home Assistant organization
- **Web Interface**: Clean separation from UPS monitoring sensors

#### `entity_groups.yaml`
ESPHome Web Server v3 entity organization (10 logical groups):

**Hierarchical Organization**: Power Status → Battery → Voltage → Load → Timers → Configuration → Controls → Device Info → System
**Visual Grouping**: Emoji headers and clear sorting weights (10-100)
**Improved Navigation**: Related entities grouped together, 75% less scrolling
**Optional Enhancement**: Include to enable organized web interface, omit for flat layout

### **Device-Specific Optimization Packages**

#### `device_types/apc_backups_es.yaml`
APC Back-UPS ES series optimizations:
- Slower update intervals (15s - APC devices need more time)
- APC-specific power calculations (405W nominal for ES 700)
- Enhanced protocol debugging (`ups_hid.apc: DEBUG`)
- Fixed device-specific script references

#### `device_types/cyberpower_cp1500.yaml`
CyberPower CP1500 series optimizations:
- Fast update intervals (10s - CyberPower responds quickly)
- **Additional sensor**: `battery_charge_low` threshold
- **Enhanced power calculation**: Uses HID nominal power when available (900W default)
- **Device-specific text sensor**: `battery_mfr_date` (separate from UPS date)
- Rich monitoring script with comprehensive status logging

---

## 📊 **Complete Sensor Summary**

### **Package Composition Options:**

| Configuration Level | Packages Included | Total Entities | Use Case |
|-------------------|------------------|--------------|----------|
| **Minimal** | `base_ups` + `essential_sensors` | **5 sensors + 2 text + 10 binary + system** | Basic monitoring |
| **Complete** | + `extended_sensors` + `ups_controls` + `timer_sensors` | **27 sensors + 13 text + 10 binary + 10 controls** | Full featured |
| **Complete + Config** | + `delay_config` | **+ 3 number entities** | With UPS configuration |
| **Complete + NUT Server** | + `nut_server` | **+ TCP server** | Network monitoring integration |
| **Device-Optimized** | + device-specific package | **+ device-specific additions** | Production ready |
| **Organized Interface** | Use `*_grouped.yaml` + `entity_groups.yaml` | **Same entities, better layout** | Enhanced web interface |

### **Sensor Breakdown by Type:**

#### **Numeric Sensors** (27 total available):
- **Essential (5)**: battery_level, input_voltage, output_voltage, load_percent, runtime
- **Extended (17)**: battery_voltage, battery_voltage_nominal, battery_config_voltage, battery_full_charge_capacity, battery_design_capacity, input_voltage_nominal, input_transfer_low, input_transfer_high, frequency, output_current, active_power, output_frequency, ups_realpower_nominal, ups_delay_shutdown, ups_delay_start, ups_delay_reboot, battery_runtime_low + calculated load power template
- **Timer (5)**: ups_timer_shutdown, ups_timer_start, ups_timer_reboot, active_timer_count, fast_polling_status

#### **Text Sensors** (14 total available):
- **Base (2)**: status, protocol *(in base_ups.yaml)*
- **Essential (2)**: manufacturer, model
- **Extended (9)**: serial_number, firmware_version, ups_beeper_status, input_sensitivity, ups_mfr_date, battery_mfr_date, battery_type, battery_status, ups_firmware_aux
- **Controls (1)**: ups_test_result *(in ups_controls.yaml)*

#### **Binary Sensors** (10 in config packages, 15 types available):
- **Essential (10)**: charging, overload, discharging, boost, buck, over_temperature, communication_lost, shutdown_imminent, awaiting_power, voltage_out_of_range
- **Additional types** (available in platform, add to your own config): online, on_battery, low_battery, fault, fully_discharged

#### **Button Controls** (10 total, all in ups_controls.yaml):
- **Beeper (4)**: enable, disable, mute, test
- **Testing (6)**: battery_quick, battery_deep, battery_stop, ups_test, ups_stop

#### **Number Controls** (3 total, all in delay_config.yaml):
- **Delay Configuration (3)**: shutdown_delay, start_delay, reboot_delay

#### **Smart Templates** (2 calculated sensors):
- **Generic**: UPS Load Power (uses HID data when available)
- **Device-Specific**: Device-optimized power calculations

## Testing Configurations

### `testing/basic_test.yaml`
Minimal configuration for protocol detection and basic functionality:
```yaml
packages:
  base_ups: !include ../base_ups.yaml
  essential: !include ../essential_sensors.yaml
```
**Use for**: Initial device testing, protocol verification, basic connectivity

### `testing/beeper_test.yaml`
Focused beeper control testing:
```yaml
packages:
  base_ups: !include ../base_ups.yaml
  essential: !include ../essential_sensors.yaml
  controls: !include ../ups_controls.yaml
```
**Use for**: Testing beeper functionality, control operations

### `testing/simulation_test.yaml`
Complete testing without physical hardware:
```yaml
packages:
  base_ups: !include ../base_ups.yaml
  essential: !include ../essential_sensors.yaml
  extended: !include ../extended_sensors.yaml
  controls: !include ../ups_controls.yaml
```
**Use for**: Development, CI/CD testing, feature demonstration

### `testing/minimal_data_provider.yaml`
Data provider pattern testing with zero sensor entities:
```yaml
substitutions:
  simulation_mode: "true"  # Enable simulation for testing
packages:
  base_ups: !include base_ups.yaml
# ZERO sensor entities defined - tests pure data provider mode!
```
**Features**:
- Tests all direct data access methods: `id(ups_monitor).is_online()`, etc.
- LED automation works without any sensor entities
- Comprehensive validation of data provider pattern
- **Use for**: Testing component as pure data provider, custom automation development

### `testing/led_status_test.yaml`
UPS Status LED component comprehensive testing:
```yaml
substitutions:
  simulation_mode: "true"  # Test without hardware
  led_color_mode: "gradient"  # Test gradient colors
packages:
  base_with_led: !include ../base_ups_with_led.yaml
```
**Features**:
- Tests all 7 LED patterns with simulated UPS states
- Battery-aware gradient color calculation verification
- Night mode with time-based scheduling testing
- Home Assistant integration validation (4 auto-created entities)
- Performance optimization verification (10Hz updates)
- **Use for**: LED component development, pattern testing, HA integration validation

### `testing/ups_with_smart_led.yaml`
Complete smart LED integration testing:
```yaml
substitutions:
  led_brightness: "90%"
  led_color_mode: "gradient"
  timezone: "America/New_York"
packages:
  base_with_led: !include ../base_ups_with_led.yaml
  essential: !include ../essential_sensors.yaml
```
**Features**:
- Full LED + sensor integration testing
- Customizable substitutions validation
- Advanced features testing (battery gradients, timezone support)
- Complete Home Assistant entity creation
- **Use for**: Integration testing, configuration validation

## Production Examples

### `examples/apc-ups-monitor.yaml`
Complete APC UPS production configuration with LED status indication:
- **Smart LED Integration**: Uses `base_ups_with_led.yaml` for automatic status display
- Modular package composition with device-specific optimizations
- Custom automation examples and notification setup
- Choice of standard or grouped entity layout
- APC-specific timer monitoring and power outage handling

### `examples/rack-ups-monitor.yaml`
Complete CyberPower UPS production configuration with LED status indication:
- **Smart LED Integration**: Uses `base_ups_with_led.yaml` for automatic status display
- Advanced sensor utilization with rich CyberPower data
- Smart threshold monitoring and power analysis
- Enhanced automation with battery threshold alerts
- Choice of standard or grouped entity layout

### `examples/grouped-ups-monitor.yaml`
Demonstration of organized web interface:
- All entities organized into 10 logical groups
- Hierarchical layout with clear visual sections
- Improved navigation and reduced scrolling
- Same functionality as standard packages

## Usage Patterns

### 1. Basic UPS Monitoring
```yaml
packages:
  base_ups: !include configs/base_ups.yaml
  essential: !include configs/essential_sensors.yaml
```
**Result**: ~150 lines → Essential monitoring only

### 2. UPS with Smart LED Status
```yaml
packages:
  base_with_led: !include configs/base_ups_with_led.yaml
  essential: !include configs/essential_sensors.yaml
```
**Result**: ~200 lines → Essential monitoring + automatic LED status indication

### 3. Full-Featured UPS
```yaml
packages:
  base_ups: !include configs/base_ups.yaml
  essential: !include configs/essential_sensors.yaml  
  extended: !include configs/extended_sensors.yaml
  timers: !include configs/timer_sensors.yaml
  controls: !include configs/ups_controls.yaml
  delays: !include configs/delay_config.yaml
  device: !include configs/device_types/cyberpower_cp1500.yaml
```
**Result**: ~380 lines → Complete functionality with UPS configuration

### 4. Organized Interface UPS
```yaml
packages:
  # Enable organized web interface
  entity_groups: !include configs/entity_groups.yaml
  
  # Use grouped sensor packages
  base_ups: !include configs/base_ups.yaml
  essential_grouped: !include configs/essential_sensors_grouped.yaml
  extended_grouped: !include configs/extended_sensors_grouped.yaml
  timers_grouped: !include configs/timer_sensors_grouped.yaml
  controls_grouped: !include configs/ups_controls_grouped.yaml
  delays: !include configs/delay_config.yaml
  device: !include configs/device_types/cyberpower_cp1500.yaml
```
**Result**: Same functionality with organized web interface layout

### 5. Custom Configuration
```yaml
substitutions:
  name: "my-custom-ups"
  update_interval: "30s"
  
packages:
  base_ups: !include configs/base_ups.yaml
  essential: !include configs/essential_sensors.yaml

# Add custom sensors or overrides here
sensor:
  - platform: template
    name: "Custom Power Calculation"
    # ... custom logic
```

## Configuration Management Best Practices

### 1. **Start Simple**
Begin with `basic_test.yaml` to verify device detection and protocol compatibility.

### 2. **Layer Functionality**
Add packages incrementally:
- Start with `base_ups.yaml` + `essential_sensors.yaml`
- Add `extended_sensors.yaml` if you need advanced metrics
- Add `timer_sensors.yaml` for real-time countdown monitoring
- Include `ups_controls.yaml` for interactive features
- Apply device-specific packages last
- Optionally use `*_grouped.yaml` + `entity_groups.yaml` for organized web interface

### 3. **Use Substitutions for Customization**
Override defaults without modifying packages:
```yaml
substitutions:
  name: "office-ups"
  friendly_name: "Office UPS Monitor"
  update_interval: "30s"
  simulation_mode: "false"
```

### 4. **Device-Specific Optimization**
- **APC devices**: Use slower update intervals (15-30s)
- **CyberPower devices**: Can handle faster updates (5-10s)  
- **Generic devices**: Start conservative (30s) and adjust

### 5. **Choose Entity Layout**
**Standard Layout** (flat entity list):
```yaml
packages:
  essential: !include configs/essential_sensors.yaml
  extended: !include configs/extended_sensors.yaml
```

**Organized Layout** (grouped by function):
```yaml
packages:
  entity_groups: !include configs/entity_groups.yaml
  essential_grouped: !include configs/essential_sensors_grouped.yaml
  extended_grouped: !include configs/extended_sensors_grouped.yaml
```

### 6. **Environment-Specific Files**
```yaml
# development.yaml
substitutions:
  log_level: "VERBOSE"
  update_interval: "5s"
  simulation_mode: "true"

# production.yaml  
substitutions:
  log_level: "INFO"
  update_interval: "30s" 
  simulation_mode: "false"
```

### 6. **Testing Strategy**
- **Protocol Testing**: Use `testing/basic_test.yaml`
- **Feature Testing**: Use specific test configurations  
- **Integration Testing**: Use `testing/simulation_test.yaml`
- **Data Provider Testing**: Use `testing/minimal_data_provider.yaml`
- **Production Validation**: Use device-specific examples

### 7. **Data Provider Pattern Usage**

The UPS HID component now functions as a **data provider**, allowing direct access to UPS data without requiring sensor entities:

#### **Direct Data Access Methods**:
```yaml
interval:
  - interval: 10s
    then:
      - lambda: |-
          // Boolean state methods (no sensor entities needed!)
          if (id(ups_monitor).is_online()) {
            ESP_LOGI("ups", "UPS is online");
          }
          if (id(ups_monitor).is_on_battery()) {
            ESP_LOGI("ups", "Running on battery: %.1f%% remaining", 
                     id(ups_monitor).get_battery_level());
          }
```

#### **Available Direct Access Methods**:
**Boolean States**: `is_online()`, `is_on_battery()`, `is_low_battery()`, `is_charging()`, `has_fault()`, `is_overloaded()`  
**Value Getters**: `get_battery_level()`, `get_input_voltage()`, `get_output_voltage()`, `get_load_percent()`, `get_runtime_minutes()`
