#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <cstdint>

namespace esphome {
namespace ups_hid {

static constexpr size_t MAX_EVENT_LOG_ENTRIES = 64;
static constexpr size_t NVS_PERSIST_ENTRIES = 16;

struct StateEvent {
  std::string timestamp;
  std::string message;
};

// Ring buffer of UPS state change events.
// Critical events are persisted to NVS flash so they survive reboots.
// Query via NUT `GET VAR <ups> ups.debug.event.N` after recovery.
class StateEventLog {
 public:
  void record(const std::string &timestamp, const std::string &message);

  // Returns all events oldest-first as a formatted string.
  std::string get_log() const;

  // Returns a single event by index (0 = oldest still in buffer).
  // Empty string if out of range.
  std::string get_event(size_t index) const;

  // Number of events currently stored.
  size_t size() const;

  // Clear all events from memory and NVS.
  void clear();

  // Load previously persisted events from NVS (call once at startup).
  void load_from_nvs();

  // Persist the most recent events to NVS flash.
  void save_to_nvs() const;

 private:
  mutable std::mutex mutex_;
  std::vector<StateEvent> buffer_;
  size_t head_{0};   // Next write position
  size_t count_{0};  // Number of stored events

};

// Snapshot of key UPS state fields for change detection.
struct StateSnapshot {
  bool online{false};
  bool on_battery{false};
  bool low_battery{false};
  bool charging{false};
  bool overloaded{false};
  bool fault{false};
  int battery_level_bucket{-1};  // Battery level in 5% buckets (-1 = unknown)
  bool valid{false};             // Has been populated at least once

  // Returns a human-readable status string (e.g. "OL", "OB LB").
  std::string status_string() const;
};

}  // namespace ups_hid
}  // namespace esphome
