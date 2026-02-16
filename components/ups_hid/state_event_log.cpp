#include "state_event_log.h"
#include <cstdio>
#include <cmath>

namespace esphome {
namespace ups_hid {

void StateEventLog::record(uint32_t uptime_ms, const std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (buffer_.size() < MAX_EVENT_LOG_ENTRIES) {
    buffer_.push_back({uptime_ms, message});
  } else {
    buffer_[head_] = {uptime_ms, message};
  }

  head_ = (head_ + 1) % MAX_EVENT_LOG_ENTRIES;
  if (count_ < MAX_EVENT_LOG_ENTRIES) {
    count_++;
  }
}

std::string StateEventLog::get_log() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::string result;
  if (count_ == 0) return "(no events)\n";

  size_t start = (count_ < MAX_EVENT_LOG_ENTRIES) ? 0 : head_;
  for (size_t i = 0; i < count_; i++) {
    size_t idx = (start + i) % MAX_EVENT_LOG_ENTRIES;
    result += format_uptime(buffer_[idx].uptime_ms);
    result += " ";
    result += buffer_[idx].message;
    result += "\n";
  }
  return result;
}

std::string StateEventLog::get_event(size_t index) const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (index >= count_) return "";

  size_t start = (count_ < MAX_EVENT_LOG_ENTRIES) ? 0 : head_;
  size_t idx = (start + index) % MAX_EVENT_LOG_ENTRIES;

  return format_uptime(buffer_[idx].uptime_ms) + " " + buffer_[idx].message;
}

size_t StateEventLog::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return count_;
}

std::string StateEventLog::format_uptime(uint32_t ms) {
  uint32_t total_s = ms / 1000;
  uint32_t h = total_s / 3600;
  uint32_t m = (total_s % 3600) / 60;
  uint32_t s = total_s % 60;

  char buf[16];
  snprintf(buf, sizeof(buf), "+%02u:%02u:%02u", h, m, s);
  return std::string(buf);
}

std::string StateSnapshot::status_string() const {
  std::string s;
  if (online && !on_battery) {
    s = "OL";
    if (charging) s += " CHRG";
  } else if (on_battery) {
    s = "OB";
    if (low_battery) s += " LB";
  } else {
    s = "OFF";
  }
  if (overloaded) s += " OVER";
  if (fault) s += " ALARM";
  return s;
}

}  // namespace ups_hid
}  // namespace esphome
