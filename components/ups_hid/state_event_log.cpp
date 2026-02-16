#include "state_event_log.h"
#include <cstdio>

namespace esphome {
namespace ups_hid {

void StateEventLog::record(const std::string &timestamp, const std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (buffer_.size() < MAX_EVENT_LOG_ENTRIES) {
    buffer_.push_back({timestamp, message});
  } else {
    buffer_[head_] = {timestamp, message};
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
    result += buffer_[idx].timestamp;
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

  return buffer_[idx].timestamp + " " + buffer_[idx].message;
}

size_t StateEventLog::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return count_;
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
