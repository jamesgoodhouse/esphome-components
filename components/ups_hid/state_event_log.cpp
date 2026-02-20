#include "state_event_log.h"
#include "esphome/core/log.h"
#include <cstdio>
#include <nvs_flash.h>
#include <nvs.h>

namespace esphome {
namespace ups_hid {

static const char *const NVS_TAG = "ups_hid.nvs";
static const char *const NVS_NAMESPACE = "ups_evtlog";
static const char *const NVS_KEY_COUNT = "count";

void StateEventLog::record(const std::string &timestamp, const std::string &message) {
  {
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
  // Auto-persist on status changes (not battery level ticks)
  if (message.find("Status:") != std::string::npos ||
      message.find("Initial state:") != std::string::npos ||
      message.find("Boot:") != std::string::npos) {
    save_to_nvs();
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

void StateEventLog::load_from_nvs() {
  std::lock_guard<std::mutex> lock(mutex_);

  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    ESP_LOGD(NVS_TAG, "No saved event log found (nvs_open: %s)", esp_err_to_name(err));
    return;
  }

  uint8_t saved_count = 0;
  err = nvs_get_u8(handle, NVS_KEY_COUNT, &saved_count);
  if (err != ESP_OK || saved_count == 0) {
    nvs_close(handle);
    return;
  }

  if (saved_count > NVS_PERSIST_ENTRIES) saved_count = NVS_PERSIST_ENTRIES;

  size_t loaded = 0;
  for (uint8_t i = 0; i < saved_count; i++) {
    char ts_key[12], msg_key[12];
    snprintf(ts_key, sizeof(ts_key), "ts_%u", i);
    snprintf(msg_key, sizeof(msg_key), "msg_%u", i);

    size_t ts_len = 0, msg_len = 0;
    if (nvs_get_str(handle, ts_key, nullptr, &ts_len) != ESP_OK) continue;
    if (nvs_get_str(handle, msg_key, nullptr, &msg_len) != ESP_OK) continue;

    std::string ts(ts_len - 1, '\0');
    std::string msg(msg_len - 1, '\0');
    if (nvs_get_str(handle, ts_key, &ts[0], &ts_len) != ESP_OK) continue;
    if (nvs_get_str(handle, msg_key, &msg[0], &msg_len) != ESP_OK) continue;

    buffer_.push_back({ts, msg});
    count_++;
    head_ = count_ % MAX_EVENT_LOG_ENTRIES;
    loaded++;
  }

  nvs_close(handle);

  if (loaded > 0) {
    ESP_LOGI(NVS_TAG, "Restored %zu events from previous boot", loaded);
  }
}

void StateEventLog::save_to_nvs() const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (count_ == 0) return;

  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGW(NVS_TAG, "Failed to open NVS for writing: %s", esp_err_to_name(err));
    return;
  }

  // Save the most recent N entries
  size_t to_save = std::min(count_, NVS_PERSIST_ENTRIES);
  size_t start_offset = (count_ > to_save) ? count_ - to_save : 0;
  size_t ring_start = (count_ < MAX_EVENT_LOG_ENTRIES) ? 0 : head_;

  nvs_set_u8(handle, NVS_KEY_COUNT, static_cast<uint8_t>(to_save));

  for (size_t i = 0; i < to_save; i++) {
    size_t idx = (ring_start + start_offset + i) % MAX_EVENT_LOG_ENTRIES;
    char ts_key[12], msg_key[12];
    snprintf(ts_key, sizeof(ts_key), "ts_%zu", i);
    snprintf(msg_key, sizeof(msg_key), "msg_%zu", i);

    nvs_set_str(handle, ts_key, buffer_[idx].timestamp.c_str());
    nvs_set_str(handle, msg_key, buffer_[idx].message.c_str());
  }

  nvs_commit(handle);
  nvs_close(handle);

  ESP_LOGD(NVS_TAG, "Persisted %zu events to NVS", to_save);
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
