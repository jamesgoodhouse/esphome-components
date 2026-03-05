#include "control_button.h"
#include "constants_ups.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ups_hid {

static const char *const BUTTON_TAG = "ups_hid.button";

void UpsHidButton::dump_config() {
  ESP_LOGCONFIG(BUTTON_TAG, "UPS HID Button:");
  if (button_type_ == BUTTON_TYPE_BEEPER) {
    ESP_LOGCONFIG(BUTTON_TAG, "  Beeper action: %s", beeper_action_.c_str());
  } else if (button_type_ == BUTTON_TYPE_TEST) {
    ESP_LOGCONFIG(BUTTON_TAG, "  Test action: %s", test_action_.c_str());
  } else if (button_type_ == BUTTON_TYPE_DEBUG) {
    ESP_LOGCONFIG(BUTTON_TAG, "  Debug action: %s", debug_action_.c_str());
  }
}

void UpsHidButton::press_action() {
  if (!parent_) {
    ESP_LOGE(BUTTON_TAG, log_messages::NO_PARENT_COMPONENT);
    return;
  }

  if (button_type_ == BUTTON_TYPE_DEBUG) {
    if (debug_action_ == "clear_event_log") {
      ESP_LOGI(BUTTON_TAG, "Clearing event log");
      parent_->get_event_log_mut().clear();
    } else {
      ESP_LOGE(BUTTON_TAG, "Unknown debug action: %s", debug_action_.c_str());
    }
    return;
  }

  if (!parent_->is_connected()) {
    ESP_LOGW(BUTTON_TAG, "UPS not connected");
    return;
  }

  if (button_type_ == BUTTON_TYPE_BEEPER) {
    ESP_LOGI(BUTTON_TAG, "Queuing beeper action: %s", beeper_action_.c_str());
    if (beeper_action_ == beeper::ACTION_ENABLE) parent_->beeper_enable();
    else if (beeper_action_ == beeper::ACTION_DISABLE) parent_->beeper_disable();
    else if (beeper_action_ == beeper::ACTION_MUTE) parent_->beeper_mute();
    else if (beeper_action_ == beeper::ACTION_TEST) parent_->beeper_test();
    else ESP_LOGE(BUTTON_TAG, "Unknown beeper action: %s", beeper_action_.c_str());
  } else if (button_type_ == BUTTON_TYPE_TEST) {
    ESP_LOGI(BUTTON_TAG, "Queuing test action: %s", test_action_.c_str());
    if (test_action_ == test::ACTION_BATTERY_QUICK) parent_->start_battery_test_quick();
    else if (test_action_ == test::ACTION_BATTERY_DEEP) parent_->start_battery_test_deep();
    else if (test_action_ == test::ACTION_BATTERY_STOP) parent_->stop_battery_test();
    else if (test_action_ == test::ACTION_UPS_TEST) parent_->start_ups_test();
    else if (test_action_ == test::ACTION_UPS_STOP) parent_->stop_ups_test();
    else ESP_LOGE(BUTTON_TAG, "Unknown test action: %s", test_action_.c_str());
  }
}

}  // namespace ups_hid
}  // namespace esphome
