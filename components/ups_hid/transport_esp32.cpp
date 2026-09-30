#include "transport_esp32.h"
#include "constants_ups.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esp_idf_version.h"
#include <cinttypes>
#include <cstring>
#include <cstdio>
#include <cctype>
#include <algorithm>

#ifdef USE_ESP32

// usb_host_lib_set_root_port_power() exists in ESP-IDF >= 5.4 and was
// backported to 5.2.5+ (but not 5.3.x). Can be overridden with a build flag
// for IDF forks that carry the backport.
#ifndef UPS_HID_HAVE_ROOT_PORT_POWER
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0) || \
    (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 5) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 0))
#define UPS_HID_HAVE_ROOT_PORT_POWER 1
#else
#define UPS_HID_HAVE_ROOT_PORT_POWER 0
#endif
#endif

namespace esphome {
namespace ups_hid {

static const char *const ESP32_USB_TAG = "ups_hid.esp32_usb";

#ifndef USB_CLASS_HID
#define USB_CLASS_HID 0x03
#endif

// Minimum time the root port stays unpowered during a recovery so the device
// sees a real disconnect, and the longest we wait for the stack to close the
// device / recover the port before giving up on a recovery cycle.
static constexpr uint32_t PORT_RESET_OFF_MS = 1000;
static constexpr uint32_t PORT_RESET_MAX_MS = 15000;

static const char *transfer_status_name(usb_transfer_status_t s) {
    switch (s) {
        case USB_TRANSFER_STATUS_COMPLETED: return "COMPLETED";
        case USB_TRANSFER_STATUS_ERROR:     return "ERROR";
        case USB_TRANSFER_STATUS_TIMED_OUT: return "TIMED_OUT";
        case USB_TRANSFER_STATUS_CANCELED:  return "CANCELED";
        case USB_TRANSFER_STATUS_STALL:     return "STALL";
        case USB_TRANSFER_STATUS_OVERFLOW:  return "OVERFLOW";
        case USB_TRANSFER_STATUS_SKIPPED:   return "SKIPPED";
        case USB_TRANSFER_STATUS_NO_DEVICE: return "NO_DEVICE";
        default:                            return "UNKNOWN";
    }
}

Esp32UsbTransport::Esp32UsbTransport() {
    ctrl_done_sem_ = xSemaphoreCreateBinary();
}

Esp32UsbTransport::~Esp32UsbTransport() {
    deinitialize();
    // If a transfer is somehow still in flight the semaphore must outlive it;
    // leaking it is preferable to a use-after-free in the completion callback.
    if (ctrl_done_sem_ && !ctrl_inflight_.load()) {
        vSemaphoreDelete(ctrl_done_sem_);
        ctrl_done_sem_ = nullptr;
    }
}

// ============================================================================
// Lifecycle
// ============================================================================

esp_err_t Esp32UsbTransport::initialize() {
    if (initialized_.load()) {
        return ESP_OK;
    }
    if (!ctrl_done_sem_) {
        set_last_error("Control transfer semaphore missing");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(ESP32_USB_TAG, "Initializing ESP32 USB transport");

    if (!ctrl_xfer_) {
        esp_err_t ret = usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + CTRL_XFER_MAX_DATA, 0, &ctrl_xfer_);
        if (ret != ESP_OK) {
            ctrl_xfer_ = nullptr;
            set_last_error("Failed to allocate control transfer: " + std::string(esp_err_to_name(ret)));
            return ret;
        }
    }

    esp_err_t ret = setup_usb_host();
    if (ret != ESP_OK) {
        set_last_error("Failed to setup USB host: " + std::string(esp_err_to_name(ret)));
        return ret;
    }

    ret = register_client_and_scan();
    if (ret != ESP_OK) {
        teardown_usb_host();
        return ret;
    }

    initialized_ = true;
    ESP_LOGI(ESP32_USB_TAG, "ESP32 USB transport initialized - waiting for USB device connection events");
    return ESP_OK;
}

esp_err_t Esp32UsbTransport::deinitialize() {
    if (!initialized_.load() && !usb_tasks_running_.load()) {
        return ESP_OK;
    }

    ESP_LOGI(ESP32_USB_TAG, "Deinitializing ESP32 USB transport");

    // Signal early so the client task and callbacks bail out. Do NOT hold
    // device_mutex_ here: teardown waits for the client task, which takes the
    // mutex on every iteration.
    initialized_ = false;
    connected_ = false;
    new_device_pending_ = false;

    esp_err_t ret = teardown_usb_host();
    if (ret != ESP_OK) {
        ESP_LOGW(ESP32_USB_TAG, "USB teardown had issues: %s", esp_err_to_name(ret));
    }

    if (ctrl_xfer_ && !ctrl_inflight_.load()) {
        usb_host_transfer_free(ctrl_xfer_);
        ctrl_xfer_ = nullptr;
    }

    return ret;
}

bool Esp32UsbTransport::is_connected() const {
    return connected_.load() && initialized_.load();
}

uint16_t Esp32UsbTransport::get_vendor_id() const {
    std::lock_guard<std::mutex> lock(device_mutex_);
    return device_.vendor_id;
}

uint16_t Esp32UsbTransport::get_product_id() const {
    std::lock_guard<std::mutex> lock(device_mutex_);
    return device_.product_id;
}

std::string Esp32UsbTransport::get_last_error() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

void Esp32UsbTransport::set_last_error(const std::string& error) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error;
    ESP_LOGW(ESP32_USB_TAG, "%s", error.c_str());
}

void Esp32UsbTransport::request_recovery(const char* reason) {
    if (!usb_tasks_running_.load()) {
        return;
    }
    if (port_powered_off_.load()) {
        return;  // recovery already in progress
    }
    if (!recovery_requested_.exchange(true)) {
        ESP_LOGW(ESP32_USB_TAG, "USB recovery requested: %s", reason ? reason : "unspecified");
    }
}

// ============================================================================
// Control transfers
// ============================================================================

void Esp32UsbTransport::ctrl_transfer_done_cb(usb_transfer_t* transfer) {
    auto* self = static_cast<Esp32UsbTransport*>(transfer->context);
    self->ctrl_status_ = transfer->status;
    self->ctrl_actual_bytes_ = transfer->actual_num_bytes;
    xSemaphoreGive(self->ctrl_done_sem_);
}

esp_err_t Esp32UsbTransport::control_transfer(uint8_t bmRequestType, uint8_t bRequest,
                                              uint16_t wValue, uint16_t wIndex,
                                              uint8_t* data, size_t* data_len,
                                              uint32_t timeout_ms, const char* what) {
    std::lock_guard<std::mutex> xfer_lock(xfer_mutex_);

    const bool is_in = (bmRequestType & USB_BM_REQUEST_TYPE_DIR_IN) != 0;
    size_t len = data_len ? *data_len : 0;
    if (len > CTRL_XFER_MAX_DATA) len = CTRL_XFER_MAX_DATA;
    if (len > 0 && !data) return ESP_ERR_INVALID_ARG;
    if (is_in && data_len) *data_len = 0;
    if (timeout_ms < 100) timeout_ms = 100;

    {
        std::lock_guard<std::mutex> lock(device_mutex_);
        if (!initialized_.load() || device_gone_pending_.load() || port_powered_off_.load() ||
            !device_.dev_hdl || !ctrl_xfer_) {
            return ESP_ERR_INVALID_STATE;
        }

        auto* setup = reinterpret_cast<usb_setup_packet_t*>(ctrl_xfer_->data_buffer);
        setup->bmRequestType = bmRequestType;
        setup->bRequest = bRequest;
        setup->wValue = wValue;
        setup->wIndex = wIndex;
        setup->wLength = static_cast<uint16_t>(len);
        if (len > 0) {
            if (is_in) {
                memset(ctrl_xfer_->data_buffer + sizeof(usb_setup_packet_t), 0, len);
            } else {
                memcpy(ctrl_xfer_->data_buffer + sizeof(usb_setup_packet_t), data, len);
            }
        }

        ctrl_xfer_->device_handle = device_.dev_hdl;
        ctrl_xfer_->bEndpointAddress = 0;
        ctrl_xfer_->num_bytes = sizeof(usb_setup_packet_t) + len;
        ctrl_xfer_->timeout_ms = timeout_ms;  // not implemented by ESP-IDF; kept for documentation
        ctrl_xfer_->callback = ctrl_transfer_done_cb;
        ctrl_xfer_->context = this;

        xSemaphoreTake(ctrl_done_sem_, 0);  // clear any stale completion
        ctrl_status_ = USB_TRANSFER_STATUS_ERROR;
        ctrl_actual_bytes_ = 0;
        ctrl_inflight_ = true;

        esp_err_t ret = usb_host_transfer_submit_control(device_.client_hdl, ctrl_xfer_);
        if (ret != ESP_OK) {
            ctrl_inflight_ = false;
            ESP_LOGD(ESP32_USB_TAG, "%s: submit failed: %s", what, esp_err_to_name(ret));
            return ret;
        }
    }

    // Wait for completion without holding device_mutex_ (the client task needs
    // it to deliver events). The transfer is never abandoned: if the device
    // does not answer within two timeouts, request a recovery and keep waiting
    // until the stack cancels the transfer as part of the port power-cycle.
    uint32_t waited_ms = 0;
    uint32_t next_report_ms = 30000;
    int misses = 0;
    while (xSemaphoreTake(ctrl_done_sem_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        waited_ms += timeout_ms;
        misses++;
        if (misses == 1) {
            ESP_LOGW(ESP32_USB_TAG, "%s: no response after %" PRIu32 "ms", what, waited_ms);
        } else if (misses == 2) {
            ctrl_stalls_++;
            ESP_LOGE(ESP32_USB_TAG, "%s: no response after %" PRIu32 "ms (stall #%" PRIu32 "), requesting USB port reset",
                     what, waited_ms, ctrl_stalls_.load());
            request_recovery("control transfer stalled");
        } else if (waited_ms >= next_report_ms) {
            next_report_ms += 30000;
            ESP_LOGE(ESP32_USB_TAG, "%s: still waiting for the USB stack to cancel the stalled transfer (%" PRIu32 "s)",
                     what, waited_ms / 1000);
            request_recovery("control transfer still stalled");
        }
    }
    ctrl_inflight_ = false;

    const usb_transfer_status_t status = ctrl_status_;
    if (status == USB_TRANSFER_STATUS_COMPLETED) {
        if (is_in && data_len) {
            size_t received = ctrl_actual_bytes_ > sizeof(usb_setup_packet_t)
                ? ctrl_actual_bytes_ - sizeof(usb_setup_packet_t) : 0;
            size_t copy_len = std::min(received, len);
            if (copy_len > 0) {
                memcpy(data, ctrl_xfer_->data_buffer + sizeof(usb_setup_packet_t), copy_len);
            }
            *data_len = copy_len;
        }
        return ESP_OK;
    }

    ESP_LOGD(ESP32_USB_TAG, "%s: failed, usb_status=%s", what, transfer_status_name(status));
    if (status == USB_TRANSFER_STATUS_NO_DEVICE || status == USB_TRANSFER_STATUS_CANCELED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (status == USB_TRANSFER_STATUS_TIMED_OUT) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_FAIL;
}

esp_err_t Esp32UsbTransport::hid_get_report(uint8_t report_type, uint8_t report_id,
                                           uint8_t* data, size_t* data_len,
                                           uint32_t timeout_ms) {
    if (!data || !data_len || *data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    // HID reports are at most 64 bytes on the devices we talk to.
    if (*data_len > 64) *data_len = 64;

    uint16_t interface_num;
    {
        std::lock_guard<std::mutex> lock(device_mutex_);
        interface_num = device_.interface_num;
    }

    ESP_LOGV(ESP32_USB_TAG, "HID GET_REPORT: type=0x%02X, id=0x%02X, max_len=%zu",
             report_type, report_id, *data_len);

    char what[32];
    snprintf(what, sizeof(what), "HID GET_REPORT 0x%02X", report_id);
    esp_err_t ret = control_transfer(
        USB_BM_REQUEST_TYPE_DIR_IN | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        0x01,  // GET_REPORT
        static_cast<uint16_t>((report_type << 8) | report_id),
        interface_num, data, data_len, timeout_ms, what);

    if (ret == ESP_OK) {
        if (*data_len == 0) {
            return ESP_FAIL;
        }
        ESP_LOGV(ESP32_USB_TAG, "HID GET_REPORT 0x%02X: received %zu bytes", report_id, *data_len);
    } else {
        *data_len = 0;
    }
    return ret;
}

esp_err_t Esp32UsbTransport::hid_set_report(uint8_t report_type, uint8_t report_id,
                                           const uint8_t* data, size_t data_len,
                                           uint32_t timeout_ms) {
    if (!data || data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t interface_num;
    {
        std::lock_guard<std::mutex> lock(device_mutex_);
        interface_num = device_.interface_num;
    }

    ESP_LOGD(ESP32_USB_TAG, "HID SET_REPORT: type=0x%02X, id=0x%02X, len=%zu",
             report_type, report_id, data_len);

    char what[32];
    snprintf(what, sizeof(what), "HID SET_REPORT 0x%02X", report_id);
    size_t len = data_len;
    esp_err_t ret = control_transfer(
        USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        0x09,  // SET_REPORT
        static_cast<uint16_t>((report_type << 8) | report_id),
        interface_num, const_cast<uint8_t*>(data), &len, timeout_ms, what);

    if (ret == ESP_OK) {
        ESP_LOGD(ESP32_USB_TAG, "HID SET_REPORT 0x%02X success", report_id);
    } else {
        ESP_LOGW(ESP32_USB_TAG, "HID SET_REPORT 0x%02X failed: %s", report_id, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t Esp32UsbTransport::get_string_descriptor(uint8_t string_index, std::string& result) {
    result.clear();

    ESP_LOGD(ESP32_USB_TAG, "USB GET_STRING_DESCRIPTOR: index=%d, language_id=0x0409", string_index);

    uint8_t desc[255];
    size_t desc_len = sizeof(desc);
    esp_err_t ret = control_transfer(
        USB_BM_REQUEST_TYPE_DIR_IN | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_DEVICE,
        USB_B_REQUEST_GET_DESCRIPTOR,
        static_cast<uint16_t>((USB_B_DESCRIPTOR_TYPE_STRING << 8) | string_index),
        0x0409, desc, &desc_len, timing::USB_CONTROL_TRANSFER_TIMEOUT_MS, "GET_STRING_DESCRIPTOR");

    if (ret != ESP_OK) {
        ESP_LOGD(ESP32_USB_TAG, "String descriptor %d request failed: %s", string_index, esp_err_to_name(ret));
        return ret;
    }
    if (desc_len < 2) {
        ESP_LOGW(ESP32_USB_TAG, "String descriptor %d too short: %zu bytes", string_index, desc_len);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t bLength = desc[0];
    uint8_t bDescriptorType = desc[1];
    if (bDescriptorType != USB_B_DESCRIPTOR_TYPE_STRING || bLength < 2) {
        ESP_LOGW(ESP32_USB_TAG, "Invalid string descriptor %d: type=0x%02X, length=%d",
                 string_index, bDescriptorType, bLength);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // UTF-16LE payload; only ASCII is preserved.
    size_t string_data_len = std::min(static_cast<size_t>(bLength - 2), desc_len - 2);
    const uint8_t* string_data = desc + 2;
    result.reserve(string_data_len / 2);
    for (size_t i = 0; i + 1 < string_data_len; i += 2) {
        uint16_t utf16_char = string_data[i] | (string_data[i + 1] << 8);
        if (utf16_char > 0 && utf16_char < 128) {
            result += static_cast<char>(utf16_char);
        } else if (utf16_char >= 128) {
            result += '?';
        }
    }
    while (!result.empty() && std::isspace(static_cast<unsigned char>(result.back()))) {
        result.pop_back();
    }

    ESP_LOGI(ESP32_USB_TAG, "USB string descriptor %d: \"%s\"", string_index, result.c_str());
    return ESP_OK;
}

esp_err_t Esp32UsbTransport::get_hid_report_descriptor(std::vector<uint8_t>& descriptor) {
    descriptor.clear();

    // Find the HID class descriptor in the active configuration to learn the
    // report descriptor length, then fetch it with GET_DESCRIPTOR(Report).
    uint16_t report_desc_length = 0;
    uint16_t interface_num;
    {
        std::lock_guard<std::mutex> lock(device_mutex_);
        if (device_gone_pending_.load() || !device_.dev_hdl) {
            set_last_error("USB device not ready for report descriptor fetch");
            return ESP_ERR_INVALID_STATE;
        }
        interface_num = device_.interface_num;

        const usb_config_desc_t* config_desc = nullptr;
        esp_err_t ret = usb_host_get_active_config_descriptor(device_.dev_hdl, &config_desc);
        if (ret != ESP_OK || !config_desc) {
            set_last_error("Failed to get config descriptor: " + std::string(esp_err_to_name(ret)));
            return ret != ESP_OK ? ret : ESP_FAIL;
        }

        const uint8_t* raw = reinterpret_cast<const uint8_t*>(config_desc);
        size_t total_len = config_desc->wTotalLength;
        size_t pos = 0;
        bool in_hid_interface = false;
        while (pos + 2 <= total_len) {
            uint8_t bLength = raw[pos];
            uint8_t bDescriptorType = raw[pos + 1];
            if (bLength == 0 || pos + bLength > total_len) break;

            if (bDescriptorType == 0x04 && bLength >= 9) {  // Interface descriptor
                in_hid_interface = (raw[pos + 5] == USB_CLASS_HID) && (raw[pos + 2] == interface_num);
            } else if (bDescriptorType == 0x21 && in_hid_interface && bLength >= 9) {  // HID class descriptor
                if (raw[pos + 6] == 0x22) {  // Report descriptor
                    report_desc_length = raw[pos + 7] | (raw[pos + 8] << 8);
                }
                break;
            }
            pos += bLength;
        }
    }

    if (report_desc_length == 0) {
        ESP_LOGW(ESP32_USB_TAG, "Could not find HID class descriptor, using default length 512");
        report_desc_length = 512;
    }
    if (report_desc_length > CTRL_XFER_MAX_DATA) {
        ESP_LOGW(ESP32_USB_TAG, "Report descriptor length %u too large, capping at %zu",
                 report_desc_length, CTRL_XFER_MAX_DATA);
        report_desc_length = CTRL_XFER_MAX_DATA;
    }
    ESP_LOGD(ESP32_USB_TAG, "Fetching HID report descriptor (%u bytes)...", report_desc_length);

    descriptor.resize(report_desc_length);
    size_t len = report_desc_length;
    esp_err_t ret = control_transfer(
        USB_BM_REQUEST_TYPE_DIR_IN | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        USB_B_REQUEST_GET_DESCRIPTOR,
        static_cast<uint16_t>(0x22 << 8),  // Report descriptor, index 0
        interface_num, descriptor.data(), &len, timing::USB_CONTROL_TRANSFER_TIMEOUT_MS,
        "GET_DESCRIPTOR(Report)");

    if (ret != ESP_OK || len == 0) {
        descriptor.clear();
        ESP_LOGW(ESP32_USB_TAG, "HID report descriptor request failed: %s",
                 ret != ESP_OK ? esp_err_to_name(ret) : "no data");
        return ret != ESP_OK ? ret : ESP_FAIL;
    }

    descriptor.resize(len);
    ESP_LOGI(ESP32_USB_TAG, "HID report descriptor fetched: %zu bytes", descriptor.size());
    return ESP_OK;
}

// ============================================================================
// USB Host library setup / teardown
// ============================================================================

esp_err_t Esp32UsbTransport::setup_usb_host() {
    if (usb_tasks_running_.load()) {
        return ESP_OK;
    }

    usb_lib_task_exited_ = false;
    usb_client_task_exited_ = false;
    usb_host_install_state_ = 0;
    usb_tasks_running_ = true;
    recovery_requested_ = false;
    port_powered_off_ = false;

    if (xTaskCreate(usb_lib_task, "usb_lib_task", 4096, this, 2, &usb_lib_task_handle_) != pdTRUE) {
        ESP_LOGE(ESP32_USB_TAG, "Failed to create USB Host Library task");
        usb_tasks_running_ = false;
        usb_lib_task_exited_ = true;
        usb_client_task_exited_ = true;
        return ESP_FAIL;
    }

    // The lib task performs usb_host_install(); wait for its verdict before
    // registering a client or starting the client task.
    for (int i = 0; i < 100 && usb_host_install_state_.load() == 0; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (usb_host_install_state_.load() != 1) {
        ESP_LOGE(ESP32_USB_TAG, "USB Host Library did not install (state %d)", usb_host_install_state_.load());
        usb_tasks_running_ = false;
        for (int i = 0; i < 240 && !usb_lib_task_exited_.load(); i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        usb_lib_task_handle_ = nullptr;
        usb_client_task_exited_ = true;
        return ESP_ERR_INVALID_STATE;
    }

    if (xTaskCreate(usb_client_task, "usb_client_task", 6144, this, 3, &usb_client_task_handle_) != pdTRUE) {
        ESP_LOGE(ESP32_USB_TAG, "Failed to create USB client task");
        usb_client_task_exited_ = true;
        usb_tasks_running_ = false;
        for (int i = 0; i < 200 && !usb_lib_task_exited_.load(); i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        usb_lib_task_handle_ = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(ESP32_USB_TAG, "USB Host tasks created successfully");
    return ESP_OK;
}

esp_err_t Esp32UsbTransport::teardown_usb_host() {
    if (!usb_tasks_running_.load()) {
        return ESP_OK;
    }

    ESP_LOGI(ESP32_USB_TAG, "Stopping USB Host tasks...");

    // A control transfer may still be in flight on another task. The stack
    // asserts if the device is closed while that is the case, so cancel it via
    // a port power-cycle first and wait for it to complete.
    if (ctrl_inflight_.load()) {
        request_recovery("teardown with control transfer in flight");
        for (int i = 0; i < 200 && ctrl_inflight_.load(); i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (ctrl_inflight_.load()) {
            ESP_LOGE(ESP32_USB_TAG, "Control transfer still in flight after 10s; refusing to close device");
            return ESP_ERR_INVALID_STATE;
        }
    }

    usb_tasks_running_ = false;
    for (int i = 0; i < 60 && !usb_client_task_exited_.load(); i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!usb_client_task_exited_.load()) {
        ESP_LOGW(ESP32_USB_TAG, "USB client task did not exit in time");
    }
    usb_client_task_handle_ = nullptr;

    {
        std::lock_guard<std::mutex> lock(device_mutex_);
        usb_device_handle_t dev = device_.dev_hdl;
        usb_host_client_handle_t cli = device_.client_hdl;
        device_gone_pending_ = false;

        if (dev && cli) {
            ESP_LOGI(ESP32_USB_TAG, "Cleaning up device resources");
            release_device_locked(dev, cli);
        }
        device_.dev_hdl = nullptr;

        if (cli) {
            esp_err_t dereg_ret = usb_host_client_deregister(cli);
            ESP_LOGI(ESP32_USB_TAG, "  client_deregister: %s", esp_err_to_name(dereg_ret));
            device_.client_hdl = nullptr;
        }
        device_.address = 0;
        device_.vendor_id = 0;
        device_.product_id = 0;
    }
    connected_ = false;

    // Deregistering the last client makes the lib task free all devices,
    // uninstall the library and exit.
    for (int i = 0; i < 200 && !usb_lib_task_exited_.load(); i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!usb_lib_task_exited_.load()) {
        ESP_LOGW(ESP32_USB_TAG, "USB lib task did not exit in time");
    }
    usb_lib_task_handle_ = nullptr;

    device_gone_pending_ = false;
    new_device_pending_ = false;
    port_powered_off_ = false;
    recovery_requested_ = false;

    ESP_LOGI(ESP32_USB_TAG, "USB Host tasks stopped");
    return ESP_OK;
}

esp_err_t Esp32UsbTransport::register_client_and_scan() {
    std::lock_guard<std::mutex> lock(device_mutex_);

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = usb_client_event_callback,
            .callback_arg = this
        }
    };

    esp_err_t ret = usb_host_client_register(&client_config, &device_.client_hdl);
    if (ret != ESP_OK) {
        device_.client_hdl = nullptr;
        set_last_error("Client register failed: " + std::string(esp_err_to_name(ret)));
        return ret;
    }
    ESP_LOGI(ESP32_USB_TAG, "USB client registered, waiting for device connection events...");

    // Devices enumerated before the client existed do not generate NEW_DEV
    // events, so pick them up explicitly.
    vTaskDelay(pdMS_TO_TICKS(100));
    int num_dev = 10;
    uint8_t dev_addr_list[10];
    if (usb_host_device_addr_list_fill(num_dev, dev_addr_list, &num_dev) == ESP_OK && num_dev > 0) {
        ESP_LOGI(ESP32_USB_TAG, "Found %d existing USB device(s) during initial enumeration", num_dev);
        for (int i = 0; i < num_dev; i++) {
            handle_new_device_locked(dev_addr_list[i]);
        }
    } else {
        ESP_LOGI(ESP32_USB_TAG, "No existing USB devices found - waiting for connection events");
    }

    return ESP_OK;
}

esp_err_t Esp32UsbTransport::claim_interface() {
    const usb_config_desc_t* config_desc;
    esp_err_t ret = usb_host_get_active_config_descriptor(device_.dev_hdl, &config_desc);
    if (ret != ESP_OK) {
        set_last_error("Failed to get config descriptor");
        return ret;
    }

    const usb_intf_desc_t* intf_desc = nullptr;
    int offset = 0;
    for (int i = 0; i < config_desc->bNumInterfaces; i++) {
        intf_desc = usb_parse_interface_descriptor(config_desc, i, 0, &offset);
        if (intf_desc && intf_desc->bInterfaceClass == USB_CLASS_HID) {
            device_.interface_num = intf_desc->bInterfaceNumber;
            break;
        }
    }
    if (!intf_desc || intf_desc->bInterfaceClass != USB_CLASS_HID) {
        set_last_error("No HID interface found");
        return ESP_ERR_NOT_FOUND;
    }

    ret = usb_host_interface_claim(device_.client_hdl, device_.dev_hdl, device_.interface_num, 0);
    if (ret != ESP_OK) {
        set_last_error("Failed to claim interface: " + std::string(esp_err_to_name(ret)));
        return ret;
    }
    return ESP_OK;
}

esp_err_t Esp32UsbTransport::find_endpoints() {
    const usb_config_desc_t* config_desc;
    esp_err_t ret = usb_host_get_active_config_descriptor(device_.dev_hdl, &config_desc);
    if (ret != ESP_OK) {
        return ret;
    }

    int offset = 0;
    const usb_intf_desc_t* intf_desc = usb_parse_interface_descriptor(
        config_desc, device_.interface_num, 0, &offset);
    if (!intf_desc) {
        set_last_error("Interface descriptor not found");
        return ESP_ERR_NOT_FOUND;
    }

    device_.ep_in = 0;
    device_.ep_out = 0;
    int ep_offset = offset;
    for (int i = 0; i < intf_desc->bNumEndpoints; i++) {
        const usb_ep_desc_t* ep_desc = usb_parse_endpoint_descriptor_by_index(
            intf_desc, i, config_desc->wTotalLength, &ep_offset);
        if (!ep_desc) {
            ESP_LOGW(ESP32_USB_TAG, "Failed to parse endpoint %d", i);
            continue;
        }
        if (USB_EP_DESC_GET_EP_DIR(ep_desc)) {
            device_.ep_in = ep_desc->bEndpointAddress;
            device_.max_packet_size_in = ep_desc->wMaxPacketSize;
            ESP_LOGD(ESP32_USB_TAG, "Found IN endpoint: 0x%02X (max packet size: %d)",
                     device_.ep_in, device_.max_packet_size_in);
        } else {
            device_.ep_out = ep_desc->bEndpointAddress;
            device_.max_packet_size_out = ep_desc->wMaxPacketSize;
            ESP_LOGD(ESP32_USB_TAG, "Found OUT endpoint: 0x%02X (max packet size: %d)",
                     device_.ep_out, device_.max_packet_size_out);
        }
    }

    if (device_.ep_in == 0) {
        set_last_error("No IN endpoint found");
        return ESP_ERR_NOT_FOUND;
    }
    if (device_.ep_out == 0) {
        ESP_LOGI(ESP32_USB_TAG, "Input-only HID device (no OUT endpoint); using control transfers only");
    }
    return ESP_OK;
}

// ============================================================================
// Device lifecycle (usb_client_task context, device_mutex_ held)
// ============================================================================

void Esp32UsbTransport::handle_new_device_locked(uint8_t dev_addr) {
    ESP_LOGI(ESP32_USB_TAG, "Handling new USB device at address %d", dev_addr);

    if (device_.dev_hdl != nullptr) {
        ESP_LOGW(ESP32_USB_TAG, "Device already connected - skipping new device at address %d", dev_addr);
        return;
    }

    esp_err_t ret = usb_host_device_open(device_.client_hdl, dev_addr, &device_.dev_hdl);
    if (ret != ESP_OK) {
        device_.dev_hdl = nullptr;
        ESP_LOGE(ESP32_USB_TAG, "Failed to open device at address %d: %s", dev_addr, esp_err_to_name(ret));
        return;
    }

    usb_device_info_t dev_info;
    ret = usb_host_device_info(device_.dev_hdl, &dev_info);
    if (ret != ESP_OK) {
        ESP_LOGE(ESP32_USB_TAG, "Failed to get device info: %s", esp_err_to_name(ret));
        usb_host_device_close(device_.client_hdl, device_.dev_hdl);
        device_.dev_hdl = nullptr;
        return;
    }
    device_.address = dev_addr;
    device_.speed = dev_info.speed;

    const usb_device_desc_t* device_desc;
    ret = usb_host_get_device_descriptor(device_.dev_hdl, &device_desc);
    if (ret == ESP_OK) {
        device_.vendor_id = device_desc->idVendor;
        device_.product_id = device_desc->idProduct;
        ESP_LOGI(ESP32_USB_TAG, "USB device opened: VID=0x%04X, PID=0x%04X, Speed=%d",
                 device_.vendor_id, device_.product_id, dev_info.speed);

        if (device_desc->bDeviceClass == USB_CLASS_HID || device_desc->bDeviceClass == 0x00) {
            ret = claim_interface();
            if (ret == ESP_OK) {
                ret = find_endpoints();
                if (ret == ESP_OK) {
                    device_gone_pending_ = false;
                    connected_ = true;
                    ESP_LOGI(ESP32_USB_TAG, "UPS device successfully configured and ready");
                    return;
                }
                usb_host_interface_release(device_.client_hdl, device_.dev_hdl, device_.interface_num);
            }
        } else {
            ESP_LOGW(ESP32_USB_TAG, "Connected device is not a HID device (class=0x%02X)", device_desc->bDeviceClass);
        }
    }

    usb_host_device_close(device_.client_hdl, device_.dev_hdl);
    device_.dev_hdl = nullptr;
    device_.vendor_id = 0;
    device_.product_id = 0;
}

void Esp32UsbTransport::release_device_locked(usb_device_handle_t dev, usb_host_client_handle_t cli) {
    // Deliver any completions still queued for this client.
    usb_host_client_handle_events(cli, 0);

    // Documented teardown order: halt/flush claimed endpoints, release the
    // interface, close the device.
    uint8_t eps[] = {device_.ep_in, device_.ep_out};
    for (auto ep : eps) {
        if (ep == 0) continue;
        esp_err_t hr = usb_host_endpoint_halt(dev, ep);
        if (hr == ESP_OK || hr == ESP_ERR_INVALID_STATE) {
            usb_host_endpoint_flush(dev, ep);
        }
    }
    usb_host_client_handle_events(cli, 0);

    esp_err_t rel_ret = usb_host_interface_release(cli, dev, device_.interface_num);
    ESP_LOGD(ESP32_USB_TAG, "  interface_release: %s", esp_err_to_name(rel_ret));

    esp_err_t close_ret = usb_host_device_close(cli, dev);
    ESP_LOGD(ESP32_USB_TAG, "  device_close: %s", esp_err_to_name(close_ret));
}

void Esp32UsbTransport::process_device_gone_locked() {
    device_gone_pending_ = false;
    connected_ = false;

    if (device_.dev_hdl && device_.client_hdl) {
        release_device_locked(device_.dev_hdl, device_.client_hdl);
    }
    device_.dev_hdl = nullptr;
    device_.address = 0;
    device_.vendor_id = 0;
    device_.product_id = 0;
    device_.ep_in = 0;
    device_.ep_out = 0;

    ESP_LOGI(ESP32_USB_TAG, "USB device resources cleaned up");
}

// Root-port power cycle. Powering the port off makes the host stack report
// the device gone (which cancels any in-flight EP0 transfer), the client then
// closes the device, the stack recovers the port, and powering it back on
// re-enumerates the device with a fresh bus reset.
void Esp32UsbTransport::process_port_reset() {
#if !UPS_HID_HAVE_ROOT_PORT_POWER
    if (recovery_requested_.exchange(false)) {
        ESP_LOGE(ESP32_USB_TAG, "USB recovery requested but this ESP-IDF version cannot power-cycle "
                 "the root port; a reboot will be required if the device stays unresponsive");
    }
    return;
#else
    uint32_t now = millis();

    if (!port_powered_off_.load()) {
        if (!recovery_requested_.exchange(false)) {
            return;
        }
        esp_err_t ret = usb_host_lib_set_root_port_power(false);
        if (ret != ESP_OK) {
            ESP_LOGE(ESP32_USB_TAG, "USB recovery: failed to power off root port: %s", esp_err_to_name(ret));
            return;
        }
        port_resets_++;
        port_reset_started_ms_ = now;
        port_powered_off_ = true;
        ESP_LOGW(ESP32_USB_TAG, "USB recovery #%" PRIu32 ": root port powered off", port_resets_.load());
        return;
    }

    uint32_t elapsed = now - port_reset_started_ms_;
    if (elapsed < PORT_RESET_OFF_MS) {
        return;
    }

    bool device_closed;
    {
        std::lock_guard<std::mutex> lock(device_mutex_);
        device_closed = (device_.dev_hdl == nullptr);
    }
    if (!device_closed && elapsed < PORT_RESET_MAX_MS) {
        return;  // still waiting for DEV_GONE processing
    }

    // The port can only be re-powered once the stack has recovered it, which
    // happens after the old device object is freed; retry until it succeeds.
    esp_err_t ret = usb_host_lib_set_root_port_power(true);
    if (ret == ESP_OK) {
        port_powered_off_ = false;
        recovery_requested_ = false;
        ESP_LOGI(ESP32_USB_TAG, "USB recovery: root port powered on, waiting for device re-enumeration");
    } else if (elapsed > PORT_RESET_MAX_MS) {
        port_powered_off_ = false;
        ESP_LOGE(ESP32_USB_TAG, "USB recovery: could not re-power root port after %" PRIu32 "s: %s",
                 elapsed / 1000, esp_err_to_name(ret));
    }
#endif
}

// ============================================================================
// Tasks and callbacks
// ============================================================================

void Esp32UsbTransport::usb_client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg) {
    auto* transport = static_cast<Esp32UsbTransport*>(arg);

    // Runs inside usb_host_client_handle_events(); only set flags here.
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            ESP_LOGI(ESP32_USB_TAG, "New USB device detected: address=%d", event_msg->new_dev.address);
            transport->new_device_address_.store(event_msg->new_dev.address);
            transport->new_device_pending_.store(true);
            break;

        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            ESP_LOGI(ESP32_USB_TAG, "USB device disconnected");
            transport->connected_ = false;
            transport->device_gone_pending_ = true;
            break;

        default:
            ESP_LOGW(ESP32_USB_TAG, "Unhandled USB client event: %d", event_msg->event);
            break;
    }
}

void Esp32UsbTransport::usb_lib_task(void* arg) {
    auto* transport = static_cast<Esp32UsbTransport*>(arg);

    ESP_LOGI(ESP32_USB_TAG, "USB Host Library task starting...");

    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;

    esp_err_t ret = usb_host_install(&host_config);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(ESP32_USB_TAG, "USB Host install returned INVALID_STATE, retrying after delay...");
        vTaskDelay(pdMS_TO_TICKS(500));
        ret = usb_host_install(&host_config);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(ESP32_USB_TAG, "USB Host install failed: %s", esp_err_to_name(ret));
        transport->usb_host_install_state_ = -1;
        transport->usb_lib_task_exited_ = true;
        vTaskDelete(nullptr);
        return;
    }
    transport->usb_host_install_state_ = 1;
    ESP_LOGI(ESP32_USB_TAG, "USB Host library installed successfully");

    // Keep handling events until the last client is deregistered and every
    // device has been freed; only then is usb_host_uninstall() legal. A
    // deadline after shutdown is requested guards against a client that never
    // deregisters.
    bool has_clients = true;
    bool has_devices = false;
    uint32_t shutdown_requested_ms = 0;

    while (has_clients) {
        uint32_t event_flags = 0;
        ret = usb_host_lib_handle_events(pdMS_TO_TICKS(500), &event_flags);

        if (ret == ESP_OK) {
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
                ESP_LOGI(ESP32_USB_TAG, "No more USB clients");
                if (usb_host_device_free_all() == ESP_OK) {
                    has_clients = false;
                } else {
                    has_devices = true;
                }
            }
            if (has_devices && (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)) {
                ESP_LOGI(ESP32_USB_TAG, "All devices freed");
                has_clients = false;
            }
        } else if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGE(ESP32_USB_TAG, "USB Host event handling failed: %s", esp_err_to_name(ret));
        }

        if (!transport->usb_tasks_running_.load()) {
            if (shutdown_requested_ms == 0) {
                shutdown_requested_ms = millis();
            } else if (millis() - shutdown_requested_ms > 10000) {
                ESP_LOGE(ESP32_USB_TAG, "USB client never deregistered; forcing lib task exit");
                break;
            }
        }
    }

    ESP_LOGI(ESP32_USB_TAG, "Uninstalling USB Host library");
    ret = usb_host_uninstall();
    if (ret != ESP_OK) {
        ESP_LOGE(ESP32_USB_TAG, "USB Host uninstall failed: %s", esp_err_to_name(ret));
    }
    transport->usb_host_install_state_ = 0;

    ESP_LOGI(ESP32_USB_TAG, "USB Host Library task ending");
    transport->usb_lib_task_exited_ = true;
    vTaskDelete(nullptr);
}

void Esp32UsbTransport::usb_client_task(void* arg) {
    auto* transport = static_cast<Esp32UsbTransport*>(arg);

    ESP_LOGI(ESP32_USB_TAG, "USB client task started");

    while (transport->usb_tasks_running_.load()) {
        usb_host_client_handle_t client_hdl;
        {
            std::lock_guard<std::mutex> lock(transport->device_mutex_);
            client_hdl = transport->device_.client_hdl;
        }
        if (!client_hdl) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Transfer completions and device events are delivered here. Must not
        // hold device_mutex_: the event callback only sets flags, but the
        // deferred handlers below take the mutex.
        esp_err_t ret = usb_host_client_handle_events(client_hdl, pdMS_TO_TICKS(timing::USB_CLIENT_EVENT_TIMEOUT_MS));
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            ESP_LOGW(ESP32_USB_TAG, "USB client event handling failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (transport->device_gone_pending_.load()) {
            std::lock_guard<std::mutex> lock(transport->device_mutex_);
            if (transport->device_gone_pending_.load()) {
                transport->process_device_gone_locked();
            }
        }

        // Leave the event pending until the transport is ready to open devices
        // (initialize() may still be finishing).
        if (transport->new_device_pending_.load() && transport->initialized_.load() &&
            !transport->port_powered_off_.load()) {
            transport->new_device_pending_ = false;
            uint8_t addr = transport->new_device_address_.load();
            std::lock_guard<std::mutex> lock(transport->device_mutex_);
            transport->handle_new_device_locked(addr);
        }

        transport->process_port_reset();

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(ESP32_USB_TAG, "USB client task stopping");
    transport->usb_client_task_exited_ = true;
    vTaskDelete(nullptr);
}

} // namespace ups_hid
} // namespace esphome

#endif // USE_ESP32
