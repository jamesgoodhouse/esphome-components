#pragma once

#include "transport_interface.h"

#ifdef USE_ESP32
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <mutex>
#include <atomic>

namespace esphome {
namespace ups_hid {

/**
 * ESP32 USB Transport Implementation (ESP-IDF USB Host API)
 *
 * All HID traffic goes over the default control pipe (EP0) using a single,
 * persistent transfer object and completion semaphore. ESP-IDF does not
 * implement usb_transfer_t::timeout_ms and EP0 cannot be halted from user
 * code, so an in-flight control transfer must never be abandoned: the stack
 * asserts on usb_host_device_close() if any control transfer is still in
 * flight. Instead, a transfer that receives no response requests a recovery,
 * which power-cycles the root port. That makes the stack cancel the transfer
 * (completing it with CANCELED), tear the device down cleanly and re-enumerate
 * it with a fresh bus reset.
 */
class Esp32UsbTransport : public IUsbTransport {
public:
    Esp32UsbTransport();
    ~Esp32UsbTransport() override;

    // IUsbTransport implementation
    esp_err_t initialize() override;
    esp_err_t deinitialize() override;

    bool is_connected() const override;
    uint16_t get_vendor_id() const override;
    uint16_t get_product_id() const override;

    esp_err_t hid_get_report(uint8_t report_type, uint8_t report_id,
                           uint8_t* data, size_t* data_len,
                           uint32_t timeout_ms = 1000) override;

    esp_err_t hid_set_report(uint8_t report_type, uint8_t report_id,
                           const uint8_t* data, size_t data_len,
                           uint32_t timeout_ms = 1000) override;

    esp_err_t get_string_descriptor(uint8_t string_index,
                                  std::string& result) override;

    esp_err_t get_hid_report_descriptor(std::vector<uint8_t>& descriptor) override;

    std::string get_last_error() const override;

    void request_recovery(const char* reason) override;
    bool is_recovering() const override { return port_powered_off_.load(); }
    uint32_t get_recovery_count() const override { return port_resets_.load(); }
    uint32_t get_stall_count() const override { return ctrl_stalls_.load(); }

private:
    // Largest control transfer payload we ever request (HID report descriptors
    // are capped to this size).
    static constexpr size_t CTRL_XFER_MAX_DATA = 4096;

    struct UsbDevice {
        usb_host_client_handle_t client_hdl{nullptr};
        usb_device_handle_t dev_hdl{nullptr};
        uint8_t address{0};
        uint8_t interface_num{0};
        uint8_t ep_in{0};
        uint8_t ep_out{0};
        uint16_t vendor_id{0};
        uint16_t product_id{0};
        uint16_t max_packet_size_in{0};
        uint16_t max_packet_size_out{0};
        usb_speed_t speed{USB_SPEED_LOW};
    };

    UsbDevice device_;
    mutable std::mutex device_mutex_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> initialized_{false};

    // Deferred event flags. The USB client event callback runs inside
    // usb_host_client_handle_events(); calling USB host library functions from
    // that context is re-entrant and unsafe, so callbacks only set these flags
    // and usb_client_task processes them after the call returns.
    std::atomic<bool> device_gone_pending_{false};
    std::atomic<bool> new_device_pending_{false};
    std::atomic<uint8_t> new_device_address_{0};

    // USB Host Library tasks
    TaskHandle_t usb_lib_task_handle_{nullptr};
    TaskHandle_t usb_client_task_handle_{nullptr};
    std::atomic<bool> usb_tasks_running_{false};
    std::atomic<bool> usb_lib_task_exited_{true};
    std::atomic<bool> usb_client_task_exited_{true};
    // 0 = install pending, 1 = installed, -1 = install failed
    std::atomic<int8_t> usb_host_install_state_{0};

    // Single persistent control transfer. Serialized by xfer_mutex_; the
    // completion callback publishes status/length and gives ctrl_done_sem_.
    std::mutex xfer_mutex_;
    usb_transfer_t* ctrl_xfer_{nullptr};
    SemaphoreHandle_t ctrl_done_sem_{nullptr};
    std::atomic<bool> ctrl_inflight_{false};
    volatile usb_transfer_status_t ctrl_status_{USB_TRANSFER_STATUS_ERROR};
    volatile size_t ctrl_actual_bytes_{0};
    std::atomic<uint32_t> ctrl_stalls_{0};

    // Root-port power-cycle recovery, driven by usb_client_task.
    std::atomic<bool> recovery_requested_{false};
    std::atomic<bool> port_powered_off_{false};
    uint32_t port_reset_started_ms_{0};
    std::atomic<uint32_t> port_resets_{0};

    mutable std::mutex error_mutex_;
    std::string last_error_;

    static void usb_lib_task(void* arg);
    static void usb_client_task(void* arg);
    static void usb_client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg);
    static void ctrl_transfer_done_cb(usb_transfer_t* transfer);

    void handle_new_device_locked(uint8_t dev_addr);
    void process_device_gone_locked();
    void release_device_locked(usb_device_handle_t dev, usb_host_client_handle_t cli);
    void process_port_reset();

    esp_err_t setup_usb_host();
    esp_err_t teardown_usb_host();
    esp_err_t register_client_and_scan();
    esp_err_t claim_interface();
    esp_err_t find_endpoints();

    // Issues one control transfer on EP0 and blocks until the USB stack
    // completes or cancels it. For IN transfers *data_len is the buffer
    // capacity on entry and the received length on return.
    esp_err_t control_transfer(uint8_t bmRequestType, uint8_t bRequest,
                               uint16_t wValue, uint16_t wIndex,
                               uint8_t* data, size_t* data_len,
                               uint32_t timeout_ms, const char* what);

    void set_last_error(const std::string& error);
};

} // namespace ups_hid
} // namespace esphome

#endif // USE_ESP32
