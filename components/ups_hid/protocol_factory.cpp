#include "protocol_factory.h"
#include "protocol_apc.h"
#include "protocol_cyberpower.h"
#include "protocol_tripplite.h"
#include "protocol_generic.h"
#include "constants_ups.h"
#include "ups_hid.h"
#include "esphome/core/log.h"
#include <algorithm>

namespace esphome {
namespace ups_hid {

static const char *const FACTORY_TAG = "ups_hid.factory";

// Static registry implementations
std::unordered_map<uint16_t, std::vector<ProtocolFactory::ProtocolInfo>>& 
ProtocolFactory::get_vendor_registry() {
    static std::unordered_map<uint16_t, std::vector<ProtocolInfo>> vendor_registry;
    return vendor_registry;
}

std::vector<ProtocolFactory::ProtocolInfo>& 
ProtocolFactory::get_fallback_registry() {
    static std::vector<ProtocolInfo> fallback_registry;
    return fallback_registry;
}

void ProtocolFactory::ensure_initialized() {
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        
        // Explicitly register all built-in protocols.
        // Static self-registration via global constructors can be unreliable on
        // ESP32/ESP-IDF when the linker strips translation units with no direct
        // symbol references. This explicit registration guarantees all protocols
        // are available regardless of link-time optimizations.
        
        auto& vendor_reg = get_vendor_registry();
        auto& fallback_reg = get_fallback_registry();
        
        // Register APC protocol for vendor 0x051D (if not already registered)
        if (vendor_reg.find(usb::VENDOR_ID_APC) == vendor_reg.end()) {
            ProtocolInfo apc_info;
            apc_info.creator = create_apc_protocol;
            apc_info.name = "APC HID Protocol";
            apc_info.description = "APC Back-UPS and Smart-UPS HID protocol";
            apc_info.supported_vendors = {usb::VENDOR_ID_APC};
            apc_info.priority = 100;
            vendor_reg[usb::VENDOR_ID_APC].push_back(apc_info);
            ESP_LOGD(FACTORY_TAG, "Explicitly registered APC HID Protocol for vendor 0x%04X", usb::VENDOR_ID_APC);
        }
        
        // Register CyberPower protocol for vendor 0x0764
        if (vendor_reg.find(usb::VENDOR_ID_CYBERPOWER) == vendor_reg.end()) {
            ProtocolInfo cp_info;
            cp_info.creator = create_cyberpower_protocol;
            cp_info.name = "CyberPower HID Protocol";
            cp_info.description = "CyberPower CP series HID protocol";
            cp_info.supported_vendors = {usb::VENDOR_ID_CYBERPOWER};
            cp_info.priority = 100;
            vendor_reg[usb::VENDOR_ID_CYBERPOWER].push_back(cp_info);
            ESP_LOGD(FACTORY_TAG, "Explicitly registered CyberPower HID Protocol for vendor 0x%04X", usb::VENDOR_ID_CYBERPOWER);
        }
        
        // Register Tripp Lite protocol for vendor 0x09AE
        if (vendor_reg.find(usb::VENDOR_ID_TRIPPLITE) == vendor_reg.end()) {
            ProtocolInfo tl_info;
            tl_info.creator = create_tripplite_protocol;
            tl_info.name = "Tripp Lite HID Protocol";
            tl_info.description = "Tripp Lite USB HID UPS protocol with vendor-specific scaling";
            tl_info.supported_vendors = {usb::VENDOR_ID_TRIPPLITE};
            tl_info.priority = 100;
            vendor_reg[usb::VENDOR_ID_TRIPPLITE].push_back(tl_info);
            ESP_LOGD(FACTORY_TAG, "Explicitly registered Tripp Lite HID Protocol for vendor 0x%04X", usb::VENDOR_ID_TRIPPLITE);
        }
        
        // Register Generic fallback protocol
        if (fallback_reg.empty()) {
            ProtocolInfo gen_info;
            gen_info.creator = create_generic_protocol;
            gen_info.name = "Generic HID Protocol";
            gen_info.description = "Universal HID protocol fallback for unknown UPS vendors";
            gen_info.supported_vendors = {};
            gen_info.priority = 10;
            fallback_reg.push_back(gen_info);
            ESP_LOGD(FACTORY_TAG, "Explicitly registered Generic HID fallback protocol");
        }
        
        ESP_LOGD(FACTORY_TAG, "Protocol factory initialized: %zu vendor entries, %zu fallback entries",
                 vendor_reg.size(), fallback_reg.size());
    }
}

void ProtocolFactory::register_protocol_for_vendor(uint16_t vendor_id, 
                                                  const ProtocolInfo& info) {
    ensure_initialized();
    
    auto& registry = get_vendor_registry();
    
    // Check for duplicate registration (can happen when both static initializers
    // and explicit registration in ensure_initialized() succeed)
    auto& entries = registry[vendor_id];
    for (const auto& existing : entries) {
        if (existing.name == info.name) {
            ESP_LOGD(FACTORY_TAG, "Protocol '%s' already registered for vendor 0x%04X, skipping",
                     info.name.c_str(), vendor_id);
            return;
        }
    }
    
    entries.push_back(info);
    
    // Sort by priority (higher first)
    std::sort(entries.begin(), entries.end(),
              [](const ProtocolInfo& a, const ProtocolInfo& b) {
                  return a.priority > b.priority;
              });
    
    ESP_LOGI(FACTORY_TAG, "Registered protocol '%s' for vendor 0x%04X (priority %d)", 
             info.name.c_str(), vendor_id, info.priority);
}

void ProtocolFactory::register_fallback_protocol(const ProtocolInfo& info) {
    ensure_initialized();
    
    auto& registry = get_fallback_registry();
    
    // Check for duplicate registration
    for (const auto& existing : registry) {
        if (existing.name == info.name) {
            ESP_LOGD(FACTORY_TAG, "Fallback protocol '%s' already registered, skipping",
                     info.name.c_str());
            return;
        }
    }
    
    registry.push_back(info);
    
    // Sort by priority (higher first)
    std::sort(registry.begin(), registry.end(),
              [](const ProtocolInfo& a, const ProtocolInfo& b) {
                  return a.priority > b.priority;
              });
    
    ESP_LOGI(FACTORY_TAG, "Registered fallback protocol '%s' (priority %d)", 
             info.name.c_str(), info.priority);
}

std::unique_ptr<UpsProtocolBase> 
ProtocolFactory::create_for_vendor(uint16_t vendor_id, UpsHidComponent* parent) {
    ensure_initialized();
    
    if (!parent) {
        ESP_LOGE(FACTORY_TAG, "Cannot create protocol with null parent component");
        return nullptr;
    }
    
    // Try vendor-specific protocols first
    auto& vendor_registry = get_vendor_registry();
    auto vendor_it = vendor_registry.find(vendor_id);
    
    if (vendor_it != vendor_registry.end()) {
        ESP_LOGD(FACTORY_TAG, "Found %zu vendor-specific protocols for 0x%04X", 
                 vendor_it->second.size(), vendor_id);
        
        for (const auto& info : vendor_it->second) {
            ESP_LOGD(FACTORY_TAG, "Trying vendor protocol '%s' for 0x%04X", 
                     info.name.c_str(), vendor_id);
            
            auto protocol = info.creator(parent);
            if (protocol && protocol->detect()) {
                ESP_LOGI(FACTORY_TAG, "Successfully created protocol '%s' for vendor 0x%04X", 
                         info.name.c_str(), vendor_id);
                return protocol;
            }
        }
    }
    
    // Try fallback protocols
    auto& fallback_registry = get_fallback_registry();
    ESP_LOGD(FACTORY_TAG, "Trying %zu fallback protocols for vendor 0x%04X", 
             fallback_registry.size(), vendor_id);
    
    for (const auto& info : fallback_registry) {
        ESP_LOGD(FACTORY_TAG, "Trying fallback protocol '%s' for 0x%04X", 
                 info.name.c_str(), vendor_id);
        
        auto protocol = info.creator(parent);
        if (protocol && protocol->detect()) {
            ESP_LOGI(FACTORY_TAG, "Successfully created fallback protocol '%s' for vendor 0x%04X", 
                     info.name.c_str(), vendor_id);
            return protocol;
        }
    }
    
    ESP_LOGW(FACTORY_TAG, "No suitable protocol found for vendor 0x%04X", vendor_id);
    return nullptr;
}

std::vector<ProtocolFactory::ProtocolInfo> 
ProtocolFactory::get_protocols_for_vendor(uint16_t vendor_id) {
    ensure_initialized();
    
    std::vector<ProtocolInfo> protocols;
    
    // Add vendor-specific protocols first
    auto& vendor_registry = get_vendor_registry();
    auto vendor_it = vendor_registry.find(vendor_id);
    
    if (vendor_it != vendor_registry.end()) {
        for (const auto& info : vendor_it->second) {
            protocols.push_back(info);
        }
    }
    
    // Add fallback protocols
    auto& fallback_registry = get_fallback_registry();
    for (const auto& info : fallback_registry) {
        protocols.push_back(info);
    }
    
    return protocols;
}

std::vector<std::pair<uint16_t, ProtocolFactory::ProtocolInfo>> 
ProtocolFactory::get_all_protocols() {
    ensure_initialized();
    
    std::vector<std::pair<uint16_t, ProtocolInfo>> all_protocols;
    
    // Add vendor-specific protocols
    auto& vendor_registry = get_vendor_registry();
    for (const auto& vendor_pair : vendor_registry) {
        uint16_t vendor_id = vendor_pair.first;
        for (const auto& info : vendor_pair.second) {
            all_protocols.emplace_back(vendor_id, info);
        }
    }
    
    // Add fallback protocols (use 0x0000 as special vendor ID for fallbacks)
    auto& fallback_registry = get_fallback_registry();
    for (const auto& info : fallback_registry) {
        all_protocols.emplace_back(0x0000, info);
    }
    
    return all_protocols;
}

bool ProtocolFactory::has_vendor_support(uint16_t vendor_id) {
    ensure_initialized();
    
    auto& vendor_registry = get_vendor_registry();
    auto it = vendor_registry.find(vendor_id);
    
    // Has support if vendor-specific protocols exist OR fallback protocols exist
    bool has_vendor_specific = (it != vendor_registry.end() && !it->second.empty());
    bool has_fallback = !get_fallback_registry().empty();
    
    return has_vendor_specific || has_fallback;
}

std::unique_ptr<UpsProtocolBase> 
ProtocolFactory::create_by_name(const std::string& protocol_name, UpsHidComponent* parent) {
    ensure_initialized();
    
    if (!parent) {
        ESP_LOGE(FACTORY_TAG, "Cannot create protocol with null parent component");
        return nullptr;
    }
    
    ESP_LOGD(FACTORY_TAG, "Creating protocol by name: %s", protocol_name.c_str());
    
    // Search through all registered protocols to find one with matching name
    auto& vendor_registry = get_vendor_registry();
    for (const auto& vendor_pair : vendor_registry) {
        for (const auto& info : vendor_pair.second) {
            // Match protocol name (case-insensitive)
            std::string info_name_lower = info.name;
            std::string protocol_name_lower = protocol_name;
            std::transform(info_name_lower.begin(), info_name_lower.end(), info_name_lower.begin(), ::tolower);
            std::transform(protocol_name_lower.begin(), protocol_name_lower.end(), protocol_name_lower.begin(), ::tolower);
            
            if (info_name_lower.find(protocol_name_lower) != std::string::npos) {
                ESP_LOGD(FACTORY_TAG, "Found matching protocol '%s' for name '%s'", 
                         info.name.c_str(), protocol_name.c_str());
                auto protocol = info.creator(parent);
                if (protocol) {
                    ESP_LOGI(FACTORY_TAG, "Successfully created protocol '%s' by name", 
                             protocol->get_protocol_name().c_str());
                    return protocol;
                }
            }
        }
    }
    
    // Search through fallback protocols
    auto& fallback_registry = get_fallback_registry();
    for (const auto& info : fallback_registry) {
        std::string info_name_lower = info.name;
        std::string protocol_name_lower = protocol_name;
        std::transform(info_name_lower.begin(), info_name_lower.end(), info_name_lower.begin(), ::tolower);
        std::transform(protocol_name_lower.begin(), protocol_name_lower.end(), protocol_name_lower.begin(), ::tolower);
        
        if (info_name_lower.find(protocol_name_lower) != std::string::npos) {
            ESP_LOGD(FACTORY_TAG, "Found matching fallback protocol '%s' for name '%s'", 
                     info.name.c_str(), protocol_name.c_str());
            auto protocol = info.creator(parent);
            if (protocol) {
                ESP_LOGI(FACTORY_TAG, "Successfully created fallback protocol '%s' by name", 
                         protocol->get_protocol_name().c_str());
                return protocol;
            }
        }
    }
    
    ESP_LOGE(FACTORY_TAG, "No protocol found with name containing '%s'", protocol_name.c_str());
    return nullptr;
}

} // namespace ups_hid
} // namespace esphome