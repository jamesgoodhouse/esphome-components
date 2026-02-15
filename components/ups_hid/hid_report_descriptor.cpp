#include "hid_report_descriptor.h"
#include "constants_hid.h"
#include "esphome/core/log.h"
#include <cmath>
#include <algorithm>

namespace esphome {
namespace ups_hid {

static const char* const TAG = "hid_report_map";

// =============================================================================
// HID Item Parsing Helpers
// =============================================================================
//
// HID short item prefix byte: [bTag:4][bType:2][bSize:2]
//   bSize: 0=0 bytes, 1=1 byte, 2=2 bytes, 3=4 bytes
//   bType: 0=Main, 1=Global, 2=Local, 3=Reserved
//   bTag:  meaning depends on bType
//
// We parse by extracting bTag and bType independently, then switching on bTag
// within each bType category. This avoids the pitfall of matching on the full
// prefix byte (which includes size bits that vary with payload length).

// Global item tags (bType=1)
// Ref: HID spec section 6.2.2.7
constexpr uint8_t GTAG_USAGE_PAGE     = 0x0;
constexpr uint8_t GTAG_LOGICAL_MIN    = 0x1;
constexpr uint8_t GTAG_LOGICAL_MAX    = 0x2;
constexpr uint8_t GTAG_PHYSICAL_MIN   = 0x3;
constexpr uint8_t GTAG_PHYSICAL_MAX   = 0x4;
constexpr uint8_t GTAG_UNIT_EXPONENT  = 0x5;
constexpr uint8_t GTAG_UNIT           = 0x6;
constexpr uint8_t GTAG_REPORT_SIZE    = 0x7;
constexpr uint8_t GTAG_REPORT_ID      = 0x8;
constexpr uint8_t GTAG_REPORT_COUNT   = 0x9;
constexpr uint8_t GTAG_PUSH           = 0xA;
constexpr uint8_t GTAG_POP            = 0xB;

// Local item tags (bType=2)
// Ref: HID spec section 6.2.2.8
constexpr uint8_t LTAG_USAGE          = 0x0;
constexpr uint8_t LTAG_USAGE_MIN      = 0x1;
constexpr uint8_t LTAG_USAGE_MAX      = 0x2;

// Main item tags (bType=0)
// Ref: HID spec section 6.2.2.4-6
constexpr uint8_t MTAG_INPUT          = 0x8;
constexpr uint8_t MTAG_OUTPUT         = 0x9;
constexpr uint8_t MTAG_COLLECTION     = 0xA;
constexpr uint8_t MTAG_FEATURE        = 0xB;
constexpr uint8_t MTAG_END_COLLECTION = 0xC;

// Limits for ESP32 memory
constexpr size_t MAX_STACK_DEPTH      = 8;
constexpr size_t MAX_COLLECTION_DEPTH = 8;
constexpr size_t MAX_USAGE_LIST       = 64;

// =============================================================================
// Parser Internal State
// =============================================================================

struct GlobalState {
  uint16_t usage_page{0};
  int32_t logical_min{0};
  int32_t logical_max{0};
  int32_t physical_min{0};
  int32_t physical_max{0};
  int8_t unit_exponent{0};
  uint32_t unit{0};
  uint8_t report_size{0};
  uint16_t report_count{0};
  uint8_t report_id{0};
};

struct ParserState {
  const uint8_t* data{nullptr};
  size_t length{0};
  size_t offset{0};
  GlobalState global;
  std::vector<GlobalState> stack;
  std::vector<uint32_t> collection_path;
  std::vector<uint32_t> usage_list;
  uint32_t usage_min{0};  // For Usage Min/Max range expansion
  bool has_usage_min{false};
  bool has_report_id{false};
  // Per (report_id, report_type) bit offset tracking
  // Key: (report_id << 8) | report_type
  std::map<uint16_t, uint16_t> bit_offsets;
};

// =============================================================================
// Low-level item readers
// =============================================================================

static int32_t read_signed(const uint8_t* data, size_t size) {
  if (size == 0) return 0;
  if (size == 1) return static_cast<int32_t>(static_cast<int8_t>(data[0]));
  if (size == 2) return static_cast<int32_t>(static_cast<int16_t>(data[0] | (data[1] << 8)));
  if (size == 4) {
    return static_cast<int32_t>(
        data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
  }
  return 0;
}

static uint32_t read_unsigned(const uint8_t* data, size_t size) {
  if (size == 0) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < size && i < 4; i++) {
    v |= static_cast<uint32_t>(data[i]) << (i * 8);
  }
  return v;
}

static uint8_t get_item_size(uint8_t prefix) {
  uint8_t s = prefix & 0x03;
  return (s == 3) ? 4 : s;
}

static uint8_t get_item_type(uint8_t prefix) {
  return (prefix >> 2) & 0x03;
}

static uint8_t get_item_tag(uint8_t prefix) {
  return (prefix >> 4) & 0x0F;
}

/// Build a full 32-bit usage from a local Usage item's data.
/// If item_size >= 4, the value is an extended usage (page in high 16, id in low 16).
/// Otherwise, the current global usage page is prepended.
static uint32_t make_usage(uint16_t global_page, const uint8_t* data, size_t item_size) {
  uint32_t raw = read_unsigned(data, item_size);
  if (item_size >= 4) {
    return raw;  // Extended usage: page already in high 16 bits
  }
  return (static_cast<uint32_t>(global_page) << 16) | (raw & 0xFFFF);
}

/// Get the bit offset for a (report_id, report_type) pair, initializing if needed.
static uint16_t get_bit_offset(ParserState& state, uint8_t report_id, uint8_t report_type) {
  uint16_t key = (static_cast<uint16_t>(report_id) << 8) | report_type;
  auto it = state.bit_offsets.find(key);
  if (it != state.bit_offsets.end()) {
    return it->second;
  }
  // First field for this (report_id, report_type) pair
  uint16_t initial = state.has_report_id ? 8 : 0;  // Skip report ID byte if present
  state.bit_offsets[key] = initial;
  return initial;
}

/// Advance the bit offset for a (report_id, report_type) pair.
static void advance_bit_offset(ParserState& state, uint8_t report_id, uint8_t report_type, uint16_t bits) {
  uint16_t key = (static_cast<uint16_t>(report_id) << 8) | report_type;
  state.bit_offsets[key] += bits;
}

// =============================================================================
// HidReportMap::parse
// =============================================================================

bool HidReportMap::parse(const uint8_t* descriptor, size_t length) {
  fields_.clear();

  if (!descriptor || length == 0) {
    ESP_LOGW(TAG, "Empty or null descriptor");
    return false;
  }

  ESP_LOGD(TAG, "Parsing HID report descriptor (%zu bytes)", length);

  ParserState state;
  state.data = descriptor;
  state.length = length;
  state.stack.reserve(MAX_STACK_DEPTH);
  state.collection_path.reserve(MAX_COLLECTION_DEPTH);
  state.usage_list.reserve(MAX_USAGE_LIST);

  while (state.offset < state.length) {
    uint8_t prefix = state.data[state.offset];
    state.offset++;

    // Long item: prefix has tag nibble = 0xF
    // Format: prefix(0xFE) | bDataSize(1) | bLongItemTag(1) | data[bDataSize]
    if (get_item_tag(prefix) == 0x0F) {
      if (state.offset >= state.length) {
        ESP_LOGW(TAG, "Truncated long item at offset %zu", state.offset - 1);
        break;
      }
      uint8_t data_size = state.data[state.offset];
      state.offset++;  // skip bDataSize
      size_t skip = 1u + data_size;  // bLongItemTag + data
      if (state.offset + skip > state.length) {
        ESP_LOGW(TAG, "Truncated long item data at offset %zu", state.offset);
        break;
      }
      state.offset += skip;
      ESP_LOGD(TAG, "Skipped long item (%u bytes)", data_size);
      continue;
    }

    uint8_t item_size = get_item_size(prefix);
    uint8_t item_type = get_item_type(prefix);
    uint8_t item_tag  = get_item_tag(prefix);

    if (state.offset + item_size > state.length) {
      ESP_LOGW(TAG, "Descriptor truncated at offset %zu (need %u more bytes)",
               state.offset, item_size);
      break;
    }

    const uint8_t* item_data = state.data + state.offset;
    state.offset += item_size;

    // -----------------------------------------------------------------
    // Global items (bType = 1)
    // -----------------------------------------------------------------
    if (item_type == 1) {
      switch (item_tag) {
        case GTAG_USAGE_PAGE:
          state.global.usage_page = static_cast<uint16_t>(read_unsigned(item_data, item_size));
          break;
        case GTAG_LOGICAL_MIN:
          state.global.logical_min = read_signed(item_data, item_size);
          break;
        case GTAG_LOGICAL_MAX:
          state.global.logical_max = read_signed(item_data, item_size);
          break;
        case GTAG_PHYSICAL_MIN:
          state.global.physical_min = read_signed(item_data, item_size);
          break;
        case GTAG_PHYSICAL_MAX:
          state.global.physical_max = read_signed(item_data, item_size);
          break;
        case GTAG_UNIT_EXPONENT: {
          // Unit exponent is a 4-bit signed value (nibble) per HID spec
          int32_t raw = read_signed(item_data, item_size);
          // HID spec: values 0x0-0x7 = 0 to 7, 0x8-0xF = -8 to -1
          if (raw >= 0x8 && raw <= 0xF) {
            state.global.unit_exponent = static_cast<int8_t>(raw - 16);
          } else {
            state.global.unit_exponent = static_cast<int8_t>(raw);
          }
          break;
        }
        case GTAG_UNIT:
          state.global.unit = read_unsigned(item_data, item_size);
          break;
        case GTAG_REPORT_SIZE:
          state.global.report_size = static_cast<uint8_t>(read_unsigned(item_data, item_size));
          break;
        case GTAG_REPORT_ID:
          state.global.report_id = static_cast<uint8_t>(read_unsigned(item_data, item_size));
          state.has_report_id = true;
          break;
        case GTAG_REPORT_COUNT:
          state.global.report_count = static_cast<uint16_t>(read_unsigned(item_data, item_size));
          break;
        case GTAG_PUSH:
          if (state.stack.size() < MAX_STACK_DEPTH) {
            state.stack.push_back(state.global);
          } else {
            ESP_LOGW(TAG, "Global state stack overflow (max %zu)", MAX_STACK_DEPTH);
          }
          break;
        case GTAG_POP:
          if (!state.stack.empty()) {
            state.global = state.stack.back();
            state.stack.pop_back();
          } else {
            ESP_LOGW(TAG, "Global state stack underflow");
          }
          break;
        default:
          ESP_LOGD(TAG, "Unknown global item tag 0x%X", item_tag);
          break;
      }
      continue;
    }

    // -----------------------------------------------------------------
    // Local items (bType = 2)
    // -----------------------------------------------------------------
    if (item_type == 2) {
      switch (item_tag) {
        case LTAG_USAGE:
          if (state.usage_list.size() < MAX_USAGE_LIST) {
            state.usage_list.push_back(make_usage(state.global.usage_page, item_data, item_size));
          }
          break;
        case LTAG_USAGE_MIN:
          state.usage_min = make_usage(state.global.usage_page, item_data, item_size);
          state.has_usage_min = true;
          break;
        case LTAG_USAGE_MAX: {
          uint32_t usage_max = make_usage(state.global.usage_page, item_data, item_size);
          if (state.has_usage_min) {
            // Expand the range into individual usages
            uint16_t min_id = state.usage_min & 0xFFFF;
            uint16_t max_id = usage_max & 0xFFFF;
            uint32_t page = state.usage_min & 0xFFFF0000;
            state.usage_list.clear();
            for (uint32_t u = min_id; u <= max_id && state.usage_list.size() < MAX_USAGE_LIST; u++) {
              state.usage_list.push_back(page | u);
            }
            state.has_usage_min = false;
          } else {
            state.usage_list.push_back(usage_max);
          }
          break;
        }
        default:
          // Other local items (Designator, String, Delimiter) - ignore for UPS
          break;
      }
      continue;
    }

    // -----------------------------------------------------------------
    // Main items (bType = 0)
    // -----------------------------------------------------------------
    if (item_type == 0) {
      if (item_tag == MTAG_COLLECTION) {
        // Collection start - push collection usage onto path
        if (state.collection_path.size() < MAX_COLLECTION_DEPTH) {
          uint32_t coll_usage = state.usage_list.empty()
              ? (static_cast<uint32_t>(state.global.usage_page) << 16)
              : state.usage_list[0];
          state.collection_path.push_back(coll_usage);
        }
        // Local state is consumed by Main item
        state.usage_list.clear();
        state.has_usage_min = false;
        continue;
      }

      if (item_tag == MTAG_END_COLLECTION) {
        if (!state.collection_path.empty()) {
          state.collection_path.pop_back();
        }
        // Local state is consumed by Main item
        state.usage_list.clear();
        state.has_usage_min = false;
        continue;
      }

      // Input, Output, or Feature
      uint8_t report_type = 0;
      if (item_tag == MTAG_INPUT)        report_type = HID_REPORT_TYPE_INPUT;
      else if (item_tag == MTAG_OUTPUT)  report_type = HID_REPORT_TYPE_OUTPUT;
      else if (item_tag == MTAG_FEATURE) report_type = HID_REPORT_TYPE_FEATURE;
      else {
        // Unknown main item tag - skip
        state.usage_list.clear();
        state.has_usage_min = false;
        continue;
      }

      uint16_t report_size = state.global.report_size;
      uint16_t report_count = state.global.report_count;
      if (report_size == 0 || report_count == 0) {
        state.usage_list.clear();
        state.has_usage_min = false;
        continue;
      }

      uint8_t rid = state.global.report_id;
      uint16_t bit_offset = get_bit_offset(state, rid, report_type);

      for (uint16_t i = 0; i < report_count; i++) {
        HidField field;
        field.report_id = rid;
        field.report_type = report_type;
        field.usage = (i < state.usage_list.size())
            ? state.usage_list[i]
            : (state.usage_list.empty()
                ? (static_cast<uint32_t>(state.global.usage_page) << 16)
                : state.usage_list.back());
        field.usage_path = state.collection_path;
        field.bit_offset = bit_offset + (i * report_size);
        field.bit_size = report_size;
        field.logical_min = state.global.logical_min;
        field.logical_max = state.global.logical_max;
        field.physical_min = state.global.physical_min;
        field.physical_max = state.global.physical_max;
        field.unit_exponent = state.global.unit_exponent;
        field.unit = state.global.unit;
        fields_.push_back(field);
      }

      // Advance bit offset for this (report_id, report_type)
      uint16_t total_bits = report_count * report_size;
      uint16_t key = (static_cast<uint16_t>(rid) << 8) | report_type;
      state.bit_offsets[key] = bit_offset + total_bits;

      // Local state is consumed by Main item
      state.usage_list.clear();
      state.has_usage_min = false;
    }
  }

  ESP_LOGI(TAG, "Parsed %zu fields from %zu-byte descriptor", fields_.size(), length);
  return !fields_.empty();
}

// =============================================================================
// Field lookup methods
// =============================================================================

const HidField* HidReportMap::find_field_by_usage(uint32_t usage) const {
  for (const auto& f : fields_) {
    if (f.usage == usage) return &f;
  }
  return nullptr;
}

const HidField* HidReportMap::find_field_by_usage_path(
    const std::vector<uint32_t>& path, uint32_t usage) const {
  for (const auto& f : fields_) {
    if (f.usage != usage) continue;
    if (f.usage_path.size() < path.size()) continue;
    // Check if path is a prefix of (or exactly matches) the field's path
    bool match = true;
    for (size_t i = 0; i < path.size(); i++) {
      if (f.usage_path[i] != path[i]) {
        match = false;
        break;
      }
    }
    if (match) return &f;
  }
  return nullptr;
}

std::vector<const HidField*> HidReportMap::get_fields_for_report(
    uint8_t report_id, uint8_t report_type) const {
  std::vector<const HidField*> result;
  for (const auto& f : fields_) {
    if (f.report_id == report_id && f.report_type == report_type) {
      result.push_back(&f);
    }
  }
  return result;
}

std::set<uint8_t> HidReportMap::get_report_ids() const {
  std::set<uint8_t> ids;
  for (const auto& f : fields_) {
    ids.insert(f.report_id);
  }
  return ids;
}

size_t HidReportMap::get_report_size_bytes(uint8_t report_id, uint8_t report_type) const {
  size_t max_bits = 0;
  for (const auto& f : fields_) {
    if (f.report_id == report_id && f.report_type == report_type) {
      size_t end = static_cast<size_t>(f.bit_offset) + f.bit_size;
      if (end > max_bits) max_bits = end;
    }
  }
  if (max_bits == 0) return 0;
  // bit_offset already accounts for the report ID byte (starts at bit 8),
  // so no additional adjustment needed.
  return (max_bits + 7) / 8;
}

// =============================================================================
// Value extraction
// =============================================================================

float HidReportMap::extract_field_value(
    const HidField& field, const uint8_t* report_data, size_t report_len) const {
  if (!report_data || report_len == 0) return NAN;

  size_t bit_end = static_cast<size_t>(field.bit_offset) + field.bit_size;
  size_t byte_end = (bit_end + 7) / 8;
  if (byte_end > report_len) {
    ESP_LOGW(TAG, "Report too short for field: need %zu bytes, have %zu", byte_end, report_len);
    return NAN;
  }

  // Extract raw bits (little-endian bit ordering)
  int64_t raw = 0;
  for (uint16_t b = 0; b < field.bit_size; b++) {
    size_t bit_idx = field.bit_offset + b;
    size_t byte_idx = bit_idx / 8;
    uint8_t bit_in_byte = bit_idx % 8;
    if (report_data[byte_idx] & (1u << bit_in_byte)) {
      raw |= (1LL << b);
    }
  }

  // Sign-extend if the logical range is signed
  bool is_signed = (field.logical_min < 0);
  if (is_signed && field.bit_size > 0 && field.bit_size < 64) {
    int64_t sign_bit = 1LL << (field.bit_size - 1);
    if (raw & sign_bit) {
      raw |= ~((1LL << field.bit_size) - 1);
    }
  }

  int64_t logical_val = raw;

  // Clamp to logical range (skip if range is zero/undefined)
  if (field.logical_min != field.logical_max) {
    if (logical_val < field.logical_min) logical_val = field.logical_min;
    if (logical_val > field.logical_max) logical_val = field.logical_max;
  }

  float result;

  // Physical conversion
  if (field.physical_min == field.physical_max) {
    // No physical range defined: use logical value directly
    result = static_cast<float>(logical_val);
  } else {
    int64_t logical_range = static_cast<int64_t>(field.logical_max) - field.logical_min;
    if (logical_range == 0) {
      result = static_cast<float>(field.physical_min);
    } else {
      float physical_range = static_cast<float>(field.physical_max - field.physical_min);
      result = static_cast<float>(logical_val - field.logical_min) * physical_range /
                   static_cast<float>(logical_range) +
               static_cast<float>(field.physical_min);
    }
  }

  // Apply unit exponent: multiply by 10^exponent
  if (field.unit_exponent != 0) {
    result *= std::pow(10.0f, static_cast<float>(field.unit_exponent));
  }

  return result;
}

// =============================================================================
// Debug dump
// =============================================================================

static const char* report_type_name(uint8_t rt) {
  switch (rt) {
    case HID_REPORT_TYPE_INPUT:   return "Input";
    case HID_REPORT_TYPE_OUTPUT:  return "Output";
    case HID_REPORT_TYPE_FEATURE: return "Feature";
    default:                      return "Unknown";
  }
}

static const char* usage_page_name(uint16_t page) {
  switch (page) {
    case 0x84: return "PowerDevice";
    case 0x85: return "BatterySystem";
    case 0x01: return "GenericDesktop";
    default:   return "Unknown";
  }
}

void HidReportMap::dump(const char* tag) const {
  const char* t = tag ? tag : TAG;
  ESP_LOGI(t, "=== HID Report Map: %zu fields ===", fields_.size());

  auto ids = get_report_ids();
  ESP_LOGI(t, "Report IDs: %zu unique", ids.size());
  for (uint8_t id : ids) {
    size_t feat_sz = get_report_size_bytes(id, HID_REPORT_TYPE_FEATURE);
    size_t inp_sz  = get_report_size_bytes(id, HID_REPORT_TYPE_INPUT);
    if (feat_sz > 0) {
      ESP_LOGI(t, "  Report 0x%02X: Feature %zu bytes", id, feat_sz);
    }
    if (inp_sz > 0) {
      ESP_LOGI(t, "  Report 0x%02X: Input %zu bytes", id, inp_sz);
    }
  }

  for (size_t i = 0; i < fields_.size(); i++) {
    const HidField& f = fields_[i];
    uint16_t page = (f.usage >> 16) & 0xFFFF;
    uint16_t uid  = f.usage & 0xFFFF;
    ESP_LOGI(t,
             "  [%zu] RID=0x%02X %s  Usage=%s:0x%04X  bits=%u@%u  "
             "log=[%ld..%ld]  phys=[%ld..%ld]  exp=%d",
             i, f.report_id, report_type_name(f.report_type),
             usage_page_name(page), uid,
             f.bit_size, f.bit_offset,
             (long)f.logical_min, (long)f.logical_max,
             (long)f.physical_min, (long)f.physical_max,
             f.unit_exponent);
  }
  ESP_LOGI(t, "=== End HID Report Map ===");
}

}  // namespace ups_hid
}  // namespace esphome
