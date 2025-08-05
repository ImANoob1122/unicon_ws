#pragma once

#include "uniconlink_flex_interfaces/msg/variant_value.hpp"
#include "memory_map.hpp"

namespace uniconlink_flex {

class VariantConverter {
public:
    static uniconlink_flex_interfaces::msg::VariantValue to_msg(const VariantType& variant);
    static VariantType from_msg(const uniconlink_flex_interfaces::msg::VariantValue& msg);
    static std::vector<uniconlink_flex_interfaces::msg::VariantValue> to_msg_vector(const std::vector<VariantType>& variants);
    static std::vector<VariantType> from_msg_vector(const std::vector<uniconlink_flex_interfaces::msg::VariantValue>& msgs);
};

} // namespace uniconlink_flex