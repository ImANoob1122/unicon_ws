#include "uniconlinkFlex/variant_converter.hpp"

namespace uniconlink_flex {

uniconlink_flex_interfaces::msg::VariantValue VariantConverter::to_msg(const VariantType& variant) {
    uniconlink_flex_interfaces::msg::VariantValue msg;
    
    if (std::holds_alternative<bool>(variant)) {
        msg.type = uniconlink_flex_interfaces::msg::VariantValue::TYPE_BOOL;
        msg.bool_value = std::get<bool>(variant);
    } else if (std::holds_alternative<int32_t>(variant)) {
        msg.type = uniconlink_flex_interfaces::msg::VariantValue::TYPE_INT;
        msg.int_value = std::get<int32_t>(variant);
    } else if (std::holds_alternative<float>(variant)) {
        msg.type = uniconlink_flex_interfaces::msg::VariantValue::TYPE_FLOAT;
        msg.float_value = std::get<float>(variant);
    } else if (std::holds_alternative<std::string>(variant)) {
        msg.type = uniconlink_flex_interfaces::msg::VariantValue::TYPE_STRING;
        msg.string_value = std::get<std::string>(variant);
    } else {
        msg.type = uniconlink_flex_interfaces::msg::VariantValue::TYPE_NONE;
    }
    
    return msg;
}

VariantType VariantConverter::from_msg(const uniconlink_flex_interfaces::msg::VariantValue& msg) {
    switch (msg.type) {
        case uniconlink_flex_interfaces::msg::VariantValue::TYPE_BOOL:
            return msg.bool_value;
        case uniconlink_flex_interfaces::msg::VariantValue::TYPE_INT:
            return msg.int_value;
        case uniconlink_flex_interfaces::msg::VariantValue::TYPE_FLOAT:
            return msg.float_value;
        case uniconlink_flex_interfaces::msg::VariantValue::TYPE_STRING:
            return msg.string_value;
        default:
            return false; // Default to bool false for invalid types
    }
}

std::vector<uniconlink_flex_interfaces::msg::VariantValue> VariantConverter::to_msg_vector(const std::vector<VariantType>& variants) {
    std::vector<uniconlink_flex_interfaces::msg::VariantValue> result;
    result.reserve(variants.size());
    
    for (const auto& variant : variants) {
        result.push_back(to_msg(variant));
    }
    
    return result;
}

std::vector<VariantType> VariantConverter::from_msg_vector(const std::vector<uniconlink_flex_interfaces::msg::VariantValue>& msgs) {
    std::vector<VariantType> result;
    result.reserve(msgs.size());
    
    for (const auto& msg : msgs) {
        result.push_back(from_msg(msg));
    }
    
    return result;
}

} // namespace uniconlink_flex