#include "uniconlinkFlex/memory_map.hpp"
#include <stdexcept>

namespace uniconlink_flex {

MemoryMap::MemoryMap() {}

MemoryMap::~MemoryMap() {}

void MemoryMap::set_variable(uint32_t index, const VariantType& value, AccessMode mode,
                             std::function<void(const VariantType&)> write_callback,
                             std::function<VariantType()> read_callback) {
    memory_variables_[index] = std::make_unique<MemoryVariable>(value, mode, write_callback, read_callback);
}

bool MemoryMap::write_variable(uint32_t index, const VariantType& value) {
    auto it = memory_variables_.find(index);
    if (it == memory_variables_.end()) {
        return false;
    }
    
    if (!is_writable(index)) {
        return false;
    }
    
    it->second->value = value;
    
    if (it->second->write_callback) {
        it->second->write_callback(value);
    }
    
    return true;
}

VariantType MemoryMap::read_variable(uint32_t index) {
    auto it = memory_variables_.find(index);
    if (it == memory_variables_.end()) {
        throw std::runtime_error("Variable at index " + std::to_string(index) + " does not exist");
    }
    
    if (!is_readable(index)) {
        throw std::runtime_error("Variable at index " + std::to_string(index) + " is not readable");
    }
    
    if (it->second->read_callback) {
        return it->second->read_callback();
    }
    
    return it->second->value;
}

std::vector<VariantType> MemoryMap::read_multiple(const std::vector<uint32_t>& indices) {
    std::vector<VariantType> result;
    result.reserve(indices.size());
    
    for (uint32_t index : indices) {
        result.push_back(read_variable(index));
    }
    
    return result;
}

bool MemoryMap::write_multiple(const std::vector<uint32_t>& indices, const std::vector<VariantType>& values) {
    if (indices.size() != values.size()) {
        return false;
    }
    
    for (size_t i = 0; i < indices.size(); ++i) {
        if (!write_variable(indices[i], values[i])) {
            return false;
        }
    }
    
    return true;
}

bool MemoryMap::is_readable(uint32_t index) const {
    auto it = memory_variables_.find(index);
    if (it == memory_variables_.end()) {
        return false;
    }
    
    return it->second->access_mode == AccessMode::READ_ONLY || 
           it->second->access_mode == AccessMode::READ_WRITE;
}

bool MemoryMap::is_writable(uint32_t index) const {
    auto it = memory_variables_.find(index);
    if (it == memory_variables_.end()) {
        return false;
    }
    
    return it->second->access_mode == AccessMode::WRITE_ONLY || 
           it->second->access_mode == AccessMode::READ_WRITE;
}

bool MemoryMap::exists(uint32_t index) const {
    return memory_variables_.find(index) != memory_variables_.end();
}

size_t MemoryMap::size() const {
    return memory_variables_.size();
}

} // namespace uniconlink_flex