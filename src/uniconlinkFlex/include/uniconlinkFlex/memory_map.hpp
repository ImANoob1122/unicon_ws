#pragma once

#include <variant>
#include <vector>
#include <map>
#include <functional>
#include <memory>
#include <string>

namespace uniconlink_flex {

using VariantType = std::variant<bool, int32_t, float, std::string>;

enum class AccessMode {
    READ_ONLY,
    WRITE_ONLY,
    READ_WRITE
};

struct MemoryVariable {
    VariantType value;
    AccessMode access_mode;
    std::function<void(const VariantType&)> write_callback;
    std::function<VariantType()> read_callback;
    
    MemoryVariable(const VariantType& initial_value, AccessMode mode,
                   std::function<void(const VariantType&)> write_cb = nullptr,
                   std::function<VariantType()> read_cb = nullptr)
        : value(initial_value), access_mode(mode), write_callback(write_cb), read_callback(read_cb) {}
};

class MemoryMap {
public:
    MemoryMap();
    ~MemoryMap();

    void set_variable(uint32_t index, const VariantType& value, AccessMode mode,
                      std::function<void(const VariantType&)> write_callback = nullptr,
                      std::function<VariantType()> read_callback = nullptr);
    
    bool write_variable(uint32_t index, const VariantType& value);
    
    VariantType read_variable(uint32_t index);
    
    std::vector<VariantType> read_multiple(const std::vector<uint32_t>& indices);
    
    bool write_multiple(const std::vector<uint32_t>& indices, const std::vector<VariantType>& values);
    
    bool is_readable(uint32_t index) const;
    bool is_writable(uint32_t index) const;
    bool exists(uint32_t index) const;
    
    size_t size() const;
    
private:
    std::map<uint32_t, std::unique_ptr<MemoryVariable>> memory_variables_;
};

} // namespace uniconlink_flex