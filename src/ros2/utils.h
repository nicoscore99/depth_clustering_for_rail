// utils.h
#ifndef UTILS_H_
#define UTILS_H_

#include <vector>
#include <cstdint>
#include <algorithm>

template <class T>
T BytesTo(const std::vector<uint8_t>& data, uint32_t start_idx) {
    const size_t kNumberOfBytes = sizeof(T);
    uint8_t byte_array[kNumberOfBytes];
    // forward bit order (it is a HACK. We do not account for bigendianes)
    for (size_t i = 0; i < kNumberOfBytes; ++i) {
        byte_array[i] = data[start_idx + i];
    }
    T result;
    std::copy(reinterpret_cast<const uint8_t*>(&byte_array[0]),
              reinterpret_cast<const uint8_t*>(&byte_array[kNumberOfBytes]),
              reinterpret_cast<uint8_t*>(&result));
    return result;
}

#endif  // UTILS_H_