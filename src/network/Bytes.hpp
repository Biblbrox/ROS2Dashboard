#pragma once

#include <cstdint>

namespace ros2monitor {
/**
 * Platform independent conversation to LE.
 * @return
 */
uint64_t u64ToLE(uint64_t);
uint32_t u32ToLE(uint32_t);
}