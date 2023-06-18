#include "Bytes.hpp"


uint64_t ros2monitor::u64ToLE(uint64_t val)
{
    return __builtin_bswap64(val);
}
