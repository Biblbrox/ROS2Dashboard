#include "DaemonException.hpp"

namespace ros2monitor {

const char *DaemonException::what() const noexcept
{
    return m_msg.c_str();
}
DaemonException::DaemonException(std::string msg)
{
    m_msg = std::move(msg);
}
}