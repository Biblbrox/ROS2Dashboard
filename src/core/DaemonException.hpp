#pragma once


#include <exception>
#include <string>

namespace ros2monitor {

class DaemonException : public std::exception {
public:
    explicit DaemonException(std::string msg);

    const char * what() const noexcept override;

private:
    std::string m_msg;
};

}
