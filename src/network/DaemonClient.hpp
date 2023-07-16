#pragma once


#include <asio.hpp>
#include <asio/io_service.hpp>
#include <string>

#include "ros2_entities/Ros2State.hpp"

namespace ros2monitor {
class DaemonClient {
    using Socket = asio::local::stream_protocol::socket;

public:
    explicit DaemonClient(std::string pid);
    ~DaemonClient();

    /**
    * Receive ros2 state from daemon.
         * @return
         */
    std::string killNodeRequest(const std::string &node_name);
    std::string stateRequest();
    std::string topicsRequest();
    std::future<std::string> killNodeRequestAsync(const std::string &node_name);
    std::future<std::string> stateRequestAsync();
    std::future<std::string> topicsRequestAsync();

    /*
         * Find Ros2 entities in filesystem
         * It is supposed that user has activated ros2 workspace with .install/setup.sh script
         */
    Ros2State discoverFilesystem() const;


private:
    uint64_t parseHeader() const;

    std::string makeRequest(const std::string &request);

    std::string m_sock;
    asio::io_service m_IOService;
    asio::thread_pool m_context;
    std::shared_ptr<Socket> m_socket;
};

}