#pragma once

#include <QObject>
#include <asio.hpp>
#include <asio/io_service.hpp>
#include <string>

#include "ros2_entities/Ros2State.hpp"

namespace ros2monitor {
class DaemonClient : public QObject {
    Q_OBJECT
    using Socket = asio::local::stream_protocol::socket;

public:
    explicit DaemonClient(std::string pid);
    ~DaemonClient();

    /**
    * Receive ros2 state from daemon.
         * @return
         */
    void receiveState();

    void killNode(const std::string& node_name);

    /*
         * Find Ros2 entities in filesystem
         * It is supposed that user has activated ros2 workspace with .install/setup.sh script
         */
    Ros2State discoverFilesystem() const;

signals:
    void hotStateUpdated(const QString &jsonState);

private:
    uint64_t parseHeader() const;

    void stateRequest();
    void topicsRequest();

    std::string m_sock;
    asio::io_service m_IOService;
    asio::thread_pool m_context;
    std::shared_ptr<Socket> m_socket;
};

}