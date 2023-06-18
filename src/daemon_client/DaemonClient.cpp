#include <filesystem>


#include "DaemonClient.hpp"
#include "core/Logger.hpp"
#include "network/Bytes.hpp"


namespace ros2monitor {
using asio::error_code;
using std::make_shared;

DaemonClient::DaemonClient(std::string pid) : m_sock(std::move(pid))
{
    if (!std::filesystem::exists(m_sock)) {
        Logger::error(fmt::format("PID file {} doesn't exists", m_sock));
    }

    m_socket = make_shared<asio::local::stream_protocol::socket>(m_IOService);

    error_code connectError;
    m_socket->connect(m_sock, connectError);
    if (connectError) {
        Logger::error(fmt::format("Can't connect to daemon socket {}", m_sock));
    }
}

void DaemonClient::receiveState()
{
    asio::write(*m_socket, asio::buffer("state"));
    std::error_code error;
    /// Read header
    const size_t header_size = sizeof(uint64_t);
    std::array<char, header_size> header{};
    asio::read(*m_socket, asio::buffer(header), error, 0);

    /// Parse header to extract the message length
    const uint64_t msg_size = u64ToLE(*reinterpret_cast<const uint64_t *>(&header[0]));
    Logger::debug(fmt::format("Receive message with size: {}", msg_size));

    /// Read message body
    std::vector<char> body(msg_size);
    size_t read = m_socket->read_some(asio::buffer(body), error);

    m_socket->close();

    emit hotStateUpdated(body.data());
}


DaemonClient::~DaemonClient()
{
    m_socket->close();
}
}
