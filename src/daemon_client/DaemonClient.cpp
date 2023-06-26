#include <filesystem>
#include <nlohmann/json.hpp>


#include "DaemonClient.hpp"
#include "core/DaemonException.hpp"
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

uint64_t DaemonClient::parseHeader() const
{
    /// Read header
    std::error_code error;
    const size_t header_size = sizeof(uint64_t);
    std::array<char, header_size> header{};
    asio::read(*m_socket, asio::buffer(header), error, 0);

    if (error)
        throw DaemonException("Unable to read message header data from the deamon");

    /// Parse header to extract the message length
    const uint64_t msg_size = u64ToLE(*reinterpret_cast<const uint64_t *>(&header[0]));
    Logger::debug(fmt::format("Receive message with size: {}", msg_size));

    return msg_size;
}

void DaemonClient::receiveState()
{
    stateRequest();
    //topicsRequest();
}

void DaemonClient::stateRequest()
{
    using json = nlohmann::json;
    // Create json request
    std::string request = "{ \"command\": \"state\", \"arguments\": [] }";

    std::error_code write_error;
    asio::write(*m_socket, asio::buffer(request), write_error);
    if (write_error) {
        throw DaemonException("Unable to write message to daemon");
    }

    /// Read header
    size_t msg_size = parseHeader();

    /// Read message body
    std::error_code error;
    std::vector<char> body(msg_size);
    size_t read = m_socket->read_some(asio::buffer(body), error);
    if (error)
        throw DaemonException("Unable to read message body data from the deamon");

    QString body_str;
    body_str.resize(body.size());
    std::copy(body.cbegin(), body.cend(), body_str.begin());
    emit hotStateUpdated(body_str);
}

void DaemonClient::topicsRequest()
{
    using json = nlohmann::json;
    // Create json request
    std::string request = R"({ "command": "topics", "arguments": [] })";

    asio::write(*m_socket, asio::buffer(request));

    /// Read header
    size_t msg_size = parseHeader();

    /// Read message body
    std::error_code error;
    std::vector<char> body(msg_size);
    size_t read = m_socket->read_some(asio::buffer(body), error);
    if (error)
        throw DaemonException("Unable to read message body data from the deamon");

    QString body_str = QString::fromStdString(body.data());
    emit hotStateUpdated(body_str);
}


DaemonClient::~DaemonClient()
{
    m_socket->close();
}

void DaemonClient::killNode(const std::string &node_name)
{
}

}
