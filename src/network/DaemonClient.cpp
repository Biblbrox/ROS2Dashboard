#include <filesystem>

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
        std::string error_msg = fmt::format("PID file {} doesn't exists", m_sock);
        Logger::error(error_msg);
        return;
    }

    m_socket = make_shared<asio::local::stream_protocol::socket>(m_IOService);
}

std::string DaemonClient::stateRequest()
{
    std::string request = R"({ "command": "state", "arguments": [] })";
    std::string response = makeRequest(request);
    return response;
}

std::string DaemonClient::topicsRequest()
{
    std::string request = R"({ "command": "topics", "arguments": [] })";
    std::string response = makeRequest(request);
    return response;
}

std::future<std::string> DaemonClient::killNodeRequestAsync(const std::string &node_name)
{
    return std::async(std::launch::async, [this, node_name]() -> std::string {
        std::string kill_node_response = killNodeRequest(node_name);
        stateRequest();
    });
}

std::future<std::string> DaemonClient::stateRequestAsync()
{
    return std::async(std::launch::async, [this]() -> std::string {
        return stateRequest();
    });
}

std::future<std::string> DaemonClient::topicsRequestAsync()
{
    return std::async(std::launch::async, [this]() -> std::string {
        return topicsRequest();
    });
}

DaemonClient::~DaemonClient()
{
    m_socket->close();
}

std::string DaemonClient::killNodeRequest(const std::string &node_name)
{
    std::string request = R"({ "command": "kill_node", "arguments": [{"name": "node_name", "value": ")" + node_name + "\"}]}";
    std::string response = makeRequest(request);
    return stateRequest();
}

std::string DaemonClient::makeRequest(const std::string &request)
{
    error_code connectError;
    if (m_socket->is_open())
        m_socket->close();
    m_socket->connect(m_sock, connectError);
    if (connectError) {
        Logger::error(fmt::format("Can't connect to daemon socket {}", m_sock));
    }

    std::error_code write_error;
    asio::write(*m_socket, asio::buffer(request), write_error);
    /*if (write_error) {
        throw DaemonException(fmt::format("Unable to write message to daemon. Error: {}", write_error.message()));
    }*/

    /// Read header
    uint64_t msg_size = parseHeader();

    /// Read message body
    std::error_code error;
    std::vector<char> body(msg_size);
    size_t read = m_socket->read_some(asio::buffer(body), error);
    if (error)
        throw DaemonException(fmt::format("Unable to read message body data from the deamon. Error: {}", error.message()));

    std::string body_str;
    body_str.resize(body.size());
    std::copy(body.cbegin(), body.cend(), body_str.begin());
    return body_str;
}

uint64_t DaemonClient::parseHeader() const
{
    /// Read header
    std::error_code error;
    const size_t header_size = sizeof(uint64_t);
    std::array<char, header_size> header{};
    asio::read(*m_socket, asio::buffer(header), error, 0);

    if (error && error.value() != 2)
        throw DaemonException(fmt::format("Unable to read message header data from the deamon. Error message: {}. Error code: {}", error.message(), error.value()));

    /// Parse header to extract the message length
    const uint64_t msg_size = u64ToLE(*reinterpret_cast<const uint64_t *>(&header[0]));
    Logger::debug(fmt::format("Receive message with size: {}", msg_size));

    return msg_size;
}


}
