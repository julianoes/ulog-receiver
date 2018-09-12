
#include <iostream>
#include <thread>
#include <chrono>
#include <arpa/inet.h> // for inet_pton, inet_ntoa
#include <errno.h> // for errno, strerror
#include <unistd.h> // for close()

#include "standard/mavlink.h"
#include "mavlink_comms.h"

bool MAVLinkComms::connect(int udp_port)
{

    _socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

    if (_socket_fd < 0) {
        std::cerr << "socket error" << strerror(errno);
        return false;
    }

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr));
    addr.sin_port = htons(udp_port);

    if (bind(_socket_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        std::cerr << "bind error: " << strerror(errno);
        return false;
    }

    _should_exit = false;
    _recv_thread = std::make_unique<std::thread>(&MAVLinkComms::run_recv_thread, this);
    _common_thread = std::make_unique<std::thread>(&MAVLinkComms::run_common_thread, this);

    return true;
}

void MAVLinkComms::disconnect()
{
    _should_exit = true;

    shutdown(_socket_fd, SHUT_RDWR);
    close(_socket_fd);

    _common_thread->join();
    _common_thread.reset();
    _recv_thread->join();
    _recv_thread.reset();
}

bool MAVLinkComms::is_connected()
{
    auto now = std::chrono::steady_clock::now();

    return ((now - _last_heartbeat_received).count()) * std::chrono::steady_clock::period::num /
           static_cast<double>(std::chrono::steady_clock::period::den) < 3.0;
}

void MAVLinkComms::register_message(uint32_t message_id, std::function<void(const mavlink_message_t&)> callback)
{
    _callback_map[message_id] = callback;
}

bool MAVLinkComms::send_message(const mavlink_message_t& message)
{
    struct sockaddr_in dest_addr {};
    dest_addr.sin_family = AF_INET;

    inet_pton(AF_INET, _remote_ip.c_str(), &dest_addr.sin_addr.s_addr);
    dest_addr.sin_port = htons(_remote_port);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t buffer_len = mavlink_msg_to_send_buffer(buffer, &message);

    int send_len = sendto(_socket_fd,
                          reinterpret_cast<char *>(buffer),
                          buffer_len,
                          0,
                          reinterpret_cast<const sockaddr *>(&dest_addr),
                          sizeof(dest_addr));

    if (send_len != buffer_len) {
        std::cerr << "sendto failure: " << strerror(errno);
        return false;
    }
    return true;
}

void MAVLinkComms::run_recv_thread()
{
    // Enough for MTU 1500 bytes.
    char buffer[2048];

    while (!_should_exit) {
        struct sockaddr_in src_addr = {};
        socklen_t src_addr_len = sizeof(src_addr);

        int recv_len = recvfrom(_socket_fd,
                                buffer,
                                sizeof(buffer),
                                0,
                                reinterpret_cast<struct sockaddr *>(&src_addr),
                                &src_addr_len);

        if (recv_len == 0) {
            // This can happen when shutdown is called on the socket,
            // therefore we check _should_exit again.
            continue;
        }

        if (recv_len < 0) {
            // This happens on desctruction when close(_socket_fd) is called,
            // therefore be quiet.
            continue;
        }

        _remote_ip = inet_ntoa(src_addr.sin_addr);
        _remote_port = ntohs(src_addr.sin_port);

        parse_datagram(buffer, recv_len);
    }
}

void MAVLinkComms::parse_datagram(char *buffer, int recv_len)
{
    mavlink_status_t status;
    mavlink_message_t message;

    // Note that one datagram can contain multiple mavlink messages.
    for (int i = 0; i < recv_len; ++i) {
        if (mavlink_parse_char(0, buffer[i], &message, &status) == 1) {
            // Check if someone needs this message.
            auto it = _callback_map.find(message.msgid);
            if (it != _callback_map.end()) {
                if (it->second) {
                    it->second(message);
                }
            }

            if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                _last_heartbeat_received = std::chrono::steady_clock::now();
            }
        }
    }
}

void MAVLinkComms::run_common_thread()
{
    while (!_should_exit) {
        send_heartbeat();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void MAVLinkComms::send_heartbeat()
{
    if (!_remote_ip.empty() && _remote_port != 0) {
        mavlink_message_t message;
        mavlink_msg_heartbeat_pack(1,
                                   MAV_COMP_ID_SYSTEM_CONTROL+1, // +1 is a workaround to avoid a conflict
                                   &message,
                                   MAV_TYPE_GCS,
                                   MAV_AUTOPILOT_INVALID,
                                   0,
                                   0,
                                   0);
        send_message(message);
    }
}
