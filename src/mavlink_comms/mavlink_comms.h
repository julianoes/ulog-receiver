#pragma once

#include "standard/mavlink.h"
#include <cstdint>
#include <functional>
#include <thread>
#include <memory>
#include <atomic>
#include <string>

class MAVLinkComms {
public:
    bool connect(int udp_port);
    void disconnect();

    void register_message(uint16_t message_id, std::function<void(const mavlink_message_t&)> callback);
    bool send_message(const mavlink_message_t& message);

private:
    void run_recv_thread();
    void run_heartbeat_thread();

    void parse_datagram(char *buffer, int recv_len);

    int _socket_fd{-1};
    std::string _remote_ip{};
    int _remote_port{0};

    std::atomic<bool> _should_exit{false};
    std::unique_ptr<std::thread> _recv_thread{};
    std::unique_ptr<std::thread> _heartbeat_thread{};
};

