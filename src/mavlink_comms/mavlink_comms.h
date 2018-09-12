#pragma once

#include "standard/mavlink.h"
#include <cstdint>
#include <functional>
#include <thread>
#include <memory>
#include <atomic>
#include <string>
#include <chrono>
#include <map>

class MAVLinkComms {
public:
    bool connect(int udp_port);
    void disconnect();
    bool is_connected();

    void register_message(uint32_t message_id, std::function<void(const mavlink_message_t&)> callback);
    bool send_message(const mavlink_message_t& message);

private:
    void run_recv_thread();
    void run_common_thread();
    void send_heartbeat();

    void parse_datagram(char *buffer, int recv_len);

    int _socket_fd{-1};
    std::string _remote_ip{};
    int _remote_port{0};

    std::atomic<bool> _should_exit{false};
    std::unique_ptr<std::thread> _recv_thread{};
    std::unique_ptr<std::thread> _common_thread{};

    std::map<uint32_t, std::function<void(const mavlink_message_t&)>> _callback_map{};

    std::chrono::time_point<std::chrono::steady_clock> _last_heartbeat_received{};
};

