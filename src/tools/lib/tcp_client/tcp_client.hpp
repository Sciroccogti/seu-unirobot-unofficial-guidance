#pragma once

#include <boost/asio.hpp>
#include <map>
#include <memory>
#include <thread>
#include "tcp.hpp"

class tcp_client
{
public:
    tcp_client(const std::string &addr, const int &port, tcp_callback tcb = nullptr);
    ~tcp_client();
    void start();
    void stop();
    void write(const tcp_command &cmd);
    void regist(const tcp_cmd_type &type, const tcp_data_dir &dir);
    bool is_connected() const
    {
        return is_connect_;
    }

private:
    void deliver(const tcp_command &cmd);
    void connect();
    void connect_timeout();
    void read_head();
    void read_data();
    boost::asio::ip::tcp::socket socket_;
    std::thread td_;
    std::string addr_;
    int port_;
    bool is_connect_;
    bool is_alive_;
    tcp_callback tcb_;
    boost::asio::deadline_timer timer_;
    tcp_command recv_cmd_;
    char buff_[MAX_CMD_LEN];
    tcp_cmd_type recv_type_;
    bool recv_end_;
    unsigned int recv_size_;
};

