#include "tcp_client.hpp"
#include <string>
#include <iostream>
#include <functional>

using boost::asio::ip::tcp;
using namespace std;

boost::asio::io_service tcp_service;

tcp_client::tcp_client(const std::string &addr, const int &port, tcp_callback tcb)
    : port_(port), addr_(addr), socket_(tcp_service), timer_(tcp_service), tcb_(tcb)
{
    is_connect_ = false;
    timer_.expires_at(boost::posix_time::pos_infin);
}

void tcp_client::write(const tcp_command &cmd)
{
    unsigned int t_size = cmd.size;
    int i = 0;
    tcp_command temp;
    temp.type = cmd.type;

    while (t_size > max_data_size)
    {
        temp.end = false;
        temp.size = max_data_size;
        temp.data.assign((char *)(cmd.data.c_str() + i * max_data_size), max_data_size);
        deliver(temp);
        t_size -= max_data_size;
        i++;
        usleep(10);
    }

    temp.size = t_size;
    temp.end = true;
    temp.data.assign((char *)(cmd.data.c_str() + i * max_data_size), t_size);
    deliver(temp);
}

void tcp_client::regist(const tcp_cmd_type &type, const tcp_data_dir &dir)
{
    tcp_command cmd;
    cmd.type = REG_DATA;
    cmd.size = enum_size + enum_size;
    cmd.data.clear();
    cmd.data.append((char *)&type, enum_size);
    cmd.data.append((char *)&dir, enum_size);
    this->write(cmd);
}

void tcp_client::deliver(const tcp_command &cmd)
{
    if (!is_connect_)
    {
        return;
    }

    boost::system::error_code ec;
    unsigned int data_size = cmd.size;
    unsigned int total_size = data_size + data_offset;
    char buf[MAX_CMD_LEN];
    unsigned int offset = 0;
    memcpy(buf + offset, &(cmd.type), enum_size);
    offset += enum_size;
    memcpy(buf + offset, &(cmd.end), bool_size);
    offset += bool_size;
    memcpy(buf + offset, &data_size, int_size);
    offset += int_size;
    memcpy(buf + offset, cmd.data.c_str(), cmd.data.size());
    boost::asio::async_write(socket_, boost::asio::buffer(buf, total_size),
                             [this](boost::system::error_code ec, std::size_t length)
    {
        if (ec)
        {
            socket_.close();
            is_connect_ = false;
            connect();
        }
    });
}

void tcp_client::connect()
{
    timer_.expires_from_now(boost::posix_time::seconds(1));
    boost::asio::async_connect(socket_, tcp::resolver(tcp_service).resolve({addr_.c_str(), to_string(port_).c_str()}),
                               [this](boost::system::error_code ec, tcp::resolver::iterator)
    {
        if (!ec)
        {
            is_connect_ = true;
            read_head();
        }
    });
    timer_.async_wait(std::bind(&tcp_client::connect_timeout, this));
}

void tcp_client::connect_timeout()
{
    if (!is_connect_ && is_alive_)
    {
        socket_.cancel();
        connect();
    }
}


void tcp_client::read_head()
{
    boost::asio::async_read(socket_, boost::asio::buffer(buff_, data_offset),
                            [this](boost::system::error_code ec, std::size_t length)
    {
        if (!ec)
        {
            memcpy(&recv_type_, buff_, enum_size);
            memcpy(&recv_end_, buff_ + enum_size, bool_size);
            memcpy(&recv_size_, buff_ + enum_size + bool_size, int_size);
            read_data();
        }
        else
        {
            socket_.close();
            is_connect_ = false;
            connect();
        }
    });
}

void tcp_client::read_data()
{
    boost::asio::async_read(socket_, boost::asio::buffer(buff_ + data_offset, recv_size_),
                            [this](boost::system::error_code ec, std::size_t length)
    {
        if (!ec)
        {
            if (recv_cmd_.type != recv_type_)
            {
                recv_cmd_.type = recv_type_;
                recv_cmd_.end = recv_end_;
                recv_cmd_.size = recv_size_;
                recv_cmd_.data.clear();
                recv_cmd_.data.assign((char *)(buff_ + data_offset), recv_size_);
            }
            else
            {
                if (!recv_cmd_.end)
                {
                    recv_cmd_.end = recv_end_;
                    recv_cmd_.size += recv_size_;
                    recv_cmd_.data.append((char *)(buff_ + data_offset), recv_size_);
                }
                else
                {
                    recv_cmd_.type = recv_type_;
                    recv_cmd_.end = recv_end_;
                    recv_cmd_.size = recv_size_;
                    recv_cmd_.data.clear();
                    recv_cmd_.data.assign((char *)(buff_ + data_offset), recv_size_);
                }
            }

            if (recv_end_)
            {
                if (tcb_ != nullptr)
                {
                    tcb_(recv_cmd_);
                }
            }

            read_head();
        }
        else
        {
            socket_.close();
            is_connect_ = false;
            connect();
        }
    });
}

void tcp_client::start()
{
    is_alive_ = true;
    td_ = std::thread([this]()
    {
        connect();
        tcp_service.run();
    });
}

void tcp_client::stop()
{
    is_alive_ = false;
    socket_.cancel();
    tcp_service.stop();
}

tcp_client::~tcp_client()
{
    if (td_.joinable())
    {
        td_.join();
    }
}

