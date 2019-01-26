#pragma once

#include <boost/asio.hpp>
#include <vector>
#include <list>
#include <set>
#include <thread>
#include <mutex>
#include "tcp.hpp"
#include "singleton.hpp"
#include "task/task.hpp"
#include "common.hpp"

class tcp_session;
typedef std::shared_ptr<tcp_session> tcp_session_ptr;

class tcp_pool
{
public:
    void join(tcp_session_ptr session);
    void leave(tcp_session_ptr session);
    void close();
    void deliver(const tcp_command &cmd);
private:
    std::set<tcp_session_ptr> sessions_;
};

class tcp_session: public std::enable_shared_from_this<tcp_session>
{
public:
    tcp_session(boost::asio::ip::tcp::socket sock, tcp_pool &pool, tcp_callback ncb);
    void start();
    void stop();
    void deliver(const tcp_command &cmd);
    bool check_type(const tcp_cmd_type &t);
    inline std::string info() const
    {
        return info_;
    }
private:
    void read_head();
    void read_data();
    std::map<tcp_cmd_type, tcp_data_dir> td_map_;
    boost::asio::ip::tcp::socket socket_;
    tcp_pool &pool_;
    tcp_callback tcb_;
    std::string info_;
    tcp_command recv_cmd_;
    char buff_[MAX_CMD_LEN];
    tcp_cmd_type recv_type_;
    bool recv_end_;
    unsigned int recv_size_;
};

class tcp_server: public singleton<tcp_server>
{
public:
    tcp_server();
    ~tcp_server();
    bool start();
    void stop();
    void write(const tcp_command &cmd);

    std::list<task_ptr> tasks()
    {
        std::lock_guard<std::mutex> lk(task_mtx_);
        std::list<task_ptr> res;
        res.clear();
        res.insert(res.end(), tasks_.begin(), tasks_.end());
        tasks_.clear();
        return res;
    }

    remote_data rmt_data() const
    {
        std::lock_guard<std::mutex> lk(rmt_mtx_);
        return rmt_data_;
    }

    void reset_rmt_data()
    {
        std::lock_guard<std::mutex> lk(rmt_mtx_);
        rmt_data_.type = NON_DATA;
        rmt_data_.size = 0;
        rmt_data_.data.clear();
    }

private:
    void accept();
    void data_handler(const tcp_command cmd);
    std::vector<std::thread> session_threads_;
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket socket_;
    std::thread td_;
    tcp_pool pool_;
    remote_data rmt_data_;
    int port_;
    bool is_alive_;
    std::list<task_ptr> tasks_;
    mutable std::mutex task_mtx_, rmt_mtx_;
};

#define SERVER tcp_server::instance()
