#include "server.hpp"
#include "configuration.hpp"
#include "robot/humanoid.hpp"
#include "task/walk_task.hpp"
#include "task/action_task.hpp"
#include "task/look_task.hpp"
#include <string>
#include <iostream>

using namespace std;
using namespace robot;
using boost::asio::ip::tcp;

void tcp_pool::join(tcp_session_ptr session)
{
    LOG << setw(12) << "connection:" << setw(18) << "[" + session->info() + "]" << " joined!" << ENDL;
    sessions_.insert(session);
}

void tcp_pool::leave(tcp_session_ptr session)
{
    LOG << setw(12) << "connection:" << setw(18) << "[" + session->info() + "]" << " leaved!" << ENDL;
    sessions_.erase(session);
}

void tcp_pool::deliver(const tcp_command &cmd)
{
    for (auto &session : sessions_)
    {
        if (session->check_type(cmd.type)||cmd.type == END_DATA)
        {
            session->deliver(cmd);
        }
    }
}

void tcp_pool::close()
{
    for (auto &session : sessions_)
    {
        session->stop();
    }
}

tcp_session::tcp_session(tcp::socket sock, tcp_pool &pool, tcp_callback ncb)
    : socket_(std::move(sock)), pool_(pool), tcb_(std::move(ncb))
{
    info_.clear();
    info_.append(socket_.remote_endpoint().address().to_string() + ":" + to_string(socket_.remote_endpoint().port()));
    recv_cmd_.type = NONE_DATA;
    memset(buff_, 0, MAX_CMD_LEN);
}

void tcp_session::start()
{
    pool_.join(shared_from_this());
    this->read_head();
}

void tcp_session::read_head()
{
    auto self(shared_from_this());
    boost::asio::async_read(socket_, boost::asio::buffer(buff_, data_offset),
                            [this, self](boost::system::error_code ec, std::size_t length)
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
            pool_.leave(shared_from_this());
        }
    });
}

void tcp_session::read_data()
{
    auto self(shared_from_this());
    boost::asio::async_read(socket_, boost::asio::buffer(buff_ + data_offset, recv_size_),
                            [this, self](boost::system::error_code ec, std::size_t length)
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
                if (recv_cmd_.type == REG_DATA)
                {
                    tcp_cmd_type t;
                    tcp_data_dir d;
                    std::memcpy(&t, recv_cmd_.data.c_str(), enum_size);
                    std::memcpy(&d, recv_cmd_.data.c_str() + enum_size, enum_size);
                    td_map_[t] = d;
                }
                else
                {
                    tcb_(recv_cmd_);
                }
            }

            read_head();
        }
        else
        {
            pool_.leave(shared_from_this());
        }
    });
}


void tcp_session::stop()
{
    socket_.close();
}

bool tcp_session::check_type(const tcp_cmd_type &t)
{
    for (auto &c : td_map_)
        if (c.first == t && (c.second == DIR_BOTH || c.second == DIR_APPLY))
        {
            return true;
        }

    return false;
}

void tcp_session::deliver(const tcp_command &cmd)
{
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

    auto self(shared_from_this());
    boost::asio::async_write(socket_, boost::asio::buffer(buf, total_size),
                             [this, self](boost::system::error_code ec, std::size_t length)
    {
        if (ec)
        {
            pool_.leave(shared_from_this());
        }
    });
}


boost::asio::io_service tcp_service;

tcp_server::tcp_server(): acceptor_(tcp_service, tcp::endpoint(tcp::v4(), CONF->get_config_value<int>("net.tcp.port"))), socket_(tcp_service)
{
    rmt_data_.type = NON_DATA;
    rmt_data_.size = 0;
    is_alive_ = false;
    LOG << std::setw(12) << "sensor:" << std::setw(18) << "[server]" << " started!" << ENDL;
}

void tcp_server::accept()
{
    acceptor_.async_accept(socket_, [this](boost::system::error_code ec)
    {
        if (!ec)
        {
            std::make_shared<tcp_session>(std::move(socket_), pool_,
                                          bind(&tcp_server::data_handler, this, placeholders::_1))->start();
        }

        accept();
    });
}

void tcp_server::data_handler(const tcp_command cmd)
{
    switch (cmd.type)
    {
        case JOINT_DATA:
        {
            if (cmd.size % sizeof(robot_joint_deg) == 0)
            {
                robot_joint_deg jd;

                for (size_t i = 0; i < ROBOT->get_joint_map().size(); i++)
                {
                    memcpy(&jd, cmd.data.c_str() + i * sizeof(robot_joint_deg), sizeof(robot_joint_deg));
                }
            }

            break;
        }

        case REMOTE_DATA:
        {
            rmt_mtx_.lock();
            memcpy(&(rmt_data_.type), cmd.data.c_str(), enum_size);
            rmt_data_.data.clear();
            rmt_data_.data.assign((char *)(cmd.data.c_str() + enum_size), cmd.size - enum_size);
            rmt_data_.size = cmd.size - enum_size;
            rmt_mtx_.unlock();
            break;
        }

        case TASK_DATA:
        {
            task_type t;
            memcpy(&t, cmd.data.c_str(), int_size);

            switch (t)
            {
                case TASK_WALK:
                {
                    float x, y, dir;
                    bool e;
                    memcpy(&x, cmd.data.c_str() + int_size, float_size);
                    memcpy(&y, cmd.data.c_str() + int_size + float_size, float_size);
                    memcpy(&dir, cmd.data.c_str() + int_size + 2 * float_size, float_size);
                    memcpy(&e, cmd.data.c_str() + int_size + 3 * float_size, bool_size);
                    task_mtx_.lock();
                    tasks_.push_back(make_shared<walk_task>(x, y, dir, e));
                    task_mtx_.unlock();
                    break;
                }

                case TASK_ACT:
                {
                    string act;
                    act.assign((char *)(cmd.data.c_str() + int_size), cmd.size - enum_size);
                    task_mtx_.lock();
                    tasks_.push_back(make_shared<action_task>(act));
                    task_mtx_.unlock();
                    break;
                }

                case TASK_LOOK:
                {
                    float yaw, pitch;
                    bool e;
                    memcpy(&yaw, cmd.data.c_str() + int_size, float_size);
                    memcpy(&pitch, cmd.data.c_str() + int_size + float_size, float_size);
                    memcpy(&e, cmd.data.c_str() + int_size + 2 * float_size, bool_size);
                    task_mtx_.lock();
                    tasks_.push_back(make_shared<look_task>(yaw, pitch, e));
                    break;
                }

                default:
                    break;
            }

        }

        default:
            break;
    }
}

void tcp_server::write(const tcp_command &cmd)
{
    if (!is_alive_)
    {
        return;
    }

    unsigned int t_size = cmd.size;
    unsigned max_data_size = MAX_CMD_LEN - int_size - enum_size - bool_size;
    int i = 0;
    tcp_command temp;
    temp.type = cmd.type;

    while (t_size > max_data_size)
    {
        temp.end = false;
        temp.size = max_data_size;
        temp.data.assign((char *)(cmd.data.c_str() + i * max_data_size), max_data_size);
        pool_.deliver(temp);
        t_size -= max_data_size;
        i++;
        usleep(10);
    }

    temp.size = t_size;
    temp.end = true;
    temp.data.assign((char *)(cmd.data.c_str() + i * max_data_size), t_size);
    pool_.deliver(temp);
}

bool tcp_server::start()
{
    is_alive_ = true;
    td_ = move(std::thread([this]()
    {
        accept();
        tcp_service.run();
    }));
    return true;
}

void tcp_server::stop()
{
    tcp_command cmd;
    cmd.type = END_DATA;
    cmd.size = 0;
    this->write(cmd);
    pool_.close();
    tcp_service.stop();
    is_alive_ = false;
}

tcp_server::~tcp_server()
{
    if (td_.joinable())
    {
        td_.join();
    }

    LOG << std::setw(12) << "sensor:" << std::setw(18) << "[server]" << " ended!" << ENDL;
}
