#include "team_monitor.hpp"
#include "configuration.hpp"
#include "parser/field_parser.hpp"

using boost::asio::ip::udp;
using namespace std;

boost::asio::io_service udp_service;

team_monitor::team_monitor(): socket_(udp_service, udp::endpoint(udp::v4(), CONF->get_config_value<short>("net.udp.teammate.port")))
{
    parser::field_parser::parse(CONF->field_file(), field_);
    setFixedSize(field_.field_length + 2 * field_.border_strip_width_min, field_.field_width + 2 * field_.border_strip_width_min);
    setStyleSheet("background:green");
    td_ = std::move(std::thread([this]()
    {
        this->receive();
        udp_service.run();
    }));
}

team_monitor::~team_monitor()
{
    if(td_.joinable())
    {
        td_.join();
    }
}

void team_monitor::receive()
{
    player_info p;
    socket_.async_receive_from(boost::asio::buffer((char *)&p, player_info_size), point_,
                               [this, p](boost::system::error_code ec, std::size_t bytes_recvd)
    {
        if (!ec && bytes_recvd > 0)
        {
            p_mutex_.lock();
            players_[p.id] = p;
            p_mutex_.unlock();
        }

        receive();
    });
}

void team_monitor::closeEvent(QCloseEvent *event)
{
    socket_.cancel();
    udp_service.stop();
}

void team_monitor::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.translate(field_.field_length / 2 + field_.border_strip_width_min, field_.field_width / 2 + field_.border_strip_width_min);
    painter.setPen(QPen(Qt::white, 4, Qt::SolidLine, Qt::FlatCap));
    painter.drawEllipse(-field_.center_circle_diameter / 2, -field_.center_circle_diameter / 2, field_.center_circle_diameter, field_.center_circle_diameter);
    painter.drawLine(0, -field_.field_width / 2, 0, field_.field_width / 2);
    painter.drawLine(-field_.field_length / 2, -field_.field_width / 2, field_.field_length / 2, -field_.field_width / 2);
    painter.drawLine(-field_.field_length / 2, -field_.field_width / 2, -field_.field_length / 2, field_.field_width / 2);
    painter.drawLine(-field_.field_length / 2, field_.field_width / 2, field_.field_length / 2, field_.field_width / 2);
    painter.drawLine(field_.field_length / 2, -field_.field_width / 2, field_.field_length / 2, field_.field_width / 2);
    painter.drawLine(-field_.field_length / 2, -field_.goal_area_width / 2, -(field_.field_length / 2 - field_.goal_area_length), -field_.goal_area_width / 2);
    painter.drawLine(-field_.field_length / 2, field_.goal_area_width / 2, -(field_.field_length / 2 - field_.goal_area_length), field_.goal_area_width / 2);
    painter.drawLine(-(field_.field_length / 2 - field_.goal_area_length), -field_.goal_area_width / 2, -(field_.field_length / 2 - field_.goal_area_length), field_.goal_area_width / 2);
    painter.drawLine(-(field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, -field_.field_length / 2, -field_.goal_width / 2);
    painter.drawLine(-(field_.field_length / 2 + field_.goal_depth), field_.goal_width / 2, -field_.field_length / 2, field_.goal_width / 2);
    painter.drawLine(-(field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, -(field_.field_length / 2 + field_.goal_depth), field_.goal_width / 2);
    painter.drawLine(field_.field_length / 2, -field_.goal_area_width / 2, (field_.field_length / 2 - field_.goal_area_length), -field_.goal_area_width / 2);
    painter.drawLine(field_.field_length / 2, field_.goal_area_width / 2, (field_.field_length / 2 - field_.goal_area_length), field_.goal_area_width / 2);
    painter.drawLine((field_.field_length / 2 - field_.goal_area_length), -field_.goal_area_width / 2, (field_.field_length / 2 - field_.goal_area_length), field_.goal_area_width / 2);
    painter.drawLine((field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, field_.field_length / 2, -field_.goal_width / 2);
    painter.drawLine((field_.field_length / 2 + field_.goal_depth), field_.goal_width / 2, field_.field_length / 2, field_.goal_width / 2);
    painter.drawLine((field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, (field_.field_length / 2 + field_.goal_depth), field_.goal_width / 2);

    painter.setPen(QPen(Qt::lightGray, 1, Qt::SolidLine, Qt::FlatCap));
    int i;

    for (i = 0; i < field_.field_width / 2; i += 100)
    {
        painter.drawLine(-field_.field_length / 2, i, field_.field_length / 2, i);
        painter.drawLine(-field_.field_length / 2, -i, field_.field_length / 2, -i);
    }

    for (i = 100; i < field_.field_length / 2; i += 100)
    {
        painter.drawLine(i, -field_.field_width / 2, i, field_.field_width / 2);
        painter.drawLine(-i, -field_.field_width / 2, -i, field_.field_width / 2);
    }

    painter.setPen(QPen(Qt::blue, 2, Qt::SolidLine, Qt::FlatCap));
    painter.setBrush(QBrush(Qt::blue, Qt::SolidPattern));
    painter.drawRect(-(field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, field_.goal_depth, field_.goal_width);
    painter.setPen(QPen(Qt::red, 2, Qt::SolidLine, Qt::FlatCap));
    painter.setBrush(QBrush(Qt::red, Qt::SolidPattern));
    painter.drawRect((field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, -field_.goal_depth, field_.goal_width);
    painter.setPen(QPen(Qt::white, 1, Qt::SolidLine, Qt::FlatCap));
    painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));
    painter.drawRect(-4, -4, 8, 8);
    painter.drawRect((field_.field_length / 2 - field_.penalty_mark_distance) - 4, -4, 8, 8);
    painter.drawRect(-(field_.field_length / 2 - field_.penalty_mark_distance) + 4, -4, 8, 8);

    int ballsize = 20;
    painter.setBrush(QBrush(Qt::black, Qt::SolidPattern));
    p_mutex_.lock();

    for (auto &p : players_)
    {
        painter.translate(p.second.x * 100, p.second.y * 100);
        painter.drawEllipse(-ballsize / 2, -ballsize / 2, ballsize, ballsize);
        painter.drawText(-ballsize / 2, -ballsize / 2, QString::number(p.second.id));
        painter.rotate(p.second.dir);
        painter.drawLine(0, 0, 2 * ballsize, 0);
        painter.rotate(-p.second.dir);
        painter.translate(-p.second.x * 100, -p.second.y * 100);
        painter.drawEllipse(p.second.ball_x * 100 - ballsize / 2, p.second.ball_y * 100 - ballsize / 2, ballsize, ballsize);
        painter.drawText(p.second.ball_x * 100 - ballsize / 2, p.second.ball_y * 100 - ballsize / 2, QString::number(p.second.id));
    }

    p_mutex_.unlock();
}