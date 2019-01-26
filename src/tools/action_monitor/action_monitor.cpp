#include "action_monitor.hpp"
#include "configuration.hpp"

using namespace std;
using namespace robot;

action_monitor::action_monitor()
    : client_(CONF->get_config_value<string>(CONF->player() + ".address"), CONF->get_config_value<int>("net.tcp.port"),
              bind(&action_monitor::data_handler, this, placeholders::_1))
{
    rgl = new RobotGL(ROBOT->get_main_bone(), ROBOT->get_joint_map());
    setCentralWidget(rgl);

    first_connect = true;
    net_info = QString::fromStdString(CONF->get_config_value<string>(CONF->player() + ".address"))
               + ":" + QString::number(CONF->get_config_value<int>("net.tcp.port"));
    setWindowTitle(net_info);

    timer = new QTimer;
    timer->start(1000);

    connect(timer, &QTimer::timeout, this, &action_monitor::procTimer);
    client_.start();
}

void action_monitor::data_handler(const tcp_command cmd)
{
    unsigned int size = cmd.size;

    if (cmd.type == JOINT_DATA)
    {
        robot_joint_deg jdeg;
        std::map<int, float> jdegs;

        for (unsigned int i = 0; i < size / sizeof(robot_joint_deg); i++)
        {
            memcpy(&jdeg, cmd.data.c_str() + i * sizeof(robot_joint_deg), sizeof(robot_joint_deg));
            jdegs[jdeg.id] = jdeg.deg;
        }

        rgl->turn_joint(jdegs);
        this->update();
    }
}

void action_monitor::procTimer()
{
    if (client_.is_connected())
    {
        if (first_connect)
        {
            client_.regist(JOINT_DATA, DIR_APPLY);
        }

        first_connect = false;
        statusBar()->setStyleSheet("background-color:green");
    }
    else
    {
        first_connect = true;
        statusBar()->setStyleSheet("background-color:red");
    }
}

void action_monitor::closeEvent(QCloseEvent *event)
{
    client_.stop();
}
