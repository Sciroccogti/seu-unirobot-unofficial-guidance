#include "joint_revise.hpp"
#include "configuration.hpp"
#include "parser/offset_parser.hpp"

using namespace robot;
using namespace std;

joint_revise::joint_revise(tcp_client &client, QString netinfo, QWidget *parent)
    : net_info(netinfo), client_(client), QMainWindow(parent)
{
    setAttribute(Qt::WA_DeleteOnClose);
    QHBoxLayout *mainLayout = new QHBoxLayout();
    QVBoxLayout *leftLayout = new QVBoxLayout();
    QVBoxLayout *rightLayout = new QVBoxLayout();

    first_connect = true;
    JSlider *slider;

    for (auto &jo : ROBOT->get_joint_map())
    {
        slider = new JSlider(jo.second);
        connect(slider, &JSlider::valueChanged, this, &joint_revise::procValueChanged);

        if (jo.first == "jhead2" || jo.first.find("jr") != string::npos)
        {
            rightLayout->addWidget(slider);
        }
        else
        {
            leftLayout->addWidget(slider);
        }

        j_sliders_[jo.first] = slider;
    }

    btnReset = new QPushButton("Reset");
    btnSave = new QPushButton("Save");
    leftLayout->addWidget(btnReset);
    rightLayout->addWidget(btnSave);
    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);
    setWindowTitle(net_info);

    timer = new QTimer;
    timer->start(1000);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    connect(btnReset, &QPushButton::clicked, this, &joint_revise::procBtnReset);
    connect(btnSave, &QPushButton::clicked, this, &joint_revise::procBtnSave);
    connect(timer, &QTimer::timeout, this, &joint_revise::procTimer);
    setEnabled(false);
}

void joint_revise::procTimer()
{
    if (client_.is_connected())
    {
        if (first_connect)
        {
            client_.regist(REMOTE_DATA, DIR_SUPPLY);
            first_connect = false;
        }

        setEnabled(true);
        statusBar()->setStyleSheet("background-color:green");
    }
    else
    {
        first_connect = true;
        setEnabled(false);
        statusBar()->setStyleSheet("background-color:red");
    }
}

void joint_revise::procBtnReset()
{
    for (auto &s : j_sliders_)
    {
        s.second->reset();
        ROBOT->get_joint_map()[s.first]->offset_ = 0.0;
    }
}

void joint_revise::procBtnSave()
{
    parser::offset_parser::save(CONF->offset_file(), ROBOT->get_joint_map());
}

void joint_revise::procValueChanged(int id, float v)
{
    ROBOT->get_joint(id)->offset_ = v;
    tcp_command cmd;
    remote_data_type t = JOINT_OFFSET;
    cmd.data.clear();
    cmd.type = REMOTE_DATA;
    cmd.size = sizeof(robot_joint_deg) * ROBOT->get_joint_map().size() + enum_size;
    cmd.data.clear();
    robot_joint_deg offset;
    cmd.data.append((char *)&t, enum_size);

    for (auto &j : ROBOT->get_joint_map())
    {
        offset.id = j.second->jid_;
        offset.deg = j.second->offset_;
        cmd.data.append((char *)(&offset), sizeof(robot_joint_deg));
    }

    client_.write(cmd);
}
