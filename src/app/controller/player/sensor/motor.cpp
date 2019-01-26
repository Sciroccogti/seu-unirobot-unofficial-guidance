#include "motor.hpp"
#include "configuration.hpp"
#include "core/adapter.hpp"
#include "options/options.hpp"
#include "class_exception.hpp"
#include "core/worldmodel.hpp"
#include "math/math.hpp"
#include "server/server.hpp"
#include <fstream>
#define ZERO_POS 2048
#define MAX_POS  4096
#define MIN_POS  0

#define ADDR_ID     7
#define ADDR_TORQ   64
#define ADDR_LED    65
#define ADDR_GPOS   116
#define ADDR_PPOS   132
#define ADDR_VOLT   144

#define SIZE_ID     1
#define SIZE_TORQ   1
#define SIZE_LED    1
#define SIZE_GPOS   4
#define SIZE_PPOS   4
#define SIZE_VOLT   2

using namespace std;
using namespace robot;
using namespace dynamixel;
using namespace robot_math;

motor::motor(): sensor("motor"), timer(CONF->get_config_value<int>("hardware.motor.period"))
{
    p_count_ = 0;
    portHandler_ = PortHandler::getPortHandler(CONF->get_config_value<string>("hardware.motor.dev_name").c_str());
    packetHandler_ = PacketHandler::getPacketHandler(CONF->get_config_value<float>("hardware.motor.version"));
    ledWrite_ = make_shared<GroupSyncWrite>(portHandler_, packetHandler_, ADDR_LED, SIZE_LED);
    torqWrite_ = make_shared<GroupSyncWrite>(portHandler_, packetHandler_, ADDR_TORQ, SIZE_TORQ);
    gposWrite_ = make_shared<GroupSyncWrite>(portHandler_, packetHandler_, ADDR_GPOS, SIZE_GPOS);
    pposRead_ = make_shared<GroupSyncRead>(portHandler_, packetHandler_, ADDR_PPOS, SIZE_PPOS);

    ping_id_ = static_cast<uint8_t>(ROBOT->get_joint("jrhip3")->jid_);
    is_connected_ = false;
    min_volt_ = CONF->get_config_value<float>("hardware.battery.min_volt");
    max_volt_ = CONF->get_config_value<float>("hardware.battery.max_volt");
    voltage_ = static_cast<uint16_t>(max_volt_ * 10);

    for (auto &r : ROBOT->get_joint_map())
    {
        pposRead_->addParam(static_cast<uint8_t>(r.second->jid_));
    }
}

inline uint32_t float2pos(const float &deg)
{
    return static_cast<uint32_t>(deg / (360.0f / (float)(MAX_POS - MIN_POS)) + ZERO_POS);
}

inline float pos2float(const uint32_t &pos)
{
    return (float)((int)pos - ZERO_POS) * (360.0f / (float)(MAX_POS - MIN_POS));
}

bool motor::start()
{
    if (!open())
    {
        return false;
    }

    sensor::is_alive_ = true;
    timer::is_alive_ = true;
    start_timer();
    return true;
}

void motor::run()
{
    std::map<int, float> jdmap;

    if (timer::is_alive_)
    {
        ROBOT->set_degs(MADT->get_degs());

        if (OPTS->use_debug())
        {
            virtul_act();
        }

        if (OPTS->use_robot())
        {
            real_act();
        }
        notify(SENSOR_MOTOR);
        p_count_++;
    }
}

void motor::virtul_act()
{
    tcp_command cmd;
    cmd.type = JOINT_DATA;
    robot_joint_deg jd;
    string j_data;
    j_data.clear();

    for (auto &jm : ROBOT->get_joint_map())
    {
        jd.id = jm.second->jid_;
        jd.deg = jm.second->get_deg();
        j_data.append((char *)(&jd), sizeof(robot_joint_deg));
    }

    cmd.size = ROBOT->get_joint_map().size() * sizeof(robot_joint_deg);
    cmd.data.assign(j_data.c_str(), cmd.size);
    SERVER->write(cmd);
}

void motor::real_act()
{
    uint8_t dxl_error = 0;
    int not_alert_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    uint16_t dxl_model_number;

    if (!is_connected_)
    {
        dxl_comm_result = packetHandler_->ping(portHandler_, ping_id_, &dxl_model_number, &dxl_error);
        not_alert_error = dxl_error & ~128;

        if (dxl_comm_result == COMM_SUCCESS && not_alert_error == 0)
        {
            read_voltage();
            LOG << "Voltage: " << voltage_ / 10.0f << "V !" << ENDL;

            //read_pos();
            set_torq(1);
            is_connected_ = true;
        }
    }
    else
    {
        set_gpos();
        //read_pos();

        if ((p_count_ * period_ms_ % 1000) == 0)
        {
            led_status_ = 1 - led_status_;
            set_led(led_status_);
        }
    }
}

bool motor::open()
{
    if (OPTS->use_robot())
    {
        if (!portHandler_->openPort())
        {
            return false;
        }

        if (!portHandler_->setBaudRate(CONF->get_config_value<int>("hardware.motor.baudrate")))
        {
            return false;
        }

        is_open_ = true;
    }

    return true;
}

void motor::close()
{
    if (is_open_)
    {
        is_open_ = false;
        portHandler_->closePort();
    }
}

void motor::stop()
{
    sensor::is_alive_ = false;
    timer::is_alive_ = false;
    close();
    delete_timer();
}

void motor::read_pos()
{
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t id = 0;
    dxl_comm_result = pposRead_->txRxPacket();

    if (dxl_comm_result != COMM_SUCCESS)
    {
        LOG << packetHandler_->getTxRxResult(dxl_comm_result) << ENDL;
    }
    else
    {
        for (auto &r : ROBOT->get_joint_map())
        {
            id = static_cast<uint8_t >(r.second->jid_);

            if (pposRead_->isAvailable(id, ADDR_PPOS, SIZE_PPOS))
            {
                curr_degs_[static_cast<int>(id)] = pos2float(pposRead_->getData(id, ADDR_PPOS, SIZE_PPOS));
            }
            else
            {
                curr_degs_[static_cast<int>(id)] = 0.0;
            }
        }
    }
}

void motor::read_voltage()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, ping_id_, ADDR_VOLT, (uint16_t *)&voltage_, &dxl_error);
}

void motor::set_torq(uint8_t e)
{
    torqWrite_->clearParam();
    uint8_t torq_data;
    torq_data = e;

    for (auto &r : ROBOT->get_joint_map())
    {
        torqWrite_->addParam(static_cast<uint8_t>(r.second->jid_), &torq_data);
    }

    torqWrite_->txPacket();
}

void motor::set_led(uint8_t s)
{
    ledWrite_->clearParam();
    uint8_t led_data;
    led_data = s;

    for (auto &r : ROBOT->get_joint_map())
    {
        ledWrite_->addParam(static_cast<uint8_t>(r.second->jid_), &led_data);
    }

    ledWrite_->txPacket();
}

void motor::set_gpos()
{
    gposWrite_->clearParam();
    uint8_t gpos_data[4];
    uint32_t gpos;
    float deg;

    for (auto &j : ROBOT->get_joint_map())
    {
        deg = (j.second->inverse_) * (j.second->get_deg() + j.second->offset_);
        gpos = float2pos(deg);
        gpos_data[0] = DXL_LOBYTE(DXL_LOWORD(gpos));
        gpos_data[1] = DXL_HIBYTE(DXL_LOWORD(gpos));
        gpos_data[2] = DXL_LOBYTE(DXL_HIWORD(gpos));
        gpos_data[3] = DXL_HIBYTE(DXL_HIWORD(gpos));

        if (!gposWrite_->addParam(static_cast<uint8_t>(j.second->jid_), gpos_data))
        {
            return;
        }
    }

    gposWrite_->txPacket();
}
