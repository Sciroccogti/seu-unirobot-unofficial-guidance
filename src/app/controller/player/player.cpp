#include "player.hpp"
#include "configuration.hpp"
#include "task/action_task.hpp"
#include "task/look_task.hpp"
#include "server/server.hpp"
#include "core/adapter.hpp"
#include "engine/walk/WalkEngine.hpp"
#include "engine/scan/ScanEngine.hpp"
#include "engine/action/ActionEngine.hpp"
#include "engine/led/LedEngine.hpp"

using namespace std;
using namespace motion;

player::player(): timer(CONF->get_config_value<int>("think_period"))
{
    is_alive_ = false;
    period_count_ = 0;
}

bool player::init()
{
    SERVER->start();

    if (!regist())
    {
        return false;
    }

    is_alive_ = true;

    if (OPTS->use_robot())
    {
        while (!dynamic_pointer_cast<motor>(sensors_["motor"])->is_connected() && is_alive_)
        {
            LOG << "waiting for motor connection, please turn on the power." << ENDL;
            sleep(1);
        }

        if (!is_alive_)
        {
            return true;
        }
    }

    MADT->start();
    WE->start();
    SE->start();
    AE->start();
    if(OPTS->use_robot())
    {
        LE->start();
    }
    action_task p("reset");
    p.perform();
    start_timer();
    return true;
}

void player::stop()
{
    WE->stop();
    SE->stop();
    AE->stop();
    if(OPTS->use_robot())
    {
        LE->stop();
    }
    MADT->stop();

    if (is_alive_)
    {
        delete_timer();
    }

    is_alive_ = false;
    sleep(1);
    unregist();
    SERVER->stop();
}

void player::run()
{
    if (is_alive_)
    {
        period_count_++;

        tcp_command cmd;
        int fall = WM->fall_data();
        cmd.type = WM_DATA;
        cmd.size = 5 * float_size + bool_size;
        cmd.data.append((char *) & (WM->ballx_), float_size);
        cmd.data.append((char *) & (WM->bally_), float_size);
        cmd.data.append((char *) & (WM->bodyx_), float_size);
        cmd.data.append((char *) & (WM->bodyy_), float_size);
        cmd.data.append((char *) & (WM->bodydir_), float_size);
        cmd.data.append((char *) & fall, int_size);
        SERVER->write(cmd);

        if (OPTS->use_robot())
        {
            if (WM->lost())
            {
                LOG << "hardware has no response!" << ENDL;
                raise(SIGINT);
            }
        }
        if(OPTS->use_remote())
        {
            play_with_remote();
        }
        else
        {
            list<task_ptr> tasks = SERVER->tasks();
            for(auto &tsk: tasks)
            {
                tsk->perform();
            }
        }
    }
}

bool player::regist()
{
    sensors_.clear();

    if (OPTS->use_camera())
    {
        sensors_["camera"] = std::make_shared<camera>();
        sensors_["camera"]->attach(VISION);
        sensors_["camera"]->start();

        if (!VISION->start())
        {
            return false;
        }
    }

    sensors_["motor"] = std::make_shared<motor>();
    sensors_["motor"]->attach(WM);
    sensors_["motor"]->attach(WE);

    if (!sensors_["motor"]->start())
    {
        return false;
    }

    if (OPTS->use_robot())
    {
        sensors_["imu"] = std::make_shared<imu>();
        sensors_["imu"]->attach(WM);
        sensors_["imu"]->attach(WE);
        sensors_["imu"]->start();

        sensors_["button"] = std::make_shared<button>();
        sensors_["button"]->attach(WM);
        sensors_["button"]->start();
    }

    return true;
}

void player::unregist()
{
    if (sensors_.find("button") != sensors_.end())
    {
        sensors_["button"]->detach(WM);
        sensors_["button"]->stop();
    }
    if (sensors_.find("imu") != sensors_.end())
    {
        sensors_["imu"]->detach(WM);
        sensors_["imu"]->detach(WE);
        sensors_["imu"]->stop();
    }

    if (sensors_.find("motor") != sensors_.end())
    {
        sensors_["motor"]->detach(WM);
        sensors_["motor"]->detach(WE);
        sensors_["motor"]->stop();
    }

    if (sensors_.find("camera") != sensors_.end())
    {
        sensors_["camera"]->detach(VISION);
        sensors_["camera"]->stop();
        VISION->stop();
    }
}

sensor_ptr player::get_sensor(const std::string &name)
{
    auto iter = sensors_.find(name);

    if (iter != sensors_.end())
    {
        return iter->second;
    }

    return nullptr;
}