#pragma once

#include <list>
#include "timer.hpp"
#include "core/worldmodel.hpp"
#include "robot/humanoid.hpp"
#include "options/options.hpp"
#include "sensor/imu.hpp"
#include "sensor/motor.hpp"
#include "sensor/camera.hpp"
#include "sensor/button.hpp"
#include "vision/vision.hpp"
#include "common.hpp"
#include "task/task.hpp"

class player: public timer, public std::enable_shared_from_this<player>
{
public:
    player();
    bool init();
    void stop();
    bool is_alive() const
    {
        return is_alive_;
    }

private:
    void run();
    void play_with_remote();
    bool regist();
    void unregist();
    sensor_ptr get_sensor(const std::string &name);
private:
    unsigned long period_count_;
    std::map<std::string, sensor_ptr> sensors_;
};
