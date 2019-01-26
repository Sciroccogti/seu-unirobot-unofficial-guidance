#pragma once

#include <memory>
#include <list>
#include <mutex>
#include <atomic>
#include <map>
#include "timer.hpp"
#include "sensor.hpp"
#include "robot/humanoid.hpp"
#include "dynamixel/dynamixel_sdk.h"


class motor: public sensor, public timer
{
public:
    motor();
    void stop();
    bool start();
    void run();
    inline double freq() const
    {
        return 1000.0 / (double)period_ms_;
    }
    inline float low_power() const
    {
        return (voltage_ / 10.0f) < min_volt_ ? true : false;
    }
    inline bool is_connected() const
    {
        return is_connected_;
    }

private:
    bool open();
    void close();
    void set_torq(uint8_t e);
    void set_led(uint8_t s);
    void set_gpos();
    void read_pos();
    void read_voltage();

    void virtul_act();
    void real_act();
private:
    unsigned long p_count_;
    uint16_t voltage_;
    uint8_t ping_id_;
    uint8_t led_status_;
    std::map<int, float> curr_degs_;
    float min_volt_, max_volt_;
    
    std::atomic_bool is_connected_;

    std::shared_ptr<dynamixel::PortHandler> portHandler_;
    std::shared_ptr<dynamixel::PacketHandler> packetHandler_;
    std::shared_ptr<dynamixel::GroupSyncWrite> torqWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> gposWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> ledWrite_;
    std::shared_ptr<dynamixel::GroupSyncRead> pposRead_;
};

