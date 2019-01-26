#pragma once

#include <thread>
#include <memory>
#include <atomic>
#include "sensor.hpp"
#include "gpio/gpio.hpp"

class button: public sensor
{
public:
    button();
    ~button();
    bool start();
    void stop();

    inline bool button_1()
    {
        return bt1_status_;
    }

    inline bool button_2()
    {
        return bt2_status_;
    }

private:
    void run();
    std::thread td_;
    bool can_use_;
    std::atomic_bool bt1_status_;
    std::atomic_bool bt2_status_;
    std::shared_ptr<gpio> button1_;
    std::shared_ptr<gpio> button2_;
};