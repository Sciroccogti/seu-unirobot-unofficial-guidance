#pragma once

#include "task.hpp"
#include "engine/led/LedEngine.hpp"

class led_task: public task
{
public:
    led_task(bool led1, bool led2): led1_(led1), led2_(led2), task("led")
    {

    }

    bool perform()
    {
        LE->set_state(LED_CUSTOM);
        LE->set_led(1, led1_);
        LE->set_led(2, led2_);
        return true;
    }

private:
    bool led1_;
    bool led2_;
};