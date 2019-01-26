#include "button.hpp"
#include "configuration.hpp"
#include <unistd.h>
#include "engine/led/LedEngine.hpp"

using namespace std;

button::button(): sensor("button")
{
    button1_ = make_shared<gpio>(gpio::gpio_map[CONF->get_config_value<string>("hardware.gpio.button.1")]);
    button2_ = make_shared<gpio>(gpio::gpio_map[CONF->get_config_value<string>("hardware.gpio.button.2")]);
    can_use_ = (button1_->opened()&&button2_->opened());
    if(can_use_)
    {
        button1_->set_direction(gpio::PIN_INPUT);
        button2_->set_direction(gpio::PIN_INPUT);
    }
    bt1_status_ = false;
    bt2_status_ = false;
}

button::~button()
{
    if(td_.joinable())
    {
        td_.join();
    }
}

bool button::start()
{
    is_alive_ = true;
    td_ = std::move(std::thread(&button::run, this));
    return true;
}

void button::stop()
{
    is_alive_ = false;
    button1_->gpio_unexport();
    button2_->gpio_unexport();
}

void button::run()
{
    while(is_alive_)
    {
        if(can_use_)
        {
            bt1_status_ = static_cast<bool>(button1_->get_value());
            bt2_status_ = static_cast<bool>(button2_->get_value());
            LE->set_state(LED_NORMAL);
            if(bt1_status_)
            {
                LE->set_state(LED_CUSTOM);
                LE->set_led(1, true);
            }
            if(bt2_status_)
            {
                LE->set_state(LED_CUSTOM);
                LE->set_led(2, true);
            }
            notify(SENSOR_BUTTON);
        }
        usleep(20000);
    }
}