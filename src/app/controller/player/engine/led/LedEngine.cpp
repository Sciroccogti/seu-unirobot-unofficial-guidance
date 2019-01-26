#include "LedEngine.hpp"
#include <unistd.h>
#include "common.hpp"
#include "configuration.hpp"

using namespace std;

LedEngine::LedEngine()
{
    led1_ = make_shared<gpio>(gpio::gpio_map[CONF->get_config_value<string>("hardware.gpio.led.1")]);
    led2_ = make_shared<gpio>(gpio::gpio_map[CONF->get_config_value<string>("hardware.gpio.led.2")]);
    can_use_ = (led1_->opened() && led2_->opened());
    if(can_use_)
    {
        led1_->set_direction(gpio::PIN_OUTPUT);
        led2_->set_direction(gpio::PIN_OUTPUT);
    }
    led1_status_ = false;
    led2_status_ = false;
    led_state_ = LED_NORMAL;
    count_ = 0;
    custom_ = 0;
}

LedEngine::~LedEngine()
{
    if(td_.joinable())
    {
        td_.join();
    }
    LOG << std::setw(12) << "engine:" << std::setw(18) << "[LedEngine]" << " ended!" << ENDL;
}

void LedEngine::start()
{
    is_alive_ = true;
    LOG << std::setw(12) << "engine:" << std::setw(18) << "[LedEngine]" << " started!" << ENDL;
    td_ = std::move(std::thread(&LedEngine::run, this));
}

void LedEngine::stop()
{
    is_alive_ = false;
    led1_->gpio_unexport();
    led2_->gpio_unexport();
}

void LedEngine::run()
{
    while(is_alive_)
    {
        switch(led_state_)
        {
            case LED_NORMAL:
            {
                custom_ = 0;
                if(can_use_ && count_%100 == 0)
                {
                    led1_status_ = led1_status_?false:true;
                    led2_status_ = led1_status_?false:true;
                    led1_->set_value(led1_status_?gpio::PIN_LOW:gpio::PIN_HIGH);
                    led2_->set_value(led2_status_?gpio::PIN_LOW:gpio::PIN_HIGH);
                }
                break;
            }
            case LED_WARN:
            {
                if(can_use_ && count_%50 == 0)
                {
                    led1_status_ = led1_status_?false:true;
                    led2_status_ = led1_status_?true:false;
                    led1_->set_value(led1_status_?gpio::PIN_LOW:gpio::PIN_HIGH);
                    led2_->set_value(led2_status_?gpio::PIN_LOW:gpio::PIN_HIGH);
                }
                break;
            }
            case LED_ERROR:
            {
                if(can_use_)
                {
                    led1_->set_value(gpio::PIN_LOW);
                    led2_->set_value(gpio::PIN_LOW);
                }
                break;
            }
            case LED_CUSTOM:
            {
                custom_ ++;
                if(can_use_)
                {
                    led1_->set_value(led1_status_?gpio::PIN_LOW:gpio::PIN_HIGH);
                    led2_->set_value(led2_status_?gpio::PIN_LOW:gpio::PIN_HIGH);
                }
                break;
            }
            default:
                break;
        }
        if(custom_>=500) led_state_ = LED_NORMAL;
        usleep(10000);
        count_ ++;
    }
}

bool LedEngine::set_led(int idx, bool status)
{
    if(idx == 1)
    {
        led1_status_ = status;
        return true;
    }
    else if(idx == 2)
    {
        led2_status_ = status;
        return true;
    }
    else
    {
        return false;
    }
}