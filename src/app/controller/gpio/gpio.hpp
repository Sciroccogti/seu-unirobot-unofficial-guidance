#pragma once

#include <map>
#include <string>

class gpio
{
public:
    enum pin_direction
    {
        PIN_INPUT  = 0,
        PIN_OUTPUT = 1
    } ;

    enum pin_value
    {
        PIN_LOW = 0,
        PIN_HIGH = 1
    };

    enum pin_number
    {
        PIN_GPIO0 = 388,
        PIN_GPIO1 = 298,
        PIN_GPIO2 = 480,
        PIN_GPIO3 = 486
    };
    static std::map<std::string, pin_number> gpio_map;
public:
    gpio(pin_number pin);

    bool set_direction(pin_direction dir);
    bool set_value(pin_value v);
    bool set_edge(char *edge);
    int get_value();
    bool gpio_unexport();
    
    inline bool opened()
    {
        return opened_;
    }

private:
    bool gpio_export();
    
    bool opened_;
    pin_number io_;
};