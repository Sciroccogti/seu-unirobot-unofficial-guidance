#pragma once

#include <iomanip>
#include <memory>
#include "pattern.hpp"
#include <iostream>
#include "common.hpp"

class sensor: public publisher
{
public:
    enum sensor_type
    {
        SENSOR_NONE = 0,
        SENSOR_BUTTON = 1,
        SENSOR_IMU = 4,
        SENSOR_MOTOR = 5,
        SENSOR_CAMERA = 6,
        SENSOR_END = 10
    };
    sensor(const std::string &name): name_(name)
    {
        LOG << std::setw(12) << "sensor:" << std::setw(18) << "[" + name_ + "]" << " started!" << ENDL;
    }

    ~sensor()
    {
        LOG << std::setw(12) << "sensor:" << std::setw(18) << "[" + name_ + "]" << " ended!" << ENDL;
    }

    virtual bool start() = 0;
    virtual void stop() = 0;

    inline bool is_open() const
    {
        return is_open_;
    }

protected:
    bool is_open_;
    bool is_alive_;
    std::string name_;
};

typedef std::shared_ptr<sensor> sensor_ptr;

