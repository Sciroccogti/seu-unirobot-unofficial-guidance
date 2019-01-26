#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <eigen3/Eigen/Dense>
#include "sensor.hpp"
#include "timer.hpp"
#include "model.hpp"

class imu: public sensor, public timer
{
public:
    struct imu_data
    {
        float pitch, roll, yaw;
        float ax, ay, az;
        float wx, wy, wz;
    };

    imu();
    ~imu();

    bool start();
    void stop();

    inline imu_data data() const
    {
        return imu_data_;
    }
    inline void set_zero()
    {
        reset_ = true;
    }
    inline bool lost() const
    {
        return lost_;
    }
    inline int fall_direction() const
    {
        return fall_direction_;
    }
private:
    bool open();
    void read_head0();
    void read_head1();
    void read_data();
    void run();

    enum {imu_data_size = sizeof(imu_data)};
    enum {imu_len = 11};
    unsigned char buff_[imu_len];
    imu_data imu_data_;
    std::thread td_;

    boost::asio::serial_port serial_;
    std::atomic_bool reset_;
    std::atomic_bool lost_;
    std::atomic_bool connected_;
    std::atomic_int count_;
    std::atomic_int fall_direction_;

    Eigen::Vector2f pitch_range_;
    Eigen::Vector2f roll_range_;
};
