#pragma once

#include <mutex>
#include <atomic>
#include "pattern.hpp"
#include "sensor/imu.hpp"
#include "sensor/motor.hpp"
#include "sensor/button.hpp"
#include "configuration.hpp"
#include "singleton.hpp"
#include "model.hpp"
#include "math/math.hpp"
#include "robot/humanoid.hpp"

class WorldModel: public subscriber, public singleton<WorldModel>
{
public:
    WorldModel();

    void updata(const pub_ptr &pub, const int &type);

    inline int support_foot() const
    {
        return support_foot_;
    }

    inline void set_support_foot(const robot::support_foot &sf)
    {
        support_foot_ = sf;
    }

    inline imu::imu_data imu_data() const
    {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        return imu_data_;
    }

    inline robot_math::transform_matrix head_matrix() const
    {
        std::lock_guard<std::mutex> lk(head_mtx_);
        return head_matrix_;
    }

    inline int fall_data() const
    {
        return fall_direction_;
    }

    inline bool lost() const
    {
        return lost_;
    }

public:
    float ballx_, bally_;
    float bodyx_, bodyy_, bodydir_;
    
private:
    imu::imu_data imu_data_;

    robot_math::transform_matrix body_matrix_, head_matrix_;
    std::vector<double> head_degs_;

    std::atomic_bool lost_;
    std::atomic_bool bt1_status_;
    std::atomic_bool bt2_status_;
    std::atomic_int fall_direction_;
    std::atomic_int support_foot_;

    mutable std::mutex imu_mtx_, dxl_mtx_, head_mtx_;
};

#define WM WorldModel::instance()

