#pragma once

#include <map>
#include <thread>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include "pattern.hpp"
#include "sensor/imu.hpp"
#include "robot/robot_define.hpp"
#include "singleton.hpp"
#include "QuinticWalk/QuinticWalk.hpp"

namespace motion
{

    class WalkEngine: public subscriber, public singleton<WalkEngine>
    {
    public:
        WalkEngine();
        ~WalkEngine();
        void start();
        void stop();
        void set_params(float x, float y, float d, bool enable);
        void updata(const pub_ptr &pub, const int &type);

    private:
        Leph::QuinticWalk walk_;
        Leph::VectorLabel params_;
        double dt_;
        void run();
        std::thread td_;
        bool is_alive_;
        Eigen::Vector2d xrange, yrange, drange;
        Eigen::Vector3d walk_param_;
        bool walk_enable_;
        imu::imu_data imu_data_;
        
        mutable std::mutex param_mtx_, imu_mtx_, dxl_mtx_;
    };

#define WE WalkEngine::instance()
}

