#pragma once

#include <thread>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include "singleton.hpp"

namespace motion
{
    class ScanEngine: public singleton<ScanEngine>
    {
    public:
        ScanEngine();
        ~ScanEngine();
        void start();
        void stop();
        void set_params(float yaw, float pitch, bool scan);

    private:
        void run();
        std::thread td_;
        bool is_alive_;
        float div_;
        float yaw_, pitch_;
        bool scan_;
        Eigen::Vector2f yaw_range_, pitch_range_;
        Eigen::Vector3f pitches_;
        mutable std::mutex param_mtx_;
    };

    #define SE ScanEngine::instance()
}