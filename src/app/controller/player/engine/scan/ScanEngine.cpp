#include "ScanEngine.hpp"
#include "configuration.hpp"
#include "robot/humanoid.hpp"
#include "core/adapter.hpp"
#include <cmath>

using namespace std;
using namespace robot;

namespace motion
{
ScanEngine::ScanEngine()
{
    std::vector<float> range = CONF->get_config_vector<float>("scan.pitch");
    pitch_range_[0] = range[0];
    pitch_range_[1] = range[1];
    range = CONF->get_config_vector<float>("scan.yaw");
    yaw_range_[0] = range[0];
    yaw_range_[1] = range[1];
    div_ = CONF->get_config_value<float>("scan.div");
    yaw_ = 0.0;
    pitch_ = 0.0;
    scan_ = false;
    pitches_[0] = pitch_range_[0];
    pitches_[1] = pitch_range_[0]+(pitch_range_[1]-pitch_range_[0])/2.0f;
    pitches_[2] = pitch_range_[1];
}

ScanEngine::~ScanEngine()
{
    if(td_.joinable())
    {
        td_.join();
    }
    LOG << std::setw(12) << "engine:" << std::setw(18) << "[ScanEngine]" << " ended!" << ENDL;
}

void ScanEngine::start()
{
    LOG << std::setw(12) << "engine:" << std::setw(18) << "[ScanEngine]" << " started!" << ENDL;
    is_alive_ = true;
    td_ = std::move(std::thread(&ScanEngine::run, this));
}

void ScanEngine::stop()
{
    is_alive_ = false;
}

void ScanEngine::set_params(float yaw, float pitch, bool scan)
{
    param_mtx_.lock();
    yaw_ = yaw;
    pitch_ = pitch;
    scan_ = scan;
    param_mtx_.unlock();
}

void ScanEngine::run()
{
    unsigned int line = 0;
    int id_yaw = ROBOT->get_joint("jhead1")->jid_;
    int id_pitch = ROBOT->get_joint("jhead2")->jid_;
    std::map<int, float> jdmap;
    jdmap[id_yaw] = 0.0;
    jdmap[id_pitch] = 0.0;
    float yaw, pitch;
    bool scan;
    while(is_alive_)
    {
        param_mtx_.lock();
        yaw = yaw_;
        pitch = pitch_;
        scan = scan_;
        param_mtx_.unlock();
        if(scan)
        {
            int idx = line%4-2;
            if(idx<0) idx = -idx;
            jdmap[id_pitch] = pitches_[idx];
            float s = pow(-1, idx);
            for(float yawt = yaw_range_[0]; yawt <= yaw_range_[1]&&is_alive_; yawt += div_)
            {
                jdmap[id_yaw] = s*yawt;
                while (!MADT->head_empty())
                {
                    usleep(1000);
                }
                if (!MADT->add_head_degs(jdmap))
                {
                    break;
                }
            }
            line++;
        }
        else
        {
            jdmap[id_yaw] = yaw;
            jdmap[id_pitch] = pitch;
            while (!MADT->head_empty())
            {
                usleep(1000);
            }
            if (!MADT->add_head_degs(jdmap))
            {
                break;
            }
        }
        usleep(10000);
    }
}
}