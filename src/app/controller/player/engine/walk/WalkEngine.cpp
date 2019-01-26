#include "WalkEngine.hpp"
#include "robot/humanoid.hpp"
#include "math/math.hpp"
#include "configuration.hpp"
#include <cmath>
#include <fstream>
#include "core/adapter.hpp"
#include "sensor/motor.hpp"
#include "core/worldmodel.hpp"
#include "QuinticWalk/SmoothSpline.hpp"

namespace motion
{
    using namespace Eigen;
    using namespace robot;
    using namespace robot_math;
    using namespace Leph;

    WalkEngine::WalkEngine()
        : walk_param_(0.0, 0.0, 0.0), walk_enable_(false)
    {
        std::vector<double> range = CONF->get_config_vector<double>("walk.x");
        xrange[0] = range[0];
        xrange[1] = range[1];
        range = CONF->get_config_vector<double>("walk.y");
        yrange[0] = range[0];
        yrange[1] = range[1];
        range = CONF->get_config_vector<double>("walk.dir");
        drange[0] = range[0];
        drange[1] = range[1];
        params_ = walk_.getParameters();
        dt_ = 0.0;
    }

    void WalkEngine::updata(const pub_ptr &pub, const int &type)
    {
        if (type == sensor::SENSOR_IMU)
        {
            imu_mtx_.lock();
            std::shared_ptr<imu> sptr = std::dynamic_pointer_cast<imu>(pub);
            imu_data_ = sptr->data();
            imu_mtx_.unlock();
            return;
        }

        if (type == sensor::SENSOR_MOTOR)
        {
            dxl_mtx_.lock();
            std::shared_ptr<motor> sptr = std::dynamic_pointer_cast<motor>(pub);
            dxl_mtx_.unlock();
            return;
        }
    }

    WalkEngine::~WalkEngine()
    {
        if (td_.joinable())
        {
            td_.join();
        }
        LOG << std::setw(12) << "engine:" << std::setw(18) << "[WalkEngine]" << " ended!" << ENDL;
    }

    void WalkEngine::start()
    {
        LOG << std::setw(12) << "engine:" << std::setw(18) << "[WalkEngine]" << " started!" << ENDL;
        dt_ = 1.0 / (1000.0 / CONF->get_config_value<double>("hardware.motor.period"));
        is_alive_ = true;
        td_ = std::move(std::thread(&WalkEngine::run, this));
    }

    void WalkEngine::stop()
    {
        is_alive_ = false;
    }

    void WalkEngine::set_params(float x, float y, float d, bool enable)
    {
        param_mtx_.lock();
        walk_param_.x() = x;
        walk_param_.y() = y;
        walk_param_.z() = d;
        walk_enable_ = enable;
        bound(xrange[0], xrange[1], walk_param_.x());
        bound(yrange[0], yrange[1], walk_param_.y());
        bound(drange[0], drange[1], walk_param_.z());
        walk_param_.z() = deg2rad(walk_param_.z());
        param_mtx_.unlock();
    }

    void WalkEngine::run()
    {
        transform_matrix body_mat, support_mat, swing_mat;
        Vector3d lefthand, righthand;
        std::vector<double> degs;
        std::map<int, float> jdegs;
        double phaseLeft, phaseRight;
        double handGain = 0.1;
        Vector3d param;
        bool enable;
        double lastphase = 0.0;
        double currphase = 0.0;
        support_foot supportFoot = DOUBLE_SUPPORT;

        SmoothSpline handSpline;
        handSpline.addPoint(0.0, 0.0);
        handSpline.addPoint(0.25, 1.0);
        handSpline.addPoint(0.5, 0.0);
        handSpline.addPoint(0.75, -1.0);
        handSpline.addPoint(1.0, 0.0);

        while (is_alive_)
        {
            param_mtx_.lock();
            param = walk_param_;
            enable = walk_enable_;
            param_mtx_.unlock();

            if (MADT->mode() == adapter::MODE_READY || MADT->mode() == adapter::MODE_WALK)
            {
                if (MADT->mode() == adapter::MODE_READY)
                {
                    param.x() = 0.0;
                    param.y() = 0.0;
                    param.z() = 0.0;
                }

                walk_.setOrders(param, enable);

                while (is_alive_)
                {
                    walk_.update(dt_);
                    currphase = walk_.getPhase();
                    phaseLeft = currphase;
                    phaseRight = currphase + 0.5;

                    if (phaseRight > 1.0)
                    {
                        phaseRight -= 1.0;
                    }

                    if (currphase < 0.5 && lastphase > 0.5)
                    {
                        lastphase = currphase;
                        break;
                    }

                    lastphase = currphase;
                    walk_.assignModel(supportFoot, body_mat, swing_mat);

                    if (enable)
                    {
                        if (ROBOT->leg_inverse_kinematics(body_mat, (supportFoot == RIGHT_SUPPORT ? swing_mat : support_mat), degs, true))
                        {
                            jdegs[ROBOT->get_joint("jlhip3")->jid_] = rad2deg(degs[0]);
                            jdegs[ROBOT->get_joint("jlhip2")->jid_] = rad2deg(degs[1]);
                            jdegs[ROBOT->get_joint("jlhip1")->jid_] = rad2deg(degs[2]);
                            jdegs[ROBOT->get_joint("jlknee")->jid_] = rad2deg(degs[3]);
                            jdegs[ROBOT->get_joint("jlankle2")->jid_] = rad2deg(degs[4]);
                            jdegs[ROBOT->get_joint("jlankle1")->jid_] = rad2deg(degs[5]);

                        }
                        else
                        {
                            LOG << currphase << '\t' << "left leg_inverse_kinematics faied!" << ENDL;
                        }

                        if (ROBOT->leg_inverse_kinematics(body_mat, (supportFoot == LEFT_SUPPORT ? swing_mat : support_mat) , degs, false))
                        {
                            jdegs[ROBOT->get_joint("jrhip3")->jid_] = rad2deg(degs[0]);
                            jdegs[ROBOT->get_joint("jrhip2")->jid_] = rad2deg(degs[1]);
                            jdegs[ROBOT->get_joint("jrhip1")->jid_] = rad2deg(degs[2]);
                            jdegs[ROBOT->get_joint("jrknee")->jid_] = rad2deg(degs[3]);
                            jdegs[ROBOT->get_joint("jrankle2")->jid_] = rad2deg(degs[4]);
                            jdegs[ROBOT->get_joint("jrankle1")->jid_] = rad2deg(degs[5]);
                        }
                        else
                        {
                            LOG << currphase << '\t' << "right leg_inverse_kinematics faied!" << ENDL;
                        }
                        handGain = fabs(walk_param_.x()*2.5);
                        lefthand[0] =  handGain * handSpline.pos(phaseLeft);
                        righthand[0] = handGain * handSpline.pos(phaseRight);
                        lefthand[2] = 0.1;
                        righthand[2] = 0.1;

                        if (ROBOT->arm_inverse_kinematics(lefthand, degs))
                        {
                            jdegs[ROBOT->get_joint("jlshoulder1")->jid_] = rad2deg(degs[0]);
                            jdegs[ROBOT->get_joint("jlelbow")->jid_] = -rad2deg(degs[2]);
                        }

                        if (ROBOT->arm_inverse_kinematics(righthand, degs))
                        {
                            jdegs[ROBOT->get_joint("jrshoulder1")->jid_] = rad2deg(degs[0]);
                            jdegs[ROBOT->get_joint("jrelbow")->jid_] = rad2deg(degs[2]);
                        }

                        while (!MADT->body_empty())
                        {
                            usleep(1000);
                        }

                        if (!MADT->add_body_degs(jdegs))
                        {
                            break;
                        }
                    }
                }

                if (MADT->mode() == adapter::MODE_READY)
                {
                    MADT->mode() = adapter::MODE_ACT;
                }
            }
            usleep(1000);
        }
    }
}