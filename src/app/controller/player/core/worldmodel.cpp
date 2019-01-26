#include "worldmodel.hpp"

using namespace Eigen;
using namespace robot_math;
using namespace robot;


WorldModel::WorldModel()
{
    fall_direction_ = FALL_NONE;
    support_foot_ = robot::DOUBLE_SUPPORT;
    lost_ = false;
    bodyx_ = -1.0;
    bodyy_ = 0.0;
    bodydir_ = 15.0;
    ballx_ = 1.0;
    bally_ = -1.0;
}

void WorldModel::updata(const pub_ptr &pub, const int &type)
{
    if (type == sensor::SENSOR_IMU)
    {
        imu_mtx_.lock();
        std::shared_ptr<imu> sptr = std::dynamic_pointer_cast<imu>(pub);
        imu_data_ = sptr->data();
        Eigen::AngleAxisd roll, pitch, yaw;
        yaw = AngleAxisd(deg2rad(imu_data_.yaw), Vector3d::UnitZ());
        pitch = AngleAxisd(deg2rad(imu_data_.pitch), Vector3d::UnitY());
        roll = AngleAxisd(deg2rad(imu_data_.roll), Vector3d::UnitX());
        lost_ = sptr->lost();
        fall_direction_ = sptr->fall_direction();
        imu_mtx_.unlock();

        Quaternion<double> quat = roll * pitch * yaw;
        dxl_mtx_.lock();
        body_matrix_.set_R(quat.matrix());
        head_mtx_.lock();
        head_matrix_ = body_matrix_*transform_matrix(0, 0, ROBOT->trunk_length())*transform_matrix(head_degs_[0],'z')
                        *transform_matrix(0, 0, ROBOT->neck_length())*transform_matrix(head_degs_[1], 'y')
                        *transform_matrix(0,0,ROBOT->head_length());
        head_mtx_.unlock();
        dxl_mtx_.unlock();
        return;
    }

    if (type == sensor::SENSOR_MOTOR)
    {
        dxl_mtx_.lock();
        std::shared_ptr<motor> sptr = std::dynamic_pointer_cast<motor>(pub);
        body_matrix_ = ROBOT->leg_forward_kinematics(ROBOT->get_foot_degs(support_foot_),
                        support_foot_== LEFT_SUPPORT);
        head_degs_ = ROBOT->get_head_degs();
        dxl_mtx_.unlock();
        return;
    }

    if(type == sensor::SENSOR_BUTTON)
    {
        std::shared_ptr<button> sptr = std::dynamic_pointer_cast<button>(pub);
        bt1_status_ = sptr->button_1();
        bt2_status_ = sptr->button_2();
        return;
    }
}