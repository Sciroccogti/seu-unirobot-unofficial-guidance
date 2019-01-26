#pragma once

#include <thread>
#include <mutex>
#include "robot/robot_define.hpp"
#include "singleton.hpp"
#include "math/math.hpp"

namespace motion
{
class ActionEngine: public singleton<ActionEngine>
{
public:
    ActionEngine();
    ~ActionEngine();
    void start();
    void stop();
    void set_params(const std::vector< std::map<robot::robot_motion, robot::robot_pose> > &poses, const std::vector<int> &pos_times);
private:
    void run();
    std::vector< std::map<robot::robot_motion, robot::robot_pose> > get_poses(std::map<robot::robot_motion, robot::robot_pose> &pos1,
            std::map<robot::robot_motion, robot::robot_pose> &pos2, int act_time);
    bool get_degs(const robot_math::transform_matrix &body_mat, const robot_math::transform_matrix &leftfoot_mat,
            const robot_math::transform_matrix &rightfoot_mat, const Eigen::Vector3d &lefthand, const Eigen::Vector3d &righthand,
            std::map<int, float> &degs_map);
    bool set_joints(std::map<int, float> &jdegs, int act_time);
    std::string act_name_;
    std::vector< std::map<robot::robot_motion, robot::robot_pose> > poses_;
    std::vector<int> pos_times_;
    double step_;
    bool is_alive_;
    std::thread td_;
    mutable std::mutex param_mtx_;
};
#define AE ActionEngine::instance()
}