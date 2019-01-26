#pragma once

#include "task.hpp"
#include "engine/action/ActionEngine.hpp"

class action_task: public task
{
public:
    action_task(const std::string &name)
        : act_name_(name), task("action")
    {
        
    }

    bool perform()
    {
        std::vector< std::map<robot::robot_motion, robot::robot_pose> > poses_;
        std::vector<int> pos_times_;
        auto aiter = robot::ROBOT->get_act_map().find(act_name_);

        if (aiter == robot::ROBOT->get_act_map().end())
        {
            LOG << "cannot find action: " + act_name_ << ENDL;
            return false;
        }
        else
        {
            std::vector<robot::robot_one_pos> poses = aiter->second.poses;
            for (int i = 0; i < poses.size(); i++)
            {
                pos_times_.push_back(poses[i].act_time);
                poses_.push_back(robot::ROBOT->get_pos_map()[poses[i].pos_name].pose_info);
            }
            motion::AE->set_params(poses_, pos_times_);
            return true;
        }
    }

private:
    std::string act_name_;
};