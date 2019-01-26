#pragma once

#include "basic_parser.hpp"
#include "robot/robot_define.hpp"
#include "class_exception.hpp"

namespace parser
{
#define acts_key_ "acts"
#define poses_key_ "poses"

    class action_parser: public basic_parser
    {
    public:
        static void parse(const std::string &act_file, std::map<std::string, robot::robot_act> &acts, std::map<std::string, robot::robot_pos> &poses)
        {
            bpt::ptree pt;
            acts.clear();
            poses.clear();

            if (!get_tree_from_file(act_file, pt))
            {
                throw class_exception<action_parser>("Parse act file failed");
            }

            bpt::ptree act_pt, pos_pt;

            try
            {
                act_pt = pt.get_child(acts_key_);
                pos_pt = pt.get_child(poses_key_);
            }
            catch (bpt::ptree_error &e)
            {
                throw class_exception<action_parser>("Parse act file failed");
            }

            robot::robot_pos t_pos;

            for (auto &pos : pos_pt)
            {
                t_pos.name.clear();
                t_pos.pose_info.clear();
                t_pos.name = pos.first;

                for (auto &info : pos.second)
                {
                    t_pos.pose_info[robot::get_motion_by_name(info.first)] = get_pose_from_tree(info.second);
                }

                poses[t_pos.name] = t_pos;
            }

            robot::robot_act t_act;
            robot::robot_one_pos t_one_pos;

            for (auto &act : act_pt)
            {
                t_act.name.clear();
                t_act.poses.clear();
                t_act.name = act.first;

                for (auto &pos : act.second)
                {
                    if (pos_exist(pos.first, poses))
                    {
                        t_one_pos.act_time = 0;
                        t_one_pos.pos_name.clear();
                        t_one_pos.pos_name = pos.first;
                        t_one_pos.act_time = pos.second.get_value<int>();
                        t_act.poses.push_back(t_one_pos);
                    }
                    else
                    {
                        throw class_exception<action_parser>("pos: " + pos.first + " does not exist");
                    }
                }

                acts[t_act.name] = t_act;
            }
        }

        static void save(const std::string &act_file, const std::map<std::string, robot::robot_act> &acts, const std::map<std::string, robot::robot_pos> &poses)
        {
            bpt::ptree pt;
            bpt::ptree act_pt, pos_pt;

            bpt::ptree act_info_child;

            for (auto &act : acts)
            {
                act_info_child.clear();

                for (size_t i = 0; i < act.second.poses.size(); i++)
                {
                    act_info_child.add<int>(act.second.poses[i].pos_name, act.second.poses[i].act_time);
                }

                act_pt.add_child(act.second.name, act_info_child);
            }

            pt.add_child(acts_key_, act_pt);
            bpt::ptree pos_info_child;

            for (auto &pos : poses)
            {
                pos_info_child.clear();

                for (auto &p_info : pos.second.pose_info)
                {
                    pos_info_child.add_child(robot::get_name_by_motion(p_info.first), get_tree_from_pose(p_info.second));
                }

                pos_pt.add_child(pos.second.name, pos_info_child);
            }

            pt.add_child(poses_key_, pos_pt);
            write_tree_to_file(act_file, pt);
        }

    private:
        static robot::robot_pose get_pose_from_tree(const bpt::ptree &pt)
        {
            robot::robot_pose pose;
            pose.x = pt.get<float>("x");
            pose.y = pt.get<float>("y");
            pose.z = pt.get<float>("z");
            pose.pitch = pt.get<float>("pitch");
            pose.roll = pt.get<float>("roll");
            pose.yaw = pt.get<float>("yaw");
            return pose;
        }

        static bpt::ptree get_tree_from_pose(const robot::robot_pose &pose)
        {
            bpt::ptree pt;
            pt.add("x", pose.x);
            pt.add("y", pose.y);
            pt.add("z", pose.z);
            pt.add("pitch", pose.pitch);
            pt.add("roll", pose.roll);
            pt.add("yaw", pose.yaw);
            return pt;
        }

        static bool pos_exist(const std::string &name, const std::map<std::string, robot::robot_pos> &poses)
        {
            if (poses.find(name) != poses.end())
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    };
}
