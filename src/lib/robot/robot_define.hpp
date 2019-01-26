#pragma once

#include <string>
#include <list>
#include <memory>
#include <map>
#include <eigen3/Eigen/Dense>

namespace robot
{
    class bone;
    class joint;

    typedef std::shared_ptr<joint> joint_ptr;
    typedef std::shared_ptr<bone> bone_ptr;
    typedef std::map<std::string, joint_ptr> joint_map;
    typedef std::map<std::string, bone_ptr> bone_map;

    class bone
    {
    public:
        std::string name_;
        float length_;
        Eigen::Vector3f cp_;
        Eigen::Vector3f cr_;
        std::list<joint_ptr> joints_;
    };

    class joint
    {
    public:
        joint()
        {
            jid_ = 0;
            can_turn_ = true;
            inverse_ = 1.0;
            init_deg_ = 0.0f;
            current_deg_ = 0.0f;
            offset_ = 0.0f;
        }

        bool set_deg(const float &deg)
        {
            current_deg_ = deg;
            return true;
        }

        float get_deg() const
        {
            return current_deg_;
        }

        joint &operator = (const joint &j)
        {
            jid_ = j.jid_;
            name_ = j.name_;
            can_turn_ = j.can_turn_;
            next_bone_ = j.next_bone_;
            init_deg_ = j.init_deg_;
            current_deg_ = j.current_deg_;
            offset_ = j.offset_;
            inverse_ = j.inverse_;
            return *this;
        }

        int jid_;
        std::string name_;
        bool can_turn_;
        bone_ptr next_bone_;
        float init_deg_;
        float inverse_;
        float offset_;
        Eigen::Vector3f cp_;
        Eigen::Vector3f cr_;
    private:
        float current_deg_;
    };

    enum robot_motion
    {
        MOTION_NONE         = 0,
        MOTION_RIGHT_HAND   = 1,
        MOTION_LEFT_HAND    = 2,
        MOTION_BODY         = 3,
        MOTION_RIGHT_FOOT   = 4,
        MOTION_LEFT_FOOT    = 5,
        MOTION_HEAD         = 6
    };

    enum support_foot
    {
        DOUBLE_SUPPORT = 0,
        LEFT_SUPPORT = 1,
        RIGHT_SUPPORT = 2
    };

    static const std::map<std::string, robot_motion> name_motion_map = {{"none", MOTION_NONE},
        {"right_hand", MOTION_RIGHT_HAND},
        {"left_hand", MOTION_LEFT_HAND},
        {"body", MOTION_BODY},
        {"right_foot", MOTION_RIGHT_FOOT},
        {"left_foot", MOTION_LEFT_FOOT},
        {"head", MOTION_HEAD}
    };

    inline std::string get_name_by_motion(const robot_motion &motion)
    {
        for (auto &nm : name_motion_map)
        {
            if (nm.second == motion)
            {
                return nm.first;
            }
        }

        return "";
    }

    inline robot_motion get_motion_by_name(const std::string &name)
    {
        for (auto &nm : name_motion_map)
        {
            if (nm.first == name)
            {
                return nm.second;
            }
        }

        return MOTION_NONE;
    }

    struct robot_pose
    {
        float x, y, z;
        float pitch, roll, yaw;
    };

    struct robot_pos
    {
        std::string name;
        std::map<robot_motion, robot_pose> pose_info;
    };

    struct robot_one_pos
    {
        std::string pos_name;
        int act_time;
    };

    struct robot_act
    {
        std::string name;
        std::vector<robot_one_pos> poses;
    };

    struct robot_joint_deg
    {
        int id;
        float deg;
    };

    typedef std::map<std::string, robot_act> act_map;
    typedef std::map<std::string, robot_pos> pos_map;
}
