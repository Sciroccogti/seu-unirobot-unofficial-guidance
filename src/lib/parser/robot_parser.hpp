#pragma once

#include "basic_parser.hpp"
#include "robot/robot_define.hpp"
#include "class_exception.hpp"

namespace parser
{

    class robot_parser: public basic_parser
    {
    public:
        static robot::bone_ptr parse(const std::string &filename, robot::bone_map &bone_map, robot::joint_map &joint_map)
        {
            bpt::ptree pt;

            if (!get_tree_from_file(filename, pt))
            {
                throw class_exception<robot_parser>("Parser robot failed.");
            }

            return parse_bone(pt, bone_map, joint_map);
        }

    private:
        static robot::bone_ptr parse_bone(const bpt::ptree &pt, robot::bone_map &bone_map, robot::joint_map &joint_map)
        {
            robot::bone_ptr b = std::make_shared<robot::bone>();
            b->name_ = pt.get<std::string>("name");
            //std::cout<<b->name_<<"\n";
            b->length_ = pt.get<float>("length");
            bpt::ptree c_pt = pt.get_child("cp");
            std::vector<float> temp;
            temp.clear();

            for (auto &p : c_pt)
            {
                temp.push_back(p.second.get_value<float>());
            }

            b->cp_ << temp[0], temp[1], temp[2];

            c_pt = pt.get_child("cr");
            temp.clear();

            for (auto &r : c_pt)
            {
                temp.push_back(r.second.get_value<float>());
            }

            b->cr_ << temp[0], temp[1], temp[2];
            b->joints_.clear();
            bone_map[b->name_] = b;

            try
            {
                bpt::ptree cb = pt.get_child("joints");

                for (auto &j : cb)
                {
                    b->joints_.push_back(parse_joint(j.first, j.second, bone_map, joint_map));
                }
            }
            catch (bpt::ptree_error &e)
            {
                return b;
            }

            return b;
        }

        static robot::joint_ptr parse_joint(const std::string &j_name, const bpt::ptree &pt, robot::bone_map &bone_map, robot::joint_map &joint_map)
        {
            robot::joint_ptr j = std::make_shared<robot::joint>();
            robot::bone_ptr b = std::make_shared<robot::bone>();
            bpt::ptree cb;
            //std::cout<<j_name<<"\n";
            j->name_ = j_name;
            j->can_turn_ = pt.get<bool>("ct");

            bpt::ptree c_pt = pt.get_child("cp");
            std::vector<float> temp;
            temp.clear();

            for (auto &p : c_pt)
            {
                temp.push_back(p.second.get_value<float>());
            }

            j->cp_ << temp[0], temp[1], temp[2];
            c_pt = pt.get_child("cr");
            temp.clear();

            for (auto &r : c_pt)
            {
                temp.push_back(r.second.get_value<float>());
            }

            j->cr_ << temp[0], temp[1], temp[2];

            if (j->can_turn_)
            {
                j->init_deg_ = pt.get<float>("init");
                j->jid_ = pt.get<int>("jid");
                j->set_deg(pt.get<float>("cur"));

                try
                {
                    j->inverse_ = (pt.get<bool>("inver") ? -1.0 : 1.0);
                }
                catch (bpt::ptree_error &e)
                {
                    j->inverse_ = 1.0;
                }

                joint_map[j_name] = j;
            }

            try
            {
                j->next_bone_ = parse_bone(pt.get_child("bone"), bone_map, joint_map);
            }
            catch (bpt::ptree_error &e)
            {
                LOG << e.what() << ENDL;
            }

            return j;
        }
    };
}
