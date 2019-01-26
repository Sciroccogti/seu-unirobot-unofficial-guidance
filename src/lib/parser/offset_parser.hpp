#pragma once

#include <map>
#include "basic_parser.hpp"
#include "robot/robot_define.hpp"

namespace parser
{
    class offset_parser: public basic_parser
    {
    public:
        static void parse(const std::string &filename, robot::joint_map &jmap)
        {
            bpt::ptree pt;

            if (!get_tree_from_file(filename, pt))
            {
                return;
            }

            for (auto &offset : pt)
            {
                jmap[offset.first]->offset_ = offset.second.get_value<float>();
            }
        }

        static void save(const std::string &filename, const robot::joint_map &jmap)
        {
            bpt::ptree pt;

            for (auto &j : jmap)
            {
                pt.add(j.first, j.second->offset_);
            }

            write_tree_to_file(filename, pt);
        }
    };
}
