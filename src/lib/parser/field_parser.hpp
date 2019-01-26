#pragma once

#include <map>
#include "basic_parser.hpp"
#include "model.hpp"

namespace parser
{
    class field_parser: public basic_parser
    {
    public:
        static bool parse(const std::string &filename, filed_info &field)
        {
            bpt::ptree pt;

            if (!get_tree_from_file(filename, pt))
            {
                return false;
            }

            field.field_length = pt.get<int>("field_length");
            field.field_width = pt.get<int>("field_width");
            field.goal_depth = pt.get<int>("goal_depth");
            field.goal_width = pt.get<int>("goal_width");
            field.goal_height = pt.get<int>("goal_height");
            field.goal_area_length = pt.get<int>("goal_area_length");
            field.goal_area_width = pt.get<int>("goal_area_width");
            field.penalty_mark_distance = pt.get<int>("penalty_mark_distance");
            field.center_circle_diameter = pt.get<int>("center_circle_diameter");
            field.border_strip_width_min = pt.get<int>("border_strip_width_min");
            return true;
        }
    };
}

