#pragma once

#include <boost/program_options.hpp>
#include "singleton.hpp"

class options: public singleton<options>
{
public:
    enum robot_choice
    {
        ROBOT_NONE = 0,
        ROBOT_REAL = 1,
        ROBOT_VIRTUAL = 2
    };

    options();

    bool init(int argc, char *argv[]);
    int id() const
    {
        return id_;
    }
    bool use_debug() const
    {
        return use_debug_;
    }
    bool use_camera() const
    {
        return use_camera_;
    }
    bool use_robot() const
    {
        return use_robot_;
    }
    bool use_remote() const
    {
        return use_remote_;
    }

private:
    template<typename T> T arg(const std::string &opt) const
    {
        return var_map_[opt].as<T>();
    }
    boost::program_options::options_description opts_desc_;
    boost::program_options::variables_map var_map_;
    int id_;
    bool use_debug_;
    bool use_camera_;
    bool use_robot_;
    bool use_remote_;
};

#define OPTS options::instance()

