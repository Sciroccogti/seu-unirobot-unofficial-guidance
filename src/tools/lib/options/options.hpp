#pragma once

#include <boost/program_options.hpp>
#include "singleton.hpp"
#include "class_exception.hpp"

class options: public singleton<options>
{
public:
    options();
    bool init(int argc, char *argv[]);
    int id() const
    {
        return id_;
    }

private:
    template<typename T>
    T arg(const std::string &opt) const
    {
        return var_map_[opt].as<T>();
    }
    boost::program_options::options_description opts_desc_;
    boost::program_options::variables_map var_map_;
    int id_;
};

#define OPTS options::instance()

