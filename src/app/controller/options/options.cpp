#include "options.hpp"
#include <iostream>
#include "common.hpp"

using namespace std;
using namespace boost::program_options;

options::options(): opts_desc_("  Options description")
{
    opts_desc_.add_options()
    ("help,h", "Print this message and exit.")
    ("player,p", value<int>()->default_value(0),
     "Player ID number.")
    ("debug,d", value<bool>()->default_value(false),
     "If you want to start in debug mode.")
    ("camera,c", value<bool>()->default_value(true),
     "If you want to use camera.")
    ("robot,r", value<bool>()->default_value(true),
     "If you want to use robot")
    ("mote,m", value<bool>()->default_value(false),
     "If you want to use remote.");
}

bool options::init(int argc, char **argv)
{
    try
    {
        store(parse_command_line(argc, argv, opts_desc_), var_map_);
        id_ = arg<int>("player");
        use_debug_ = arg<bool>("debug");
        use_camera_ = arg<bool>("camera");
        use_robot_ = arg<bool>("robot");
        use_remote_ = arg<bool>("mote");

        if (var_map_.count("help"))
        {
            LOG << '\n' << opts_desc_ << ENDL;
            return false;
        }
    }
    catch (boost::program_options::unknown_option &e)
    {
        LOG << e.what() << ENDL;
    }
    catch (std::exception &e)
    {
        LOG << e.what() << ENDL;
        return false;
    }

    return true;
}