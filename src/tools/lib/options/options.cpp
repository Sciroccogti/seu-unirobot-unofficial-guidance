#include "options.hpp"

using namespace boost::program_options;

options::options(): opts_desc_("  Options description")
{
    opts_desc_.add_options()
    ("help,h", "Print this message and exit.")
    ("player,p", value<int>()->default_value(0), "Player ID number.");
}

bool options::init(int argc, char *argv[])
{
    try
    {
        store(parse_command_line(argc, argv, opts_desc_), var_map_);
        id_ = arg<int>("player");

        if (var_map_.count("help"))
        {
            std::cout << opts_desc_ << "\n";
            return false;
        }
    }
    catch (boost::program_options::unknown_option &e)
    {
        std::cout << e.what() << "\n";
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << "\n";
        return false;
    }

    return true;
}
