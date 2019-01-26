#pragma once

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace parser
{
    namespace bpt = boost::property_tree;

    class basic_parser
    {
    protected:
        static bool get_tree_from_file(const std::string &filename, bpt::ptree &pt)
        {
            std::ifstream ifs(filename.c_str(), std::ios::in);
            std::string line;
            int count_of_quotatuion = 0;
            std::stringstream json_data;

            if (!ifs)
            {
                return false;
            }

            while (std::getline(ifs, line))
            {
                count_of_quotatuion = 0;

                for (int i = 0; i < line.size(); i++)
                {
                    if (line[i] == '\'' || line[i] == '\"')
                    {
                        count_of_quotatuion++;
                    }

                    if (i < line.size() - 2)
                    {
                        if (line[i] == '/' && line[i + 1] == '/' && count_of_quotatuion % 2 == 0)
                        {
                            break;
                        }
                    }

                    json_data << line[i];
                }

                line.clear();
            }

            bpt::read_json(json_data, pt);
            return true;
        }

        static void write_tree_to_file(const std::string &filename, const bpt::ptree &pt)
        {
            std::ostringstream os;
            bpt::write_json(os, pt);
            std::ofstream tree(filename.c_str());
            tree << os.str();
            tree.close();
        }
    };
}
