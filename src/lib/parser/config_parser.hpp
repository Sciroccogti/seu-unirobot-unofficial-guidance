#pragma once

#include "basic_parser.hpp"
#include "class_exception.hpp"

namespace parser
{
    namespace bpt = boost::property_tree;

    class config_parser: public basic_parser
    {
    public:
        static bool parse(const std::string &cfgname, bpt::ptree &pt)
        {
            return parse_file(cfgname, pt);
        }

        static void save(const std::string &filename, const bpt::ptree &pt)
        {
            write_tree_to_file(filename, pt);
        }
    private:
        static bool add_child(bpt::ptree &oript, const std::string &key, const bpt::ptree &pt)
        {
            try
            {
                oript.erase(key);
                oript.add_child(key, pt);
            }
            catch (bpt::ptree_error &e)
            {
                return false;
            }

            return true;
        }

        static bool parse_file(const std::string &cfgname, bpt::ptree &pt)
        {
            if (get_tree_from_file(cfgname, pt))
            {
                return parse_ptree(pt);
            }
            else
            {
                return false;
            }
        }

        static bool parse_ptree(bpt::ptree &pt)
        {
            std::string data;
            bpt::ptree tpt;
            bpt::ptree oript = pt;
            int size = oript.size();

            if (size == 1)
            {
                data.clear();
                data = oript.begin()->second.data();

                if (data.size() == 0)
                {
                    tpt = oript.begin()->second;

                    if (parse_ptree(tpt))
                    {
                        if (!add_child(pt, oript.begin()->first, tpt))
                        {
                            return false;
                        }
                        else
                        {
                            return true;
                        }
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    if (oript.begin()->first.empty())
                    {
                        if (parse_file(data, tpt))
                        {
                            pt = tpt;
                        }
                        else
                        {
                            return false;
                        }

                        return true;
                    }
                }
            }

            auto iter = oript.begin();

            while (iter != oript.end())
            {
                tpt = iter->second;
                data.clear();
                data = tpt.data();

                if (data.size() == 0)
                {
                    if (parse_ptree(tpt))
                    {
                        if (!add_child(pt, iter->first, tpt))
                        {
                            return false;
                        }
                    }
                    else
                    {
                        return false;
                    }
                }

                iter++;
            }

            return true;
        }
    };
}
