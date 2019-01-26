#pragma once

#include <mutex>
#include <list>
#include <map>
#include "robot/humanoid.hpp"
#include "singleton.hpp"
#include "worldmodel.hpp"

class adapter: public singleton<adapter>
{
public:
    enum act_mode
    {
        MODE_NONE = 0,
        MODE_WALK = 1,
        MODE_READY = 2,
        MODE_ACT = 3
    };
    adapter()
    {
        act_mode_ = MODE_NONE;
    }
    inline std::map<int, float> get_degs()
    {
        std::map<int, float> degs;
        degs.clear();
        bd_mutex_.lock();

        if (!body_degs_list.empty())
        {
            degs.insert(body_degs_list.front().begin(), body_degs_list.front().end());
            body_degs_list.pop_front();
        }

        bd_mutex_.unlock();
        hd_mutex_.lock();

        if (!head_degs_list.empty())
        {
            degs.insert(head_degs_list.front().begin(), head_degs_list.front().end());
            head_degs_list.pop_front();
        }

        hd_mutex_.unlock();
        return degs;
    }
    inline std::map<int, float> get_body_degs() const
    {
        bd_mutex_.lock();
        std::map<int, float> res = latest_body_deg;
        bd_mutex_.unlock();
        return res;
    }
    inline std::map<int, float> get_head_degs() const
    {
        hd_mutex_.lock();
        std::map<int, float> res = latest_head_deg;
        hd_mutex_.unlock();
        return res;
    }
    inline bool add_body_degs(const std::map<int, float> &jdmap)
    {
        if (!is_alive_)
        {
            return false;
        }

        bd_mutex_.lock();
        body_degs_list.push_back(jdmap);
        latest_body_deg = jdmap;
        bd_mutex_.unlock();
        return true;
    }
    inline bool add_head_degs(std::map<int, float> &jdmap)
    {
        if (!is_alive_)
        {
            return false;
        }
        if(WM->fall_data()!=FALL_NONE)
        {
            for(auto &jd:jdmap)
            {
                jd.second = 0.0;
            }
        }
        hd_mutex_.lock();
        head_degs_list.push_back(jdmap);
        latest_head_deg = jdmap;
        hd_mutex_.unlock();
        return true;
    }

    inline bool body_empty() const
    {
        bd_mutex_.lock();
        bool res = body_degs_list.empty();
        bd_mutex_.unlock();
        return res;
    }

    inline bool head_empty() const
    {
        hd_mutex_.lock();
        bool res = head_degs_list.empty();
        hd_mutex_.unlock();
        return res;
    }

    void start()
    {
        robot::joint_ptr j = robot::ROBOT->get_joint("jhead1");
        latest_head_deg[j->jid_] = j->get_deg();
        j = robot::ROBOT->get_joint("jhead2");
        latest_head_deg[j->jid_] = j->get_deg();

        for (auto &j : robot::ROBOT->get_joint_map())
        {
            if (j.second->name_.find("head") == std::string::npos)
            {
                latest_body_deg[j.second->jid_] = j.second->get_deg();
            }
        }

        is_alive_ = true;
    }
    void stop()
    {
        is_alive_ = false;
    }
    act_mode mode() const
    {
        return act_mode_;
    }
    act_mode &mode()
    {
        return act_mode_;
    }
private:
    std::list< std::map<int, float> > head_degs_list;
    std::list< std::map<int, float> > body_degs_list;
    std::map<int, float> latest_head_deg;
    std::map<int, float> latest_body_deg;
    mutable std::mutex bd_mutex_, hd_mutex_;
    bool is_alive_;
    act_mode act_mode_;
};

#define MADT adapter::instance()

