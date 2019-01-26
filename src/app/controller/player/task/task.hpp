#pragma once

#include <memory>
#include <string>
#include <map>
#include <vector>

class task
{
public:
    task(const std::string &name)
        : name_(name)
    {

    }

    std::string name() const
    {
        return name_;
    }

    virtual bool perform() = 0;

private:
    std::string name_;
};

typedef std::shared_ptr<task> task_ptr;
