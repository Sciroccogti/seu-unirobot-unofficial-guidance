#pragma once

#include <eigen3/Eigen/Dense>
#include "task.hpp"
#include "engine/walk/WalkEngine.hpp"
#include "core/adapter.hpp"

class walk_task: public task
{
public:
    walk_task(float x, float y, float dir, bool enable)
        : x_(x), y_(y), d_(dir), e_(enable), task("walk")
    {

    }

    bool perform()
    {
        MADT->mode() = adapter::MODE_WALK;
        motion::WE->set_params(x_, y_, d_, e_);
        return true;
    }
private:
    float x_, y_, d_;
    bool e_;
};