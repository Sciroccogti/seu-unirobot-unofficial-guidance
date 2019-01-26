#pragma once

#include "darknet/network.h"

bool CompareDetGreater(const detection &a, const detection &b)
{
    return a.prob[0]>b.prob[0];
};

bool CompareDetLess(const detection &a, const detection &b)
{
    return a.prob[0]<b.prob[0];
};