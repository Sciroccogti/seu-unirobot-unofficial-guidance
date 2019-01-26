#pragma once

#include <ctime>
#include <cstdio>
#include <string>
#include <iostream>
#include <iomanip>

#define LOG std::cout<<"\033[32m[controller]\t"
#define ENDL "\033[0m"<<std::endl;

inline std::string get_time()
{
    time_t timep;
    std::time(&timep);
    char tmp[64];
    std::strftime(tmp, sizeof(tmp), "%Y-%m-%d_%H%M%S", std::localtime(&timep));
    return std::string(tmp);
}

