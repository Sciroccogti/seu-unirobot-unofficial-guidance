#pragma once

#include <eigen3/Eigen/Dense>
#include <cmath>

namespace robot_math
{

    inline double deg2rad(const double &x)
    {
        return x * M_PI / 180.0;
    }

    inline double rad2deg(const double &x)
    {
        return x * 180.0 / M_PI;
    }

    inline double cos_deg(const double &d)
    {
        return cos(deg2rad(d));
    }

    inline double sin_deg(const double &d)
    {
        return sin(deg2rad(d));
    }

    inline double tan_deg(const double &d)
    {
        return tan(deg2rad(d));
    }

    inline bool is_zero(const double &x)
    {
        return fabs(x) < 1E-4;
    }

    template <typename T>
    inline T sign(const T &x)
    {
        return (x >= 0) ? 1 : -1;
    }

    template <typename T>
    inline void bound(const T &min, const T &max, T &x)
    {
        if (max < min)
        {
            return;
        }

        if (x < min)
        {
            x = min;
        }

        if (x > max)
        {
            x = max;
        }
    }

    inline Eigen::Matrix3d RotX(const double &rad)
    {
        Eigen::AngleAxisd temp(rad, Eigen::Vector3d::UnitX());
        return temp.toRotationMatrix();
    }

    inline Eigen::Matrix3d RotY(const double &rad)
    {
        Eigen::AngleAxisd temp(rad, Eigen::Vector3d::UnitY());
        return temp.toRotationMatrix();
    }

    inline Eigen::Matrix3d RotZ(const double &rad)
    {
        Eigen::AngleAxisd temp(rad, Eigen::Vector3d::UnitZ());
        return temp.toRotationMatrix();
    }
}

