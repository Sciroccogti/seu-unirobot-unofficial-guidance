#pragma once

#include <memory>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>

namespace imageproc
{
    cv::Mat buff2mat(unsigned char *buf640x480);
    inline cv::Vec3f BGR2HSI(unsigned char b, unsigned char g, unsigned char r)
    {
        float bn = b / 255.0f;
        float gn = g / 255.0f;
        float rn = r / 255.0f;

        float min_v = std::min(bn, std::min(gn, rn));
        float H, S, I;
        float eps = 0.000001;
        I = (bn + gn + rn) / 3.0f + eps;
        S = 1.0f - min_v / I;

        H = acos(0.5 * (rn - gn + rn - bn) / sqrt((rn - gn) * (rn - gn) + (rn - bn) * (gn - bn) + eps));

        if (bn > gn)
        {
            H = 2 * M_PI - H;
        }

        H = H * 180.0f / M_PI;
        S = S * 100.0f;
        I = I * 100.0f;
        return cv::Vec3f(H, S, I);
    }
}
