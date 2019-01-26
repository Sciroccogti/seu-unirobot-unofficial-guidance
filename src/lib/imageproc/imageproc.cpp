#include "imageproc.hpp"

using namespace cv;
using namespace std;

namespace imageproc
{

    Mat buff2mat(unsigned char *buf640x480)
    {
        Mat dst640x480x3;

        dst640x480x3.create(480, 640, CV_8UC3);

        unsigned char *src_ptr, *dst_ptr;
        unsigned short dst_offset, src_offset;
        unsigned short width = 640, height = 480;

        for (unsigned short y = 0; y < height; y++)
        {
            src_ptr = buf640x480 + y * (width * 2);
            dst_ptr = dst640x480x3.data + y * (width * 3);

            for (unsigned short x = 0; x < width / 2; x++)
            {
                dst_offset = x * 6;
                src_offset = x * 4;
                *(dst_ptr + dst_offset + 0) = *(src_ptr + src_offset + 0);
                *(dst_ptr + dst_offset + 1) = *(src_ptr + src_offset + 1);
                *(dst_ptr + dst_offset + 2) = *(src_ptr + src_offset + 3);
                *(dst_ptr + dst_offset + 3) = *(src_ptr + src_offset + 2);
                *(dst_ptr + dst_offset + 4) = *(src_ptr + src_offset + 1);
                *(dst_ptr + dst_offset + 5) = *(src_ptr + src_offset + 3);
            }
        }

        return dst640x480x3;
    }
}

