#pragma once

#include <map>
#include <thread>
#include <linux/videodev2.h>
#include <CameraApi.h>
#include "sensor.hpp"
#include "model.hpp"
#include "configuration.hpp"

class camera: public sensor
{
public:
    camera();
    ~camera();

    bool start();
    void run();
    void stop();
    bool open();
    void close();

    void set_camera_info(const camera_info &para);
    inline unsigned char *buffer() const
    {
        if (use_mv_)
        {
            return buffer_;
        }
        else
        {
            return buffers_[num_bufs_].start;
        }
    }

    inline int camera_w() const
    {
        return w_;
    }

    inline int camera_h() const
    {
        return h_;
    }

    inline bool use_mv() const
    {
        return use_mv_;
    }

    inline int camera_size() const
    {
        if (use_mv_)
        {
            return w_ * h_;
        }
        else
        {
            return w_ * h_ * 2;
        }
    }
private:
    struct VideoBuffer
    {
        unsigned char *start;
        size_t offset;
        size_t length;
        size_t bytesused;
        int lagtimems;
    };
    bool use_mv_;
    std::thread td_;
    VideoBuffer *buffers_;
    v4l2_buffer buf_;
    unsigned int num_bufs_;
    int fd_;
    int w_;
    int h_;
    unsigned char *buffer_;
    tSdkCameraCapbility     tCapability_;
    tSdkFrameHead           sFrameInfo_;
    std::map<std::string, camera_info> camera_infos_;
};
