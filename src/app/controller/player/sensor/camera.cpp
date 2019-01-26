#include <sys/mman.h>
#include <errno.h>
#include <functional>
#include "camera.hpp"
#include "parser/camera_parser.hpp"
#include "configuration.hpp"
#include "class_exception.hpp"
#include <sstream>
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace std;

camera::camera(): sensor("camera")
{
    parser::camera_info_parser::parse(CONF->get_config_value<string>(CONF->player() + ".camera_info_file"), camera_infos_);
}

bool camera::start()
{
    if (!this->open())
    {
        return false;
    }

    td_ = std::move(thread(bind(&camera::run, this)));

    return true;
}

void camera::set_camera_info(const camera_info &para)
{
    if (!use_mv_)
    {
        return;
    }

    for (auto &item : camera_infos_)
    {
        if (item.second.id == para.id)
        {
            item.second.value = para.value;

            switch (para.id)
            {
                case 1:
                    CameraSetAnalogGain(fd_, para.value);
                    break;

                case 2:
                    CameraSetExposureTime(fd_, para.value * 1000);
                    break;

                default:
                    break;
            }

            break;
        }
    }
}

void camera::run()
{
    is_alive_ = true;

    if (use_mv_)
    {
        while (is_alive_)
        {
            if (CameraGetImageBuffer(fd_, &sFrameInfo_, &buffer_, 1000) == CAMERA_STATUS_SUCCESS)
            {
                notify(SENSOR_CAMERA);
                usleep(10000);
                CameraReleaseImageBuffer(fd_, buffer_);
            }
        }
    }
    else
    {
        buf_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf_.memory = V4L2_MEMORY_MMAP;

        while (is_alive_)
        {

            if (ioctl(fd_, VIDIOC_DQBUF, &buf_) == -1)
            {
                LOG << "VIDIOC_DQBUF failed.\n";
                break;
            }

            num_bufs_ = buf_.index;
            buffers_[num_bufs_].bytesused = buf_.bytesused;
            buffers_[num_bufs_].length = buf_.length;
            buffers_[num_bufs_].offset = buf_.m.offset;
            notify(SENSOR_CAMERA);

            if (ioctl(fd_, VIDIOC_QBUF, &buf_) == -1)
            {
                LOG << "VIDIOC_QBUF error\n";
                break;
            }

            num_bufs_ = buf_.index;

            usleep(10000);
        }
    }
}

void camera::stop()
{
    is_alive_ = false;
    sleep(1);
    this->close();
    is_open_ = false;
}

void camera::close()
{
    if (use_mv_)
    {
        if (is_open_)
        {
            CameraUnInit(fd_);
        }
    }
    else
    {
        if (is_open_)
        {
            enum v4l2_buf_type type;
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (ioctl(fd_, VIDIOC_STREAMOFF, &type))
            {
                LOG << "VIDIOC_STREAMOFF error\n";
                return;
            }

            for (num_bufs_ = 0; num_bufs_ < 4; num_bufs_++)
            {
                munmap((void *)(buffers_[num_bufs_].start), buffers_[num_bufs_].length);
                buffers_[num_bufs_].start = nullptr;
            }

            free(buffers_);
            buffers_ = nullptr;
            ::close(fd_);
        }
    }
}

bool camera::open()
{
    int                     iCameraCounts = 1;
    int                     iStatus = -1;
    tSdkCameraDevInfo       tCameraEnumList;
    use_mv_ = true;
    CameraSdkInit(1);
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);

    if (iCameraCounts == 0)
    {
        use_mv_ = false;
    }

    if (use_mv_)
    {
        iStatus = CameraInit(&tCameraEnumList, -1, PARAMETER_TEAM_DEFAULT, &fd_);

        if (iStatus != CAMERA_STATUS_SUCCESS)
        {
            return false;
        }

        CameraGetCapability(fd_, &tCapability_);
        CameraSetAeState(fd_, false);
        CameraSetAnalogGain(fd_, camera_infos_["exposure_gain"].value);
        CameraSetExposureTime(fd_, camera_infos_["exposure_time"].value * 1000);
        CameraSetImageResolution(fd_, &(tCapability_.pImageSizeDesc[0]));
        w_ = tCapability_.pImageSizeDesc[0].iWidth;
        h_ = tCapability_.pImageSizeDesc[0].iHeight;
        CameraPlay(fd_);
    }
    else
    {
        fd_ = ::open(CONF->get_config_value<string>("image.dev_name").c_str(), O_RDWR, 0);

        if (fd_ < 0)
        {
            LOG << "open camera: " + CONF->get_config_value<string>("image.dev_name") + " failed\n";
            return false;
        }

        w_ = 640;
        h_ = 480;
        v4l2_format fmt;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.width = w_;
        fmt.fmt.pix.height = h_;
        fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0)
        {
            LOG << "set format failed\n";
            return false;
        }

        if (ioctl(fd_, VIDIOC_G_FMT, &fmt) < 0)
        {
            LOG << "get format failed\n";
            return false;
        }

        /*
        LOG << "--------------------------------------------------------\n";
        LOG << "Image Info: [";
        LOG << "format: " << (char)(fmt.fmt.pix.pixelformat & 0xFF) << (char)((fmt.fmt.pix.pixelformat >> 8)
                & 0xFF) << (char)((fmt.fmt.pix.pixelformat >> 16) & 0xFF) << (char)((fmt.fmt.pix.pixelformat >> 24) & 0xFF);
        LOG << " width: " << fmt.fmt.pix.width;
        LOG << " height: " << fmt.fmt.pix.height << "]\n";
        LOG << "--------------------------------------------------------\n";
        */
        v4l2_requestbuffers req;
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd_, VIDIOC_REQBUFS, &req) == -1)
        {
            LOG << "request buffer error \n";
            return false;
        }

        buffers_ = (VideoBuffer *)calloc(req.count, sizeof(VideoBuffer));

        for (num_bufs_ = 0; num_bufs_ < req.count; num_bufs_++)
        {
            buf_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf_.memory = V4L2_MEMORY_MMAP;
            buf_.index = num_bufs_;

            if (ioctl(fd_, VIDIOC_QUERYBUF, &buf_) == -1)
            {
                LOG << "query buffer error\n";
                return false;
            }

            buffers_[num_bufs_].length = buf_.length;
            buffers_[num_bufs_].offset = (size_t) buf_.m.offset;
            buffers_[num_bufs_].start = (unsigned char *)mmap(NULL, buf_.length, PROT_READ | PROT_WRITE,
                                        MAP_SHARED, fd_, buf_.m.offset);

            if (buffers_[num_bufs_].start == MAP_FAILED)
            {
                int err = errno;
                LOG << "buffer map error: " << err << "\n";
                return false;
            }

            if (ioctl(fd_, VIDIOC_QBUF, &buf_) == -1)
            {
                LOG << "VIDIOC_QBUF error\n";
                return false;
            }
        }

        enum v4l2_buf_type type;
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (ioctl(fd_, VIDIOC_STREAMON, &type) == -1)
        {
            LOG << "VIDIOC_STREAMON error\n";
            return false;
        }
    }

    is_open_ = true;
    return true;
}


camera::~camera()
{
    if (td_.joinable())
    {
        td_.join();
    }
}
