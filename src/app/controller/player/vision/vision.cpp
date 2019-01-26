#include "vision.hpp"
#include "parser/camera_parser.hpp"
#include "darknet/parser.h"
#include <cuda_runtime.h>
#include "cuda/cudaproc.h"
#include "server/server.hpp"
#include <algorithm>
#include "compare.hpp"
#include "core/worldmodel.hpp"

using namespace std;
using namespace cv;
using namespace robot_math;
using namespace Eigen;

Vision::Vision(): timer(CONF->get_config_value<int>("vision_period"))
{
    p_count_ = 0;
    is_busy_ = false;
    w_ = CONF->get_config_value<int>("image.width");
    h_ = CONF->get_config_value<int>("image.height");
    img_sd_type_ = IMAGE_SEND_RESULT;
    camera_src_ = nullptr;
    parser::camera_info_parser::parse(CONF->get_config_value<string>(CONF->player() + ".camera_info_file"), camera_infos_);
    parser::camera_param_parser::parse(CONF->get_config_value<string>(CONF->player() + ".camera_params_file"), params_);
    LOG << setw(12) << "algorithm:" << setw(18) << "[vision]" << " started!" << ENDL;
    ball_id_ = 0;
    post_id_ = 1;
    dets_prob_[ball_id_] = CONF->get_config_value<float>("detection_prob.ball");
    dets_prob_[post_id_] = CONF->get_config_value<float>("detection_prob.post");
}

Vision::~Vision()
{
    LOG << setw(12) << "algorithm:" << setw(18) << "[vision]" << " ended!" << ENDL;
}

void Vision::set_camera_info(const camera_info &para)
{
    for (auto &item : camera_infos_)
    {
        if (item.second.id == para.id)
        {
            item.second.value = para.value;
            break;
        }
    }
}

Eigen::Vector2f Vision::odometry(const Eigen::Vector2i &pos, const robot_math::transform_matrix &mat)
{
    float Xw, Yw;
    float OC = mat.p().z();
    Vector3d rpy = mat.R().eulerAngles(0, 1, 2);
    float roll = static_cast<float>(rpy.x());
    float theta = static_cast<float>(rpy.y());
    Vector2f centerPos(pos.x() - params_.cx, params_.cy - pos.y());
    Vector2i calCenterPos(static_cast<int>(centerPos.x()/cos(roll)), 
            static_cast<int>(centerPos.y()+centerPos.x()*tan(roll)));
    Vector2i calPos(calCenterPos.x() + params_.cx, params_.cy - calCenterPos.y());
    double gama = atan((params_.cy-(float)calPos.y())/params_.fy);
    double O_C_P = M_PI_2-theta+gama;
    Yw = OC*tan(O_C_P);
    Xw = ((float)calPos.x()-params_.cx)*OC*cos(gama)/(cos(O_C_P)*params_.fx);
    return Vector2f(Yw, Xw);
}

void Vision::src2dst()
{
    cudaError_t err;
    err = cudaMemcpy(dev_src_, camera_src_, src_size_, cudaMemcpyHostToDevice);
    check_error(err);

    if (use_mv_)
    {
        cudaBayer2BGR(dev_src_, dev_bgr_,  camera_w_,  camera_h_, camera_infos_["saturation"].value,
                      camera_infos_["red_gain"].value, camera_infos_["green_gain"].value, camera_infos_["blue_gain"].value);
        cudaUndistored(dev_bgr_, dev_undistored_, camera_w_, camera_h_, params_.fx, params_.fy, params_.cx, params_.cy,
                    params_.k1, params_.k2, params_.p1, params_.p2);
        cudaResizePacked(dev_undistored_, camera_w_,  camera_h_,  dev_ori_, w_,  h_);
    }
    else
    {
        cudaYUYV2BGR(dev_src_,  dev_bgr_,  camera_w_,  camera_h_);
        cudaResizePacked(dev_bgr_, camera_w_,  camera_h_,  dev_ori_, w_,  h_);
    }
}

void Vision::run()
{
    if (is_alive_)
    {

        if (camera_src_ == nullptr)
        {
            return;
        }

        p_count_ ++;

        if (is_busy_)
        {
            return;
        }

        frame_mutex_.lock();
        double t1 = clock();
        src2dst();
        //cout<<(clock()-t1)/CLOCKS_PER_SEC<<endl;
        frame_mutex_.unlock();
        cudaError_t err;
        cudaResizePacked(dev_ori_, w_, h_, dev_sized_, net_.w, net_.h);
        cudaBGR2RGBfp(dev_sized_, dev_rgbfp_, net_.w, net_.h);

        is_busy_ = true;
        layer l = net_.layers[net_.n - 1];
        //double t1 = clock();
        network_predict(net_, dev_rgbfp_, 0);
        int nboxes = 0;
        float nms = .45;
        detection *dets = get_network_boxes(&net_, w_, h_, 0.5, 0.5, 0, 1, &nboxes, 0);

        if (nms)
        {
            do_nms_sort(dets, nboxes, l.classes, nms);
        }
        ball_dets_.clear();
        post_dets_.clear();
        for (int i = 0; i < nboxes; i++)
        {
            if(dets[i].prob[ball_id_] > dets[i].prob[post_id_])
            {
                detection d = dets[i];
                d.prob[0] = dets[i].prob[ball_id_];
                if(d.prob[0] > dets_prob_[ball_id_])
                    ball_dets_.push_back(d);
            }
            else
            {
                detection d = dets[i];
                d.prob[0] = dets[i].prob[post_id_];
                if(d.prob[0] > dets_prob_[post_id_])
                    post_dets_.push_back(d);
            }
        }
        sort(ball_dets_.begin(), ball_dets_.end(), CompareDetGreater);
        sort(post_dets_.begin(), post_dets_.end(), CompareDetGreater);
        LOG <<"use time: "<<(clock()-t1)/CLOCKS_PER_SEC<<"s"<<ENDL;

        if (OPTS->use_debug())
        {
            Mat bgr(h_, w_, CV_8UC3);
            err = cudaMemcpy(bgr.data, dev_ori_, ori_size_, cudaMemcpyDeviceToHost);
            check_error(err);

            if (img_sd_type_ == IMAGE_SEND_ORIGIN)
            {
                send_image(bgr);
            }
            else if (img_sd_type_ == IMAGE_SEND_RESULT)
            {
                if(!ball_dets_.empty())
                {
                    /*
                    float Xw, Yw, Zw;
                    float OC = 0.44;
                    float roll = 0.0;
                    float theta = M_PI/4.0;
                    Vector2i pos(ball_dets_[0].bbox.x*w_*2, (ball_dets_[0].bbox.y + ball_dets_[0].bbox.h / 2.0)*h_*2);
                    Vector2f centerPos(pos.x() - params_.cx, params_.cy - pos.y());
                    Vector2i calCenterPos(static_cast<int>(centerPos.x()/cos(roll)), 
                            static_cast<int>(centerPos.y()+centerPos.x()*tan(roll)));
                    Vector2i calPos(calCenterPos.x() + params_.cx, params_.cy - calCenterPos.y());
                    double gama = atan((params_.cy-(float)calPos.y())/params_.fy);
                    double O_C_P = M_PI_2-theta+gama;
                    Yw = OC*tan(O_C_P);
                    Xw = ((float)calPos.x()-params_.cx)*OC*cos(gama)/(cos(O_C_P)*params_.fx);
                    Zw = -OC;
                    LOG<<sqrt(Yw*Yw+Xw*Xw)<<ENDL;
                    */
                    rectangle(bgr, Point((ball_dets_[0].bbox.x - ball_dets_[0].bbox.w / 2.0)*w_, (ball_dets_[0].bbox.y - ball_dets_[0].bbox.h / 2.0)*h_),
                              Point((ball_dets_[0].bbox.x + ball_dets_[0].bbox.w / 2.0)*w_, (ball_dets_[0].bbox.y + ball_dets_[0].bbox.h / 2.0)*h_),
                              Scalar(255, 0, 0, 0), 2);
                    putText(bgr, to_string(ball_dets_[0].prob[0]).substr(0,4), Point(ball_dets_[0].bbox.x * w_-40, ball_dets_[0].bbox.y * h_-40),
                                    FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2, 8);
                }
                if(!post_dets_.empty())
                {
                    rectangle(bgr, Point((post_dets_[0].bbox.x - post_dets_[0].bbox.w / 2.0)*w_, (post_dets_[0].bbox.y - post_dets_[0].bbox.h / 2.0)*h_),
                              Point((post_dets_[0].bbox.x + post_dets_[0].bbox.w / 2.0)*w_, (post_dets_[0].bbox.y + post_dets_[0].bbox.h / 2.0)*h_),
                              Scalar(0, 0, 255, 0), 2);
                    putText(bgr, to_string(post_dets_[0].prob[0]).substr(0,4), Point(post_dets_[0].bbox.x * w_-40, post_dets_[0].bbox.y * h_-40),
                                    FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, 8);
                }
                send_image(bgr);
            }
        }

        free_detections(dets, nboxes);
        is_busy_ = false;

    }
}

void Vision::send_image(const cv::Mat &src)
{
    cv::Mat bgr;
    src.copyTo(bgr);
    std::vector<unsigned char> jpgbuf;
    cv::imencode(".jpg", bgr, jpgbuf);
    bgr.release();
    tcp_command cmd;
    cmd.type = IMG_DATA;
    cmd.size = jpgbuf.size();
    cmd.data.assign((char *) & (jpgbuf[0]), jpgbuf.size());
    SERVER->write(cmd);
}

void Vision::updata(const pub_ptr &pub, const int &type)
{

    if (!is_alive_)
    {
        return;
    }
    if (type == sensor::SENSOR_CAMERA)
    {
        shared_ptr<camera> sptr = dynamic_pointer_cast<camera>(pub);

        if (camera_src_ ==  nullptr)
        {
            camera_w_ = sptr->camera_w();
            camera_h_ = sptr->camera_h();
            camera_size_ = sptr->camera_size();
            use_mv_ = sptr->use_mv();
            src_size_ = camera_size_;
            bgr_size_ = camera_w_ * camera_h_ * 3;
            camera_src_ = (unsigned char *)malloc(camera_size_);
            cudaError_t err;
            err = cudaMalloc((void **) &dev_src_,  src_size_);
            check_error(err);
            err = cudaMalloc((void **) &dev_bgr_, bgr_size_);
            check_error(err);
            err = cudaMalloc((void **) &dev_undistored_, bgr_size_);
            check_error(err);
        }
        frame_mutex_.lock();
        memcpy(camera_src_, sptr->buffer(), src_size_);
        camera_matrix_ = WM->head_matrix();
        frame_mutex_.unlock();
    }

}

bool Vision::start()
{
    names_.clear();
    ifstream ifs(CONF->get_config_value<string>("net_names_file"));

    while (!ifs.eof())
    {
        string s;
        ifs >> s;
        names_.push_back(s);
    }

    ifs.close();
    net_.gpu_index = 0;
    net_ = parse_network_cfg_custom((char *)CONF->get_config_value<string>("net_cfg_file").c_str(), 1);
    load_weights(&net_, (char *)CONF->get_config_value<string>("net_weights_file").c_str());
    set_batch_network(&net_, 1);
    fuse_conv_batchnorm(net_);
    calculate_binary_weights(net_);
    srand(2222222);

    ori_size_ = w_ * h_ * 3;
    sized_size_ = net_.w * net_.h * 3;
    rgbf_size_ = w_ * h_ * 3 * sizeof(float);

    cudaError_t err;
    err = cudaMalloc((void **)&dev_ori_, ori_size_);
    check_error(err);
    err = cudaMalloc((void **)&dev_sized_, sized_size_);
    check_error(err);
    err = cudaMalloc((void **)&dev_rgbfp_, rgbf_size_);
    check_error(err);
    is_alive_ = true;
    start_timer();
    return true;
}

void Vision::stop()
{
    if (is_alive_)
    {
        delete_timer();
        free_network(net_);
        free(camera_src_);
        cudaFree(dev_ori_);
        cudaFree(dev_src_);
        cudaFree(dev_bgr_);
        cudaFree(dev_undistored_);
        cudaFree(dev_rgbfp_);
        cudaFree(dev_sized_);
    }

    is_alive_ = false;
}
