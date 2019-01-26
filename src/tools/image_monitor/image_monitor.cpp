#include "image_monitor.hpp"
#include "configuration.hpp"
#include "ui/walk_remote.hpp"
#include "ui/camera_setter.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace robot;

image_monitor::image_monitor()
    : client_(CONF->get_config_value<string>(CONF->player() + ".address"), CONF->get_config_value<int>("net.tcp.port"),
              bind(&image_monitor::data_handler, this, placeholders::_1))
{
    height_ = CONF->get_config_value<int>("image.height");
    width_ = CONF->get_config_value<int>("image.width");
    first_connect = true;

    imageLab = new ImageLabel(width_, height_);
    curr_image_.create(height_, width_, CV_8UC3);

    pitchSlider = new QSlider(Qt::Vertical);
    pitchSlider->setRange(-90, 90);
    yawSlider = new QSlider(Qt::Horizontal);
    yawSlider->setRange(-90, 90);

    btnWR = new QPushButton("Walk Remote");
    btnCS = new QPushButton("Camera Setting");

    colorBox = new QComboBox();
    colorCheck = new QCheckBox("Color");

    QStringList clrs;
    clrs << "none";
    colorBox->addItems(clrs);
    QHBoxLayout *colorLayout = new QHBoxLayout();
    colorLayout->addWidget(colorCheck);
    colorLayout->addWidget(colorBox);

    imageBox = new QComboBox();
    map<image_send_type, string> image_send_types =
    {
        {IMAGE_SEND_ORIGIN, "origin"},
        {IMAGE_SEND_COLOR, "color"},
        {IMAGE_SEND_RESULT, "result"}
    };
    clrs.clear();

    for (auto ist : image_send_types)
    {
        clrs << QString::fromStdString(ist.second);
    }

    imageBox->addItems(clrs);
    imageBox->setCurrentIndex(IMAGE_SEND_RESULT);
    QHBoxLayout *imageLayout = new QHBoxLayout();
    imageLayout->addWidget(new QLabel("image"));
    imageLayout->addWidget(imageBox);

    QVBoxLayout *leftLayout = new QVBoxLayout();
    leftLayout->addWidget(imageLab);
    leftLayout->addWidget(yawSlider);

    QVBoxLayout *ctrlLayout = new QVBoxLayout;
    ctrlLayout->addWidget(btnWR);
    ctrlLayout->addWidget(btnCS);
    ctrlLayout->addLayout(imageLayout);
    ctrlLayout->addLayout(colorLayout);

    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->addLayout(leftLayout);
    mainLayout->addWidget(pitchSlider);
    mainLayout->addLayout(ctrlLayout);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    yawLab = new QLabel();
    pitchLab = new QLabel();
    netLab = new QLabel();
    netLab->setFixedWidth(100);

    statusBar()->addWidget(pitchLab);
    statusBar()->addWidget(yawLab);
    statusBar()->addWidget(netLab);

    net_info = QString::fromStdString(CONF->get_config_value<string>(CONF->player() + ".address"))
               + ":" + QString::number(CONF->get_config_value<int>("net.tcp.port"));
    setWindowTitle(net_info);

    timer = new QTimer;
    timer->start(1000);

    connect(timer, &QTimer::timeout, this, &image_monitor::procTimer);
    connect(yawSlider, &QSlider::valueChanged, this, &image_monitor::procYawSlider);
    connect(pitchSlider, &QSlider::valueChanged, this, &image_monitor::procPitchSlider);
    connect(btnWR, &QPushButton::clicked, this, &image_monitor::procBtnWR);
    connect(btnCS, &QPushButton::clicked, this, &image_monitor::procBtnCS);
    connect(imageLab, &ImageLabel::shot, this, &image_monitor::procShot);
    connect(imageBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &image_monitor::procImageBox);
    client_.start();
    yawSlider->setEnabled(false);
    pitchSlider->setEnabled(false);
}

void image_monitor::data_handler(const tcp_command cmd)
{
    if (cmd.type == IMG_DATA)
    {
        vector<unsigned char> buf(cmd.size);
        memcpy(&buf[0], cmd.data.c_str(), cmd.size);

        try
        {
            Mat bgr = imdecode(buf, cv::IMREAD_COLOR);
            Mat dst;
            cvtColor(bgr, dst, CV_BGR2RGB);
            QImage dstImage((const unsigned char *)(dst.data), dst.cols, dst.rows, QImage::Format_RGB888);
            imageLab->set_image(dstImage);
            bgr.release();
            dst.release();
        }
        catch (std::exception &e)
        {
            cout << e.what() << endl;
        }
    }
}

void image_monitor::procTimer()
{
    if (client_.is_connected())
    {
        if (first_connect)
        {
            client_.regist(IMG_DATA, DIR_APPLY);
            usleep(10000);
            client_.regist(REMOTE_DATA, DIR_SUPPLY);
        }

        first_connect = false;
        netLab->setStyleSheet("background-color:green");
        yawSlider->setEnabled(true);
        pitchSlider->setEnabled(true);
        colorCheck->setEnabled(true);
        imageBox->setEnabled(true);
    }
    else
    {
        first_connect = true;
        netLab->setStyleSheet("background-color:red");
        yawSlider->setEnabled(false);
        pitchSlider->setEnabled(false);
        colorCheck->setChecked(false);
        colorCheck->setEnabled(false);
        imageBox->setEnabled(false);
    }
}

void image_monitor::procBtnWR()
{
    walk_remote *wr = new walk_remote(client_, net_info, this);
    wr->show();
}

void image_monitor::procBtnCS()
{
    camera_setter *cs = new camera_setter(client_, net_info, this);
    cs->show();
}

void image_monitor::procShot(QRect rect)
{
    if (rect.width() > 5 && rect.height() > 5)
    {
        int x, y, w, h;
        x = rect.left();
        y = rect.top();
        w = rect.width();
        h = rect.height();

        if (colorCheck->isChecked())
        {
            int c = colorBox->currentIndex();
            remote_data_type t = COLOR_SAMPLE;
            tcp_command cmd;
            cmd.type = REMOTE_DATA;
            cmd.size = 5 * int_size + enum_size;
            cmd.data.clear();
            cmd.data.append((char *)(&t), enum_size);
            cmd.data.append((char *)(&c), int_size);
            cmd.data.append((char *)(&x), int_size);
            cmd.data.append((char *)(&y), int_size);
            cmd.data.append((char *)(&w), int_size);
            cmd.data.append((char *)(&h), int_size);
            client_.write(cmd);
        }
    }
}

void image_monitor::procImageBox(int idx)
{
    remote_data_type t = IMAGE_SEND_TYPE;
    tcp_command cmd;
    cmd.type = REMOTE_DATA;
    cmd.size = int_size + enum_size;
    cmd.data.clear();
    cmd.data.append((char *)(&t), enum_size);
    cmd.data.append((char *)(&idx), int_size);
    client_.write(cmd);
}

void image_monitor::procPitchSlider(int v)
{
    pitchLab->setText(QString::number(v));
    float yaw = (float)(yawSlider->value());
    float pitch = (float)v;
    remote_data_type t = LOOKAT_DATA;

    tcp_command cmd;
    cmd.type = REMOTE_DATA;
    cmd.size = 2 * float_size + enum_size;
    cmd.data.clear();
    cmd.data.append((char *)(&t), enum_size);
    cmd.data.append((char *)(&yaw), float_size);
    cmd.data.append((char *)(&pitch), float_size);
    client_.write(cmd);
}

void image_monitor::procYawSlider(int v)
{
    yawLab->setText(QString::number(v));
    float yaw = (float)v;
    float pitch = (float)(pitchSlider->value());
    remote_data_type t = LOOKAT_DATA;

    tcp_command cmd;
    cmd.type = REMOTE_DATA;
    cmd.size = 2 * float_size + enum_size;
    cmd.data.clear();
    cmd.data.append((char *)(&t), enum_size);
    cmd.data.append((char *)(&yaw), float_size);
    cmd.data.append((char *)(&pitch), float_size);
    client_.write(cmd);
}

void image_monitor::closeEvent(QCloseEvent *event)
{
    client_.stop();
}
