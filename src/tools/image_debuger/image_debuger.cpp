#include "image_debuger.hpp"
#include "configuration.hpp"
#include "ui/walk_remote.hpp"
#include "ui/camera_setter.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace robot;

image_debuger::image_debuger()
{
    height_ = CONF->get_config_value<int>("image.height");
    width_ = CONF->get_config_value<int>("image.width");

    srcLab = new ImageLabel(width_, height_);
    dstLab = new ImageLabel(width_, height_);
    curr_image_.create(height_, width_, CV_8UC3);
    curr_index_ = 0;
    infoLab = new QLabel("0/0");
    statusBar()->addWidget(infoLab);

    QHBoxLayout *imageLayout = new QHBoxLayout;
    imageLayout->addWidget(srcLab);
    imageLayout->addWidget(dstLab);

    funcBox = new QComboBox();
    QStringList funclist;
    funclist << "algorithm";
    funclist << "sampling";
    funcBox->addItems(funclist);
    btnLoad = new QPushButton("Load File");
    btnLast = new QPushButton("Last Frame");
    btnNext = new QPushButton("Next Frame");
    boxAuto = new QCheckBox("Auto Play(ms)");
    boxAuto->setFixedWidth(120);
    delayEdit = new QLineEdit("1000");
    delayEdit->setFixedWidth(50);
    QHBoxLayout *ctrlLayout = new QHBoxLayout;
    ctrlLayout->addWidget(funcBox);
    ctrlLayout->addWidget(btnLoad);
    ctrlLayout->addWidget(btnLast);
    ctrlLayout->addWidget(btnNext);
    ctrlLayout->addWidget(boxAuto);
    ctrlLayout->addWidget(delayEdit);

    frmSlider = new QSlider(Qt::Horizontal);
    frmSlider->setEnabled(false);

    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->addLayout(imageLayout);
    mainLayout->addLayout(ctrlLayout);
    mainLayout->addWidget(frmSlider);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    timer = new QTimer;
    connect(timer, &QTimer::timeout, this, &image_debuger::procTimer);
    connect(btnLoad, &QPushButton::clicked, this, &image_debuger::procBtnLoad);
    connect(btnLast, &QPushButton::clicked, this, &image_debuger::procBtnLast);
    connect(btnNext, &QPushButton::clicked, this, &image_debuger::procBtnNext);
    connect(boxAuto, &QCheckBox::stateChanged, this, &image_debuger::procBoxAuto);
    connect(frmSlider, &QSlider::valueChanged, this, &image_debuger::procFrmSlider);
    connect(srcLab, &ImageLabel::shot, this, &image_debuger::procShot);
}

void image_debuger::show_src()
{
    cvtColor(yuv_images_[curr_index_ - 1], rgb_src_, CV_YUV2RGB);
    QImage srcImage(rgb_src_.data, rgb_src_.cols, rgb_src_.rows, QImage::Format_RGB888);
    srcLab->set_image(srcImage);
}

void image_debuger::show_dst(Mat dst)
{
    QImage dstImage(dst.data, dst.cols, dst.rows,
                    dst.channels() == 3 ? QImage::Format_RGB888 : QImage::Format_Grayscale8);
    dstLab->set_image(dstImage);
}

void image_debuger::proc_image(const unsigned int &index)
{
    if (index < 1 || index > yuv_images_.size())
    {
        return;
    }

    curr_index_ = index;
    infoLab->setText(QString::number(curr_index_) + "/" + QString::number(yuv_images_.size()));
    frmSlider->setValue(index);
    show_src();

    if (funcBox->currentIndex() == 0) //algorithm
    {
        Mat dst;
        cvtColor(rgb_src_, dst, CV_RGB2GRAY);
        show_dst(dst);
    }
}

void image_debuger::procBtnLast()
{
    curr_index_--;

    if (curr_index_ < 1)
    {
        curr_index_ = yuv_images_.size();
    }

    proc_image(curr_index_);
}

void image_debuger::procBtnNext()
{
    curr_index_++;

    if (curr_index_ > yuv_images_.size())
    {
        curr_index_ = 1;
    }

    proc_image(curr_index_);
}

void image_debuger::procBtnLoad()
{
    timer->stop();
    QString filename = QFileDialog::getOpenFileName(this, "Open file", QDir::homePath(), tr("*.yuv"));

    if (filename.isEmpty())
    {
        return;
    }

    yuv_images_.clear();
    //yuv_images_ = read_yuv(filename.toStdString());

    if (!yuv_images_.empty())
    {
        frmSlider->setEnabled(true);
        frmSlider->setMinimum(1);
        frmSlider->setMaximum(yuv_images_.size());
        proc_image(1);
    }
}

void image_debuger::procBoxAuto()
{
    if (boxAuto->checkState() == Qt::Checked)
    {
        int delay = delayEdit->text().toInt();

        if (delay < 10)
        {
            delay = 10;
        }

        timer->start(delay);
    }
    else
    {
        timer->stop();
    }
}

void image_debuger::procShot(QRect rect)
{
    if (rect.width() > 10 && rect.height() > 10)
    {
        int x, y, w, h;
        x = rect.left();
        y = rect.top();
        w = rect.width();
        h = rect.height();

        if (x + w < width_ && y + h < height_)
        {
            if (funcBox->currentIndex() == 1) // sampling
            {
                Rect mrect(x, y, w, h);
                Mat roi = rgb_src_(mrect);
                imwrite("test.jpg", roi);
                Mat dst;
                rgb_src_.copyTo(dst);

                for (int j = 0; j < height_; j++)
                {
                    for (int i = 0; i < width_; i++)
                    {
                        if (j >= y && j < y + h && i >= x && i < x + w)
                        {
                            continue;
                        }

                        dst.data[j * width_ * 3 + i * 3] = 0;
                        dst.data[j * width_ * 3 + i * 3 + 1] = 0;
                        dst.data[j * width_ * 3 + i * 3 + 2] = 0;
                    }
                }

                show_dst(dst);
            }
        }
    }
}

void image_debuger::procFrmSlider(int v)
{
    proc_image(v);
}


void image_debuger::procTimer()
{
    procBtnNext();
}
