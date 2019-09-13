#   yolov3-tiny 笔记

[教程0：官方教程](https://pjreddie.com/darknet/yolo/)

其实整个过程完全就是跟着教程走，然而教程有写地方并没有写清楚，故导致我摸索了好久。

先送上参考教程：
*   [教程1：用YOLO训练自己的数据集--20170823](https://blog.csdn.net/jocelyn870/article/details/77868739)
*   [教程2：YOLOV3 将自己的txt转为XML，再将XML转为符合YOLO要求的txt格式](https://blog.csdn.net/qq_29762941/article/details/80797790)
*   [教程3：YOLOv3训练自己的数据详细步骤](https://blog.csdn.net/john_bh/article/details/80625220)

##  导出符合yolo要求的图片描述txt

此处使用[ImageTagger](https://imagetagger.bit-bots.de/images/)作为训练样本，该网站须注册后使用。

在网站中挑选合适的样本后，根据*教程1*将下载得到的压缩包解压至指定位置。

>   挑选样本时，请注意样本的标签，即**Annotation**，因为这是训练时所要用到的关键因素，务必按需挑选

```
#   在ImageTagger上导出.txt的参考方法
Base format：%%content

Image format：%%annotations

Annotation format：%%imagename  %%type  %%minx  %%miny  %%maxx  %%maxy

Not in image format：%%imagename  %%type  -1  -1  -1  -1
```

我已将`export2txt.py`上传，请按需修改。

运行以上脚本后，需要调用`.../darknet/scripts/voc_label.py`。

>   直接运行时，会出现`找不到xxx.jpg.xml`的报错，请自行在记载文件名的`.txt`文件内将所有后缀删除

##  编辑配置文件并运行
编辑`.cfg`文件时，请注意：
*   主要须修改两个`[yolo]`中的`classes`，及他们各自的前一个`[convolutional]`中的`filters`
*   `filters = 3 * (classes + coords + 1)`，其中，`classes`为要检测的物体种类数，`coords`为坐标数，一般为`4`

我自用的`.cfg`和`voc.data`已经上传，仅供参考

终端运行
`./darknet detector train cfg/voc.data cfg/yolov3-voc.cfg`

>   建议在root下启动，避免出现`无法打开backup`的情况

##  测试
```Bash
./darknet detector test cfg/voc.data cfg/my.cfg backup20190130/my_10000.weights data/images.jpg
```

