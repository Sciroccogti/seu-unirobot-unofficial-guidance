#pragma once

void cudaYUYV2YUV(unsigned char *in, unsigned char *out, int w, int h);
void cudaYUYV2BGR(unsigned char *in, unsigned char *out, int w, int h);
void cudaBGR2RGBfp(unsigned char *bgr, float *rgbfp, int w, int h);

void cudaBayer2BGR(unsigned char *bayer, unsigned char *bgr, int w, int h, float sat, float rgain, float ggain, float bgain);
void cudaResizePacked(float *in, int iw, int ih, float *sized, int ow, int oh);
void cudaResizePacked(unsigned char *in, int iw, int ih, unsigned char *sized, int ow, int oh);
void cudaUndistored(unsigned char *in, unsigned char *out, int w, int h, float fx, float fy, float cx, float cy,
    float k1, float k2, float p1, float p2);
