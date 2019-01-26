#include <cuda_runtime.h>

__device__ unsigned char rgb_bound(int v)
{
    return v>255?255:(v<0?0:v);
}

template<typename T>
__device__ T max(T v1, T v2)
{
    return v1>v2?v1:v2;
}

template<typename T>
__device__ T min(T v1, T v2)
{
    return v1<v2?v1:v2;
}

__global__ void yuyv2yuv_kernal(unsigned char *in, unsigned char *out, int w, int h)
{
    int x=blockIdx.x;
    int y=threadIdx.x;
    int tmp = y*w;
    int src_offset = x*2;
    int dst_offset = x*3;

    out[tmp*3+dst_offset+0] = in[tmp*2+src_offset+0];
    out[tmp*3+dst_offset+1] = in[tmp*2+src_offset+(int)powf(-1, x&1)];
    out[tmp*3+dst_offset+2] = in[tmp*2+src_offset+2+(int)powf(-1, x&1)];
}

__global__ void yuyv2bgr_kernal(unsigned char *in, unsigned char *out, int w, int h)
{
    int x=blockIdx.x;
    int y=threadIdx.x;
    int tmp = y*w;
    int src_offset = x*2;
    int dst_offset = x*3;

    unsigned char Y = in[tmp*2+src_offset+0];
    unsigned char U = in[tmp*2+src_offset+(int)powf(-1, x&1)];
    unsigned char V = in[tmp*2+src_offset+2+(int)powf(-1, x&1)];
    float r,g,b;

    r = (1.164 * (Y - 16)) + (2.018 * (V - 128));
    g = (1.164 * (Y - 16)) - (0.813 * (U - 128)) - (0.391 * (V - 128));
    b = (1.164 * (Y - 16)) + (1.596 * (U - 128));

    out[tmp*3+dst_offset+2] = rgb_bound(r);
    out[tmp*3+dst_offset+1] = rgb_bound(g);
    out[tmp*3+dst_offset+0] = rgb_bound(b);
}

__global__ void bgr2rgbfp(unsigned char *in, float *rgbfp, int w, int h)
{
    int x=blockIdx.x;
    int y=threadIdx.x;
    int offset = y*w*3+x*3;
    float rf, gf, bf;
    rf = in[offset+2]/255.0f;
    gf = in[offset+1]/255.0f;
    bf = in[offset+0]/255.0f;
    int planesize = w*h;
    int tmp = y*w+x;
    rgbfp[tmp] = rf;
    rgbfp[planesize+tmp] = gf;
    rgbfp[planesize*2+tmp] = bf;
}

__global__ void baygr2bgr_kernal(unsigned char *bayergr, unsigned char *bgr, int w, int h,
    float ds, float rgain, float ggain, float bgain)
{
    int x = blockIdx.x;
    int y = threadIdx.x;
    int outy = h-y;
    int outx = w-x;
    float r,g,b;
    float hue, sat, val;
    float rn, gn, bn;

    b = bayergr[(y+((y+1)&1))*w+x-(x&1)]*bgain;
    g = bayergr[y*w+x-(x&1)+(y&1)]*ggain;
    r = bayergr[(y-(y&1))*w+x+((x+1)&1)]*rgain;

    float rgbMax = max(max(r,g), b);
    float rgbMin = min(min(r,g), b);
    float delta = rgbMax-rgbMin;

    val = rgbMax;
    if(rgbMax == 0) sat = 0;
    else sat = delta/rgbMax;

    if(delta == 0) hue = 0;
    else
    {
        if(rgbMax == r)
        {
            if(g>=b) hue = 60*(g-b)/delta;
            else hue = 60*(g-b)/delta+360;
        }
        else if(rgbMax == g)
        {
            hue = 60*(b-r)/delta+120;
        }
        else
        {
            hue = 60*(r-g)/delta+240;
        }
    }
    if(ds>=0) sat = sat+(1-sat)*ds;
    else sat = sat+sat*ds;

    int hi = hue/60.0;
    float f = hue/60.0-hi;
    float p = val*(1-sat);
    float q = val*(1-f*sat);
    float t = val*(1-(1-f)*sat);
    switch(hi)
    {
        case 0:
            rn = val; gn = t; bn = p; break;
        case 1:
            rn = q; gn = val; bn = p; break;
        case 2:
            rn = p; gn = val; bn = t; break;
        case 3:
            rn = p; gn = q; bn = val; break;
        case 4:
            rn = t; gn = p; bn = val; break;
        case 5:
            rn = val; gn = p; bn = q; break;
        default:
            break;
    }
    bgr[outy*w*3+outx*3+0] = rgb_bound(bn);
    bgr[outy*w*3+outx*3+1] = rgb_bound(gn);
    bgr[outy*w*3+outx*3+2] = rgb_bound(rn);
}

template<typename T>
__global__ void resize_packed_kernal(T *in, int iw, int ih, T *out, int ow, int oh)
{
    int x = blockIdx.x;
    int y = threadIdx.x;
    int offset_out = y*ow*3+x*3;
    float h_scale_rate = (float)ih/oh;
    float w_scale_rate = (float)iw/ow;
    float y_scale = h_scale_rate * y;
    float x_scale = w_scale_rate * x;
    int j = y_scale, i = x_scale;
    float u = y_scale-j, v = x_scale-i;
    int offset_in1 = j*iw*3;
    int offset_in2 = (j+1)*iw*3;
    if(j+1>=ih || i+1>=iw)
    {
        out[offset_out+0] = in[offset_in1+i*3];
        out[offset_out+1] = in[offset_in1+i*3+1];
        out[offset_out+2] = in[offset_in1+i*3+2];
    }
    else
    {
        unsigned char x1,x2,x3,x4;
        x1 = in[offset_in1+i*3];
        x2 = in[offset_in1+(i+1)*3];
        x3 = in[offset_in2+i*3];
        x4 = in[offset_in2+(i+1)*3];
        out[offset_out+0] = ((1-u)*(1-v)*x1+(1-u)*v*x2+u*(1-v)*x3+u*v*x4);
        x1 = in[offset_in1+i*3+1];
        x2 = in[offset_in1+(i+1)*3+1];
        x3 = in[offset_in2+i*3+1];
        x4 = in[offset_in2+(i+1)*3+1];
        out[offset_out+1] = ((1-u)*(1-v)*x1+(1-u)*v*x2+u*(1-v)*x3+u*v*x4);
        x1 = in[offset_in1+i*3+2];
        x2 = in[offset_in1+(i+1)*3+2];
        x3 = in[offset_in2+i*3+2];
        x4 = in[offset_in2+(i+1)*3+2];
        out[offset_out+2] = ((1-u)*(1-v)*x1+(1-u)*v*x2+u*(1-v)*x3+u*v*x4);
    }
}

__global__ void undistored_kernal(unsigned char *in, unsigned char *out, int w, int h, float fx, float fy, float cx, float cy,
    float k1, float k2, float p1, float p2)
{
    int x = blockIdx.x;
    int y = threadIdx.x;
    float u_distorted = 0, v_distorted = 0;
    float x1,y1,x2,y2;
    x1 = (x-cx)/fx;
    y1 = (y-cy)/fy;
    float r2;
    r2 = powf(x1,2)+powf(y1,2);
    x2  = x1*(1+k1*r2+k2*powf(r2,2))+2*p1*x1*y1+p2*(r2+2*x1*x1);
    y2 = y1*(1+k1*r2+k2*powf(r2,2))+p1*(r2+2*y1*y1)+2*p2*x1*y1;
    u_distorted = fx*x2+cx;
    v_distorted = fy*y2+cy;
    int inx = u_distorted, iny = v_distorted;
    int odx = y*w*3+x*3;
    int idx = iny*w*3+inx*3;
    if(inx<0||inx>=w||iny<0||iny>=h)
    {
        out[odx+0] = 0;
        out[odx+1] = 0;
        out[odx+2] = 0;
    }
    else
    {
        out[odx+0] = in[idx+0];
        out[odx+1] = in[idx+1];
        out[odx+2] = in[idx+2];
    }
}

void cudaYUYV2YUV(unsigned char *in, unsigned char *out, int w, int h)
{
    yuyv2yuv_kernal<<<w, h>>>(in,out,w,h);
}

void cudaYUYV2BGR(unsigned char *in, unsigned char *out, int w, int h)
{
    yuyv2bgr_kernal<<<w, h>>>(in,out,w,h);
}

void cudaBayer2BGR(unsigned char *bayer, unsigned char *bgr, int w, int h, 
    float sat, float rgain, float ggain, float bgain)
{
    baygr2bgr_kernal<<<w,h>>>(bayer, bgr, w, h, sat, rgain, ggain, bgain);
}

void cudaBGR2RGBfp(unsigned char *bgr, float *rgbfp, int w, int h)
{
    bgr2rgbfp<<<w,h>>>(bgr, rgbfp, w, h);
}

void cudaResizePacked(float *in, int iw, int ih, float *sized, int ow, int oh)
{
    resize_packed_kernal<<<ow, oh>>>(in, iw, ih, sized, ow, oh);
}

void cudaResizePacked(unsigned char *in, int iw, int ih, unsigned char *sized, int ow, int oh)
{
    resize_packed_kernal<<<ow, oh>>>(in, iw, ih, sized, ow, oh);
}

void cudaUndistored(unsigned char *in, unsigned char *out, int w, int h, float fx, float fy, float cx, float cy,
    float k1, float k2, float p1, float p2)
{
    undistored_kernal<<<w,h>>>(in, out, w, h, fx, fy, cx, cy, k1, k2, p1, p2);
}

