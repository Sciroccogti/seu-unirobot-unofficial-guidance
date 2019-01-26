#ifndef DATA_H
#define DATA_H

#if defined(_MSC_VER) && _MSC_VER < 1900
#define inline __inline
#endif

#include "matrix.h"
#include "clist.h"
#include "image.h"
#include "tree.h"

static inline float distance_from_edge(int x, int max)
{
    int dx = (max / 2) - x;

    if (dx < 0)
    {
        dx = -dx;
    }

    dx = (max / 2) + 1 - dx;
    dx *= 2;
    float dist = (float)dx / max;

    if (dist > 1)
    {
        dist = 1;
    }

    return dist;
}

typedef struct
{
    int w, h;
    matrix X;
    matrix y;
    int shallow;
    int *num_boxes;
    box **boxes;
} data;

typedef enum
{
    CLASSIFICATION_DATA, DETECTION_DATA, CAPTCHA_DATA, REGION_DATA, IMAGE_DATA, LETTERBOX_DATA, COMPARE_DATA, WRITING_DATA, SWAG_DATA, TAG_DATA, OLD_CLASSIFICATION_DATA, STUDY_DATA, DET_DATA, SUPER_DATA
} data_type;

typedef struct load_args
{
    int threads;
    char **paths;
    char *path;
    int n;
    int m;
    char **labels;
    int h;
    int w;
    int c; // color depth
    int out_w;
    int out_h;
    int nh;
    int nw;
    int num_boxes;
    int min, max, size;
    int classes;
    int background;
    int scale;
    int small_object;
    float jitter;
    int flip;
    float angle;
    float aspect;
    float saturation;
    float exposure;
    float hue;
    data *d;
    image *im;
    image *resized;
    data_type type;
    tree *hierarchy;
} load_args;

typedef struct
{
    int id;
    float x, y, w, h;
    float left, right, top, bottom;
} box_label;

void free_data(data d);
void get_random_batch(data d, int n, float *X, float *y);
data get_data_part(data d, int part, int total);
void get_next_batch(data d, int n, int offset, float *X, float *y);

#endif
