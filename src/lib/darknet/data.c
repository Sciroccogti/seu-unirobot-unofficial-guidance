#include "data.h"
#include "utils.h"
#include "image.h"
#include "cuda.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>



matrix concat_matrix(matrix m1, matrix m2)
{
    int i, count = 0;
    matrix m;
    m.cols = m1.cols;
    m.rows = m1.rows + m2.rows;
    m.vals = calloc(m1.rows + m2.rows, sizeof(float *));

    for (i = 0; i < m1.rows; ++i)
    {
        m.vals[count++] = m1.vals[i];
    }

    for (i = 0; i < m2.rows; ++i)
    {
        m.vals[count++] = m2.vals[i];
    }

    return m;
}

void get_random_batch(data d, int n, float *X, float *y)
{
    int j;

    for (j = 0; j < n; ++j)
    {
        int index = random_gen() % d.X.rows;
        memcpy(X + j * d.X.cols, d.X.vals[index], d.X.cols * sizeof(float));
        memcpy(y + j * d.y.cols, d.y.vals[index], d.y.cols * sizeof(float));
    }
}

void get_next_batch(data d, int n, int offset, float *X, float *y)
{
    int j;

    for (j = 0; j < n; ++j)
    {
        int index = offset + j;
        memcpy(X + j * d.X.cols, d.X.vals[index], d.X.cols * sizeof(float));
        memcpy(y + j * d.y.cols, d.y.vals[index], d.y.cols * sizeof(float));
    }
}

data get_data_part(data d, int part, int total)
{
    data p = {0};
    p.shallow = 1;
    p.X.rows = d.X.rows * (part + 1) / total - d.X.rows * part / total;
    p.y.rows = d.y.rows * (part + 1) / total - d.y.rows * part / total;
    p.X.cols = d.X.cols;
    p.y.cols = d.y.cols;
    p.X.vals = d.X.vals + d.X.rows * part / total;
    p.y.vals = d.y.vals + d.y.rows * part / total;
    return p;
}
