#ifndef OPTION_LIST_H
#define OPTION_LIST_H
#include "clist.h"

#ifdef YOLODLL_EXPORTS
#if defined(_MSC_VER)
#define YOLODLL_API __declspec(dllexport)
#else
#define YOLODLL_API __attribute__((visibility("default")))
#endif
#else
#if defined(_MSC_VER)
#define YOLODLL_API
#else
#define YOLODLL_API
#endif
#endif

typedef struct
{
    char *key;
    char *val;
    int used;
} kvp;


int read_option(char *s, clist *options);
void option_insert(clist *l, char *key, char *val);
char *option_find(clist *l, char *key);
char *option_find_str(clist *l, char *key, char *def);
int option_find_int(clist *l, char *key, int def);
int option_find_int_quiet(clist *l, char *key, int def);
float option_find_float(clist *l, char *key, float def);
float option_find_float_quiet(clist *l, char *key, float def);
void option_unused(clist *l);

typedef struct
{
    int classes;
    char **names;
} metadata;


#endif
