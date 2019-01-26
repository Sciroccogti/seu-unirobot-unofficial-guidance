#ifndef CLIST_H
#define CLIST_H

typedef struct node
{
    void *val;
    struct node *next;
    struct node *prev;
} node;

typedef struct clist
{
    int size;
    node *front;
    node *back;
} clist;

clist *make_list();
int list_find(clist *l, void *val);

void list_insert(clist *, void *);

void **list_to_array(clist *l);

void free_list(clist *l);
void free_list_contents(clist *l);
void free_list_contents_kvp(clist *l);

#endif
