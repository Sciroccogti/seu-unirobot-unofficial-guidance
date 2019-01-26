#!/usr/bin/env python3
#coding: utf-8


class tcp_cmd_type:
    REG_DATA = 2
    END_DATA = 9
    WM_DATA = 15
    TASK_DATA = 20
    TCP_FMT = '=i?I'

class tcp_data_dir:
    DIR_BOTH = 0
    DIR_APPLY = 1
    DIR_SUPPLY = 2

class tcp_size:
    enum_size = 4
    int_size = 4
    float_size = 4
    bool_size = 1
    head_size = 9
