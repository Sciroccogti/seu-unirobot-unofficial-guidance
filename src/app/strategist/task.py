#!/usr/bin/env python3
#coding: utf-8

import struct
from tcp import tcp_cmd_type, tcp_size

class task_type:
    TASK_NONE = 0
    TASK_WALK = 1
    TASK_ACT = 2
    TASK_LOOK = 3
    TASK_LED = 4

     
class walk_task:
    def __init__(self, x, y, d, enable):
        self.__type = task_type.TASK_WALK
        self.__x = x
        self.__y = y
        self.__d = d
        self.__e = enable

    def data(self):
        fmt = '=ifff?'
        return struct.pack(tcp_cmd_type.TCP_FMT, tcp_cmd_type.TASK_DATA, True, struct.calcsize(fmt))\
            + struct.pack(fmt, self.__type, self.__x, self.__y, self.__d, self.__e)

        
class action_task:
    def __init__(self, name):
        self.__type = task_type.TASK_ACT
        self.__name = name

    def data(self):
        fmt = '=i%ds'%len(self.__name)
        return struct.pack(tcp_cmd_type.TCP_FMT, tcp_cmd_type.TASK_DATA, True, struct.calcsize(fmt))\
            + struct.pack(fmt, self.__type, self.__name.encode('utf-8'))


class look_task:
    def __init__(self, yaw, pitch, enable):
        self.__type = task_type.TASK_LOOK
        self.__yaw = yaw
        self.__pitch = pitch
        self.__e = enable
    
    def data(self):
        fmt = '=iff?'
        return struct.pack(tcp_cmd_type.TCP_FMT, tcp_cmd_type.TASK_DATA, True, struct.calcsize(fmt))\
            + struct.pack(fmt, self.__type, self.__yaw, self.__pitch, self.__e)


class led_task:
    def __init__(self, led1, led2):
        self.__type = task_type.TASK_LED
        self.__led1 = led1
        self.__led2 = led2
    
    def data(self):
        fmt = '=i??'
        return struct.pack(tcp_cmd_type.TCP_FMT, tcp_cmd_type.TASK_DATA, True, struct.calcsize(fmt))\
            + struct.pack(fmt, self.__type, self.__led1, self.__led2)