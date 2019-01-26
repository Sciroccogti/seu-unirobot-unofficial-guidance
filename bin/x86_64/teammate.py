#!/usr/bin/env python3
#coding: utf-8

import socket
import observer
import threading
import struct
import robot
import time
from config import CONF
from robot import ROBOT
from common import LOG
import worldmodel

class TeammateData:
    TEAMMATE_HEADER = 'SEU'
    TEAMMATE_DATA = '%dsifffff'%len(TEAMMATE_HEADER)
    def __init__(self, args=tuple()):
        self.header = TeammateData.TEAMMATE_HEADER.encode('utf-8')
        if len(args) != 0:
            self.id = args[0]
            self.ballx = args[1]
            self.bally = args[2]
            self.bodyx = args[3]
            self.bally = args[4]
            self.bodydir = args[5]

    def set_data(self, mid, args):
        self.id = mid
        self.ballx = args.ballx
        self.bally = args.bally
        self.bodyx = args.bodyx
        self.bally = args.bodyy
        self.bodydir = args.bodydir

    def data(self):
        return struct.pack(TeammateData.TEAMMATE_DATA, self.header, self.id, self.ballx, self.bally,\
            self.bodyx, self.bally, self.bodydir)


class Teammate(observer.publisher):
    def __init__(self, port, period=500):
        observer.publisher.__init__(self, 'teammate')
        self.__port = port
        self.__td = None
        self.is_alive = False
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__sock.setblocking(True)
        self.__sock.settimeout(1.0)
        self.__period = period
        self.__tm_data = TeammateData()

    def __del__(self):
        self.__td.join()
        observer.publisher.__del__(self)

    def start(self):
        try:
            self.__sock.bind(('', self.__port))
            self.is_alive = True
        except:
            return False
        self.is_alive = True
        self.__td = threading.Thread(target=self.run)
        self.__td.start()
        return True

    def stop(self):
        self.is_alive = False
        self.__sock.close()
    
    def run(self):
        t_last = int(time.time()*1000)
        while self.is_alive:
            try:
                data, addr = self.__sock.recvfrom(256)
                if len(data) != 0:
                    self.notify(observer.publisher.PUB_TEAMMATE, struct.unpack(TeammateData.TEAMMATE_DATA, data))
            except:
                pass
            try:
                t = int(time.time()*1000)
                if t-t_last >= self.__period:
                    self.__tm_data.set_data(CONF.id, ROBOT.wm_data())
                    self.__sock.sendto(self.__tm_data.data(), ('', self.__port))
                    t_last = t
            except:
                pass

