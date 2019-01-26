#!/usr/bin/env python3
#coding: utf-8

import observer
import client
import gamecontroller
import teammate
import threading
import worldmodel
from common import LOG

class Robot(observer.subscriber):
    def __init__(self):
        observer.subscriber.__init__(self)
        self.wm_mtx = threading.Lock()
        self.gc_mtx = threading.Lock()
        self.tm_mtx = threading.Lock()
        self.__wm_data = worldmodel.WorldModelData()
        self.__gc_data = gamecontroller.RoboCupGameControlData()
        self.__tm_data = {}

    def gc_data(self):
        self.gc_mtx.acquire()
        gc_data = self.__gc_data
        self.gc_mtx.release()
        return gc_data

    def wm_data(self):
        self.wm_mtx.acquire()
        wm_data = self.__wm_data
        self.wm_mtx.release()
        return wm_data

    def tm_data(self):
        self.tm_mtx.acquire()
        tm_data = self.__tm_data
        self.tm_mtx.release()
        return tm_data

    def update(self, pub, data):
        if pub == observer.publisher.PUB_WORLDMODEL:
            self.wm_mtx.acquire()
            self.__wm_data.set_data(data)
            self.wm_mtx.release()
        elif pub == observer.publisher.PUB_GAMECONTROLLER:
            if data[0].decode('utf-8') != gamecontroller.GameData.GAMECONTROLLER_STRUCT_HEADER:
                return
            self.gc_mtx.acquire()
            self.__gc_data.set_data(data)
            self.gc_mtx.release()
        elif pub == observer.publisher.PUB_TEAMMATE:
            if data[0].decode('utf-8') != teammate.TeammateData.TEAMMATE_HEADER:
                return
            self.tm_mtx.acquire()
            temp_tm = teammate.TeammateData(data)
            self.__tm_data[temp_tm.id] = temp_tm
            self.tm_mtx.release()

ROBOT = Robot()