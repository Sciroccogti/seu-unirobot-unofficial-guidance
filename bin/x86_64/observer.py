#!/usr/bin/env python3
#coding: utf-8

from common import LOG


class publisher:
    PUB_WORLDMODEL = 1
    PUB_GAMECONTROLLER = 2
    PUB_TEAMMATE = 3
    def __init__(self, name):
        LOG('{:12s}{:18s} started!'.format('sensor:','['+name+']'))
        self.__subs = []
        self.__name = name

    def __del__(self):
        LOG('{:12s}{:18s} ended!'.format('sensor:','['+self.__name+']'))

    def attach(self, sub):
        self.__subs.append(sub)

    def dettach(self, sub):
        self.__subs.remove(sub)

    def notify(self, pub, data):
        for sub in self.__subs:
            sub.update(pub, data)


class subscriber:
    def __init__(self):
        pass

    def update(self, pub, data):
        pass