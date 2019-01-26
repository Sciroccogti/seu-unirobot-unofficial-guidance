#!/usr/bin/env python3
#coding: utf-8

class FallDirection:
    FALL_NONE = 0
    FALL_FORWARD = 1
    FALL_BACKWARD = -1
    FALL_LEFT = 2
    FALL_RIGHT = -2


class WorldModelData:
    WORLDMODEL_DATA = 'fffffi'
    def __init__(self):
        self.ballx = 0.0
        self.bally = 0.0
        self.bodyx = -1.0
        self.bodyy = 0.0
        self.bodydir = 0.0
        self.fall = FallDirection.FALL_NONE
    
    def set_data(self,args):
        self.ballx = args[0]
        self.bally = args[1]
        self.bodyx = args[2]
        self.bodyy = args[3]
        self.bodydir = args[4]
        self.fall = args[5]

    def get_data(self):
        return (self.ballx, self.bally, self.bodyx, self.bodyy, self.bodydir, self.fall)