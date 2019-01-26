#!/usr/bin/env python3
#coding: utf-8

from transitions import Machine
from robot import ROBOT
from worldmodel import FallDirection
from task import walk_task, action_task, look_task


class fsm(object):
    def __init__(self):
        self.tasks = []

    def on_enter_Start(self):
        self.next_state = self.to_Ready

    def on_enter_Ready(self):
        if ROBOT.wm_data().fall != FallDirection.FALL_NONE:
            self.next_state = self.to_Getup
        else:
            self.tasks.append(action_task('reset'))
            #self.tasks.append(walk_task(0.0, 0.0, 0.0, True))
            #self.next_state = self.to_SearchBall

    def on_enter_Getup(self):
        self.next_state = self.to_Ready
        if ROBOT.wm_data().fall == FallDirection.FALL_FORWARD:
            self.tasks.append(action_task('forward_getup'))
        elif ROBOT.wm_data().fall == FallDirection.FALL_BACKWARD:
            self.tasks.append(action_task('backward_getup'))
    
    def on_enter_SearchBall(self):
        if ROBOT.wm_data().fall != FallDirection.FALL_NONE:
            self.next_state = self.to_Getup
    
    def on_enter_GotoBall(self):
        if ROBOT.wm_data().fall != FallDirection.FALL_NONE:
            self.next_state = self.to_Getup

    def on_enter_KickBall(self):
        self.next_state = self.to_SearchBall
        if ROBOT.wm_data().fall != FallDirection.FALL_NONE:
            self.next_state = self.to_Getup
        else:
            self.tasks.append(action_task('kick'))

    def run(self):
        self.tasks.clear()
        if self.next_state:
            self.next_state()
