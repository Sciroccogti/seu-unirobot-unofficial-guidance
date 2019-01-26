#!/usr/bin/env python3
#coding: utf-8

import client
import time
import task
from tcp import tcp_cmd_type, tcp_data_dir
import options
import sys
import os
import signal
import gamecontroller
import teammate
import subprocess
from options import OPTS
from config import CONF
from robot import ROBOT
from common import LOG
import fsm
from transitions import Machine

is_alive = False


def get_args():
    args = sys.argv[1:]
    args1 = []
    args2 = ''
    for arg in args:
        if '-p' in arg:
            args1.append(arg)
            args2 += ' %s'%arg
        elif '-h' in arg:
            args1.append(arg)
        elif '-t' in arg:
            args1.append(arg)
        elif '-g' in arg:
            args1.append(arg)
        else:
            args2 += ' %s'%arg
    return (args1, args2)

states = ['Start', 'Ready', 'Getup', 'SearchBall', 'GotoBall', 'KickBall']

if __name__ == '__main__':
    prog = './controller'
    (args1, args2) = get_args()
    if '-h' in args1:
        os.system('%s -h'%prog)
    OPTS.init(args1)
    CONF.init(OPTS.id)
    cmd = prog + args2
    controller = subprocess.Popen(cmd, shell=True)

    def signal_handler(sig, frame):
        global is_alive
        if sig == signal.SIGINT:
            is_alive = False

    signal.signal(signal.SIGINT,signal_handler)
    time.sleep(1)
    cl = client.client(int(CONF.get_config('net.tcp.port')))
    cl.attach(ROBOT)
    cl.start()
    is_alive = True
    while not cl.connected and is_alive:
        time.sleep(0.5)
    if not cl.connected:
        cl.dettach(ROBOT)
        cl.stop()
        exit(0)
    
    if OPTS.use_gamectrl:
        gc = gamecontroller.GameController(int(CONF.get_config('net.udp.gamectrl.recv_port')),int(CONF.get_config('net.udp.gamectrl.send_port')),\
            int(CONF.get_config('team_number')), OPTS.id, int(CONF.get_config('net.udp.gamectrl.period')))
        gc.attach(ROBOT)
        gc.start()
    if OPTS.use_teammate:
        tm = teammate.Teammate(int(CONF.get_config('net.udp.teammate.port')), int(CONF.get_config('net.udp.teammate.period')))
        tm.attach(ROBOT)
        tm.start()
    if OPTS.use_fsm:
        FSM = fsm.fsm()
        machine = Machine(model=FSM, states=states, ordered_transitions=True)
    cl.regsit(tcp_cmd_type.TASK_DATA, tcp_data_dir.DIR_SUPPLY)
    cl.regsit(tcp_cmd_type.WM_DATA, tcp_data_dir.DIR_APPLY)
    test_task = task.walk_task(0.02, 0.0, 0.0, True)
    while is_alive:
        try:
            if not cl.is_alive:
                is_alive = False
                break
            if controller.poll():
                break
                
            if cl.connected:
                if OPTS.use_fsm:
                    FSM.run()
                    for t in FSM.tasks:
                        cl.send(t.data())
                else:
                    cl.send(test_task.data())
        except:
            pass
        time.sleep(1)

    cl.dettach(ROBOT)
    cl.stop()
    if OPTS.use_teammate:
        tm.dettach(ROBOT)
        tm.stop()
    if OPTS.use_gamectrl:
        gc.dettach(ROBOT)
        gc.stop()
    controller.wait()

