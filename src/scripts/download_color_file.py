#!/usr/bin/env python3
#coding: utf-8

import sys
import common
import config

if __name__ == '__main__':
    if not common.check_argv(sys.argv, 2):
        common.print_error('no enough arguments')
        exit(2)

    robot_id = sys.argv[1]
    if not common.check_id(robot_id):
        common.print_error('please check the robot id')
        exit(3)
    color_file = common.get_config('players.%s.color_file'%robot_id)
    if color_file is None:
        common.print_error('get color file error')
        exit(4)

    remote_path = config.remote_dir+"/data/"+color_file
    cmd = "python download.py %s %s"%(robot_id, remote_path)
    common.run_cmd(cmd)