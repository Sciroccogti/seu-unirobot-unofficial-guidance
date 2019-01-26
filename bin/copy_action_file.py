#!/usr/bin/env python3
#coding: utf-8

import config
import os
import shutil
import sys
import common


if __name__ == '__main__':
    if not common.check_argv(sys.argv, 2):
        common.print_error('no enough arguments')
        exit(2)

    robot_id = sys.argv[1]
    if not common.check_id(robot_id):
        common.print_error('please check the robot id')
        exit(3)
    action_file = common.get_config('players.%s.action_file'%robot_id)
    if action_file is None:
        common.print_error('get action file error')
        exit(4)

    new_action_file = config.project_dir + '/bin/' + action_file
    origin_action_file = config.project_dir + '/src/' + action_file
    os.rename(origin_action_file, origin_action_file+'.bak')
    shutil.copyfile(new_action_file, origin_action_file)
