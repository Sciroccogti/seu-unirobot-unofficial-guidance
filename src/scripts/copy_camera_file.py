#!/usr/bin/env python3
#coding: utf-8

import config
import os
import shutil
import common
import sys

if __name__ == '__main__':
    if not common.check_argv(sys.argv, 2):
        common.print_error('no enough arguments')
        exit(2)

    robot_id = sys.argv[1]
    if not common.check_id(robot_id):
        common.print_error('please check the robot id')
        exit(3)
    camera_file = common.get_config('players.%s.camera_file'%robot_id)
    if camera_file is None:
        common.print_error('get camera file error')
        exit(4)

    new_camera_file = config.project_dir + '/bin/' + camera_file
    origin_camera_file = config.project_dir + '/src/' + camera_file
    os.rename(origin_camera_file, origin_camera_file+'.bak')
    shutil.copyfile(new_camera_file, origin_camera_file)