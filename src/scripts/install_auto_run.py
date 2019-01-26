#!/usr/bin/env python3
#coding: utf-8

import common
import sys
import SSHConnection
import config
import datetime


if __name__ == '__main__': 
    if not common.check_argv(sys.argv):
        common.print_error('no enough arguments')
        exit(2)

    robot_id = sys.argv[1]
    if not common.check_id(robot_id):
        common.print_error('please check the robot id')
        exit(3)

    if not common.build_project(True):
        common.print_error('build error, please check code')
        exit(4)

    if not common.compress_files():
        common.print_error('compress files error, please check')
        exit(5)
        
    args = common.parse_argv(sys.argv)

    ip_address = common.get_ip(robot_id)
    ssh_client = SSHConnection.SSHConnection(ip_address, config.ssh_port, config.username, config.password)
    ssh_client.upload(config.local_dir+config.compress_file_name, config.remote_dir+config.compress_file_name)

    nowTime=datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    cmd = 'date -s "%s"; cd %s; tar zxvf %s; echo %s %s >> %s; poweroff'%(nowTime, config.remote_dir,\
             config.compress_file_name, config.exec_file_name, args, config.start_up_file)
    ssh_client.exec_command(cmd)