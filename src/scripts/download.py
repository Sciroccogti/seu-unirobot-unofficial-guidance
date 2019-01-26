#!/usr/bin/env python3
#coding: utf-8

import common
import sys
import SSHConnection
import config


if __name__ == '__main__': 
    if not common.check_argv(sys.argv, 3):
        common.print_error('no enough arguments')
        exit(2)

    robot_id = sys.argv[1]
    if not common.check_id(robot_id):
        common.print_error('please check the robot id')
        exit(3)

    ip_address = common.get_ip(robot_id)
    remote_path = sys.argv[2]
    ssh_client = SSHConnection.SSHConnection(ip_address, config.ssh_port, config.username, config.password)
    ssh_client.download(remote_path, './')