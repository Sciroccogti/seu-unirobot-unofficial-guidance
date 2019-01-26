#!/usr/bin/env python3
#coding: utf-8

import common
import sys
import SSHConnection
import config


if __name__ == '__main__': 
    if not common.check_argv(sys.argv):
        common.print_error('no enough arguments')
        exit(2)

    robot_id = sys.argv[1]
    if not common.check_id(robot_id):
        common.print_error('please check the robot id')
        exit(3)

    if not common.build_project():
        common.print_error('build error, please check code')
        exit(4)

    args = common.parse_argv(sys.argv)

    ip_address = common.get_ip(robot_id)
    ssh_client = SSHConnection.SSHConnection(ip_address, config.ssh_port, config.username, config.password)

    cmd = "sed -i '1d' %s; poweroff"%config.start_up_file
    ssh_client.exec_command(cmd)