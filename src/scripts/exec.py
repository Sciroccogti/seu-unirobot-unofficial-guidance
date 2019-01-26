#!/usr/bin/env python3
#coding: utf-8

import sys
import SSHConnection
import common
import config


if __name__ == '__main__':
    common.print_info('''how to use
    function: run command in remote machine
    example: python exec.py id cmd1 cmd2 cmd3 ...
    ''')

    if not common.check_argv(sys.argv, 3):
        common.print_error('no enough arguments')
        exit(2)
        
    robot_id = sys.argv[1]
    if not common.check_id(robot_id):
        common.print_error('please check the robot id')
        exit(3)

    ip_address = common.get_ip(robot_id)
    ssh_client = SSHConnection.SSHConnection(ip_address, config.ssh_port, config.username, config.password)
    for i in range(2, len(sys.argv)):
        ssh_client.exec_command(sys.argv[i])