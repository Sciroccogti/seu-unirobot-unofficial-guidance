#!/usr/bin/env python3
#coding: utf-8

import common
import sys
import SSHConnection
import config
import datetime


if __name__ == '__main__': 
    if not common.check_argv(sys.argv, 4):
        common.print_error('no enough arguments')
        exit(2)

    robot_id = sys.argv[1]
    if not common.check_id(robot_id):
        common.print_error('please check the robot id')
        exit(3)

    ssid = sys.argv[2]
    pswd = sys.argv[3]

    ip_address = common.get_ip(robot_id)
    netmask = '255.255.0.0'
    ssh_client = SSHConnection.SSHConnection(ip_address, config.ssh_port, config.username, config.password)
    cmd = 'python %s/data/script/%s %s %s %s %s; poweroff'%(config.remote_dir, config.wifi_script, ssid, pswd, ip_address, netmask)
    ssh_client.exec_command(cmd)