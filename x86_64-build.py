#!/usr/bin/env python3

import os
import sys

if __name__ == '__main__': 
    build_dir = 'x86_64-build'
    t = '2'
    if len(sys.argv) == 2:
        t = sys.argv[1]
    if not os.path.exists(build_dir):
        os.mkdir(build_dir)
    cmd = 'cd %s; cmake ..; make install -j%s'%(build_dir, t)
    os.system(cmd)
