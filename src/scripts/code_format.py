#!/usr/bin/env python3
#coding: utf-8

import os
import config


def list_dir(dir):
    _files = []
    files = os.listdir(dir)
    for i in range(0,len(files)):
           path = os.path.join(dir,files[i])
           if os.path.isdir(path):
              _files.extend(list_dir(path))
           if os.path.isfile(path):
              _files.append(path)
    return _files


if __name__ == '__main__':
    code_dir = config.project_dir + '/src'
    code_files = list_dir(code_dir)
    for code_file in code_files:
        if os.path.splitext(code_file)[1] in ['.c', '.h', '.hpp', '.cpp']:
            cmd = '''astyle --mode=c --style=ansi --indent=spaces=4 --add-brackets \
            --convert-tabs --indent-preprocessor --align-pointer=name --pad-oper \
            --pad-header --break-blocks --suffix=none --indent-switches \
            --min-conditional-indent=1 --indent-namespaces -p %s'''%code_file
            os.system(cmd)
