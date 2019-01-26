#!/usr/bin/env python3
#coding: utf-8

import argparse

class Options:
    def __init__(self):
        self.__parser = argparse.ArgumentParser()
        self.__parser.add_argument('-p', '--player', type=int, help='player id, default: 0')
        self.__parser.add_argument('-g', '--gamectrl', type=bool, help='use gamecontroller, default: False')
        self.__parser.add_argument('-t', '--teammate', type=bool, help='use teammate info, default: False')
        self.__parser.add_argument('-f', '--fsm', type=bool, help='use fsm, default: True')
        
    def init(self, argv):
        args = None
        try:
            args = self.__parser.parse_args(argv)
        except:
            exit(1)
        self.id = 0 if args.player is None else args.player
        self.use_gamectrl = False if args.gamectrl is None else args.gamectrl
        self.use_teammate = False if args.teammate is None else args.teammate
        self.use_fsm = True if args.fsm is None else args.fsm

OPTS = Options()