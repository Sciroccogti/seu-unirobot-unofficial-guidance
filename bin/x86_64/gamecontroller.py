#!/usr/bin/env python3
#coding: utf-8

import socket
import observer
import threading
import struct
import time
from common import LOG


class GameData:
    GAMECONTROLLER_STRUCT_HEADER = 'RGme'
    TEAM_CYAN = 0
    TEAM_MAGENTA = 1
    DROPBALL = 255
    COMPETITION_PHASE_ROUNDROBIN = 0
    COMPETITION_PHASE_PLAYOFF = 1
    COMPETITION_TYPE_NORMAL = 0
    COMPETITION_TYPE_MIXEDTEAM = 1
    COMPETITION_TYPE_GENERAL_PENALTY_KICK = 2
    GAME_PHASE_NORMAL = 0
    GAME_PHASE_PENALTYSHOOT = 1
    GAME_PHASE_OVERTIME = 2
    GAME_PHASE_TIMEOUT = 3
    STATE_INITIAL = 0
    STATE_READY = 1
    STATE_SET = 2
    STATE_PLAYING = 3
    STATE_FINISHED = 4
    SET_PLAY_NONE = 0
    SET_PLAY_GOAL_FREE_KICK = 1
    SET_PLAY_PUSHING_FREE_KICK = 2
    PENALTY_NONE = 0
    PENALTY_HL_KID_BALL_MANIPULATION = 1
    PENALTY_HL_KID_PHYSICAL_CONTACT = 2
    PENALTY_HL_KID_ILLEGAL_ATTACK = 3
    PENALTY_HL_KID_ILLEGAL_DEFENSE = 4
    PENALTY_HL_KID_REQUEST_FOR_PICKUP = 5
    PENALTY_HL_KID_REQUEST_FOR_SERVICE = 6
    PENALTY_HL_KID_REQUEST_FOR_PICKUP_2_SERVICE = 7
    PENALTY_SUBSTITUTE = 14
    PENALTY_MANUAL = 15
    GAMECONTROLLER_RETURN_STRUCT_HEADER = "RGrt"
    GAMECONTROLLER_RETURN_STRUCT_VERSION = 3
    GAMECONTROLLER_RETURN_MSG_ALIVE = 0

    MAX_NUM_PLAYERS = 6

    RobotInfo = 'BB'
    TeamInfo = 'BBBBH'+RobotInfo*MAX_NUM_PLAYERS
    RoboCupGameControlData = '4sHBBBBBBBBBBHHH'+TeamInfo*2
    RoboCupGameControlReturnData = '4sBBBB'


class RobotInfo:
    PARAMS_NUM = 2
    def __init__(self, args):
        self.penalty = args[0]
        self.secsTillUnpenalised = args[1]


class TeamInfo:
    PARAMS_NUM = 17
    def __init__(self, args):
        self.teamNumber = args[0]
        self.teamColour = args[1]
        self.score = args[2]
        self.penaltyShot = args[3]
        self.singleShots = args[4]
        self.players = []
        for i in range(GameData.MAX_NUM_PLAYERS):
            self.players.append(RobotInfo(args[5+i*RobotInfo.PARAMS_NUM: 5+(i+1)*RobotInfo.PARAMS_NUM]))


class RoboCupGameControlData:
    def set_data(self, args):
        self.header = args[0]
        self.version = args[1]
        self.packetNumber = args[2]
        self.playersPerTeam = args[3]
        self.competitionPhase = args[4]
        self.competitionType = args[5]
        self.gamePhase = args[6]
        self.state = args[7]
        self.setPlay = args[8]
        self.firstHalf = args[9]
        self.kickingTeam = args[10]
        self.dropInTeam = args[11]
        self.dropInTime = args[12]
        self.secsRemaining = args[13]
        self.secondaryTime = args[14]
        self.teams = []
        for i in range(2):
            self.teams.append(TeamInfo(args[15+i*TeamInfo.PARAMS_NUM:15+(i+1)*TeamInfo.PARAMS_NUM]))

class RoboCupGameControlReturnData:
    def __init__(self, team, player):
        self.header = GameData.GAMECONTROLLER_RETURN_STRUCT_HEADER.encode('utf-8')
        self.version = GameData.GAMECONTROLLER_RETURN_STRUCT_VERSION
        self.team = team
        self.player = player
        self.message = GameData.GAMECONTROLLER_RETURN_MSG_ALIVE
    
    def data(self):
        return struct.pack(GameData.RoboCupGameControlReturnData, self.header, self.version, self.team, self.player, self.message)

class GameControllerReturn:
    def __init__(self, port):
        self.__port = port
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, data):
        self.__sock.sendto(data, ('', self.__port))

class GameController(observer.publisher):
    def __init__(self, portr, ports, team, player, period=1000):
        observer.publisher.__init__(self, 'gamecontroller')
        self.__port = portr
        self.__td = None
        self.is_alive = False
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__sock.setblocking(True)
        self.__sock.settimeout(1.0)
        self.__gcr_data = RoboCupGameControlReturnData(team,player).data()
        self.__gcr = GameControllerReturn(ports)
        self.__period = period
    
    def __del__(self):
        self.__td.join()
        observer.publisher.__del__(self)

    def start(self):
        try:
            self.__sock.bind(('', self.__port))
            self.alive = True
        except OSError as e:
            print(e.args)
            return False
        self.is_alive = True
        self.__td = threading.Thread(target=self.run)
        self.__td.start()
        return True

    def stop(self):
        self.is_alive = False
        self.__sock.close()
    
    def run(self):
        t_last = int(time.time()*1000)
        while self.is_alive:
            try:
                data, addr = self.__sock.recvfrom(1024)
                if len(data) != 0:
                    self.notify(observer.publisher.PUB_GAMECONTROLLER, struct.unpack(GameData.RoboCupGameControlData, data))
            except:
                pass
            try:
                t = int(time.time()*1000)
                if t-t_last >= self.__period:
                    self.__gcr.send(self.__gcr_data)
                    t_last = t
            except:
                pass


