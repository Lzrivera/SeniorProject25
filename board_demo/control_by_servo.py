#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/MasterPi/')
import time
import HiwonderSDK.Board as Board

Board.setPWMServoPulse(1, 1300)
time.sleep(1)
Board.setPWMServoPulse(1, 1800)
time.sleep(1)

Board.setPWMServoPulse(1, 1300)
Board.setPWMServoPulse(3, 700)
time.sleep(1)
Board.setPWMServoPulse(1, 1500)
Board.setPWMServoPulse(3, 500)
time.sleep(1)
Board.setPWMServoPulse(1, 1300)
Board.setPWMServoPulse(3, 700)
time.sleep(1)
Board.setPWMServoPulse(1, 1500)
Board.setPWMServoPulse(3, 500)
time.sleep(1)


Board.setPWMServoPulse(4, 2200)
time.sleep(1)
Board.setPWMServoPulse(4, 2400)
time.sleep(1)


Board.setPWMServoPulse(5, 580)
time.sleep(1)
Board.setPWMServoPulse(5, 780)
time.sleep(1)


Board.setPWMServoPulse(1, 1300)
Board.setPWMServoPulse(6, 1300)
time.sleep(1)
Board.setPWMServoPulse(1, 1500)
Board.setPWMServoPulse(6, 1500)
time.sleep(1)
Board.setPWMServoPulse(1, 1300)
Board.setPWMServoPulse(6, 1700)
time.sleep(1)
Board.setPWMServoPulse(1, 1500)
Board.setPWMServoPulse(6, 1500)
time.sleep(1)