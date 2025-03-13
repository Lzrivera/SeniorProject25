#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/MasterPi/')
import time
import HiwonderSDK.Board as Board
from ArmIK.InverseKinematics import *
from ArmIK.ArmMoveIK import *
# 第2章 课程基础/2.机械臂逆运动学基础课程/第4课 机械臂上下左右移动 

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
if __name__ == "__main__":
    AK = ArmIK()
    '''
    AK.setPitchRangeMoving(coordinate_data, alpha, alpha1, alpha2, movetime):
    给定坐标coordinate_data和俯仰角alpha,以及俯仰角范围的范围alpha1, alpha2，自动寻找最接近给定俯仰角的解，并转到目标位置
    如果无解返回False,否则返回舵机角度、俯仰角、运行时间
    坐标单位cm， 以元组形式传入，例如(0, 5, 10)
    alpha: 为给定俯仰角
    alpha1和alpha2: 为俯仰角的取值范围
    movetime:为舵机转动时间，单位ms, 如果不给出时间，则自动计算    
    '''
    # 设置机械臂初始位置(x:0, y:6, z:18),运行时间:1500毫秒
    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1500) 
    time.sleep(1.5) # 延时1.5秒
    AK.setPitchRangeMoving((5, 6, 18), 0,-90, 90, 1000)  # 设置机械臂X轴右移,运行时间:1000毫秒
    time.sleep(1.2) # 延时1.2秒
    AK.setPitchRangeMoving((5, 13, 11), 0,-90, 90, 1000) #设置机械臂Y轴、Z轴同时移动，运行时间:1000毫秒
    time.sleep(1.2) # 延时1.2秒
    AK.setPitchRangeMoving((-5, 13, 11), 0,-90, 90, 1000) # 设置机械臂X轴右移,运行时间:1000毫秒
    time.sleep(1.2) # 延时1.2秒
    AK.setPitchRangeMoving((-5, 6, 18), 0,-90, 90, 1000)  #设置机械臂Y轴、Z轴同时移动，运行时间:1000毫秒
    time.sleep(1.2) # 延时1.2秒
    AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1000) # 设置机械臂X轴左移,运行时间:1000毫秒
    time.sleep(1.2) # 延时1.2秒
