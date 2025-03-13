#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/MasterPi_PC_Software/')
import ActionGroupControl as AGC
sys.path.append('/home/pi/MasterPi/')
import HiwonderSDK.Board as Board
print('''
*****************************************
**********功能：动作组调用历程*************
*****************************************
Tips：
    *按下Ctrl+C可关闭此程序运行，若失败请多次尝试！
*****************************************
      ''')

if __name__ == "__main__":
    try:
        AGC.runAction('1')
    except Exception as e:
        print("运行动作组时出现错误:", e)