#coding: utf-8 

import time
import Board
import signal




print('''
**********************************************************
********Function Huaner Technology Raspberry Pi Expansion board RGB light control routine*********
**********************************************************
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
Tips:
 * Press Ctrl +C to close the program. if it fails, please try again!


''')


start = True
#processing before closing
def Stop(signum, frame):
    global start

    start = False
    print('closing...')


#turn off all lights first
Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
Board.RGB.show()

signal.signal(signal.SIGINT, Stop)

while True:
    # set2 lights to red
    Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
    Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
    Board.RGB.show()
    time.sleep(1)
    
    # set 2 lights to green
    Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
    Board.RGB.show()
    time.sleep(1)
    
    #set2 lights to blue
    Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
    Board.RGB.show()
    time.sleep(1)
    
    # set 2 lights to yelow
    Board.RGB.setPixelColor(0, Board.PixelColor(255, 255, 0))
    Board.RGB.setPixelColor(1, Board.PixelColor(255, 255, 0))
    Board.RGB.show()
    time.sleep(1)

    if not start:
        #all lights off
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()
        print('Closed')
        break
        
