#use ssh to send this script to a raspberry pi

#Script to read input from limit switch and give the clear to take-off

import time     #import time library for delays
import gpiod    #import gpio library

def configPins():    #function configures gpio pins for test

    SW1 = 23    #create variables for pin numbers
    SW2 = 24

    chip = gpiod.Chip('gpiochip4')  #tell code where to find GPIO in memory

    global sw1_line     #allow gpio pin references to be accessed by other functions
    global sw2_line

    sw1_line = chip.get_line(SW1)   #store references to switch gpio pins
    sw2_line = chip.get_line(SW2)

    sw1_line.request(consumer="SW1", type=gpiod.LINE_REQ_DIR_IN)    #configure pins as input
    sw2_line.request(consumer="SW2", type=gpiod.LINE_REQ_DIR_IN)


def readPins():  #function reads the input from pins and sets doorsOpen variable
    global doorsOpen    #allow variable to tell if both doors are open to be accessed outside of function

    try:
        while 1:
            sw1_line = sw1_line.get_value()     #continually read inputs from switch pins
            sw2_line = sw2_line.get_value()
            if (sw2_line and sw2_line) == True:     #if both switches activated, set variable to say doors open
                doorsOpen = True
            else:
                doorsOpen = False
    finally:
        sw1_line.release()      #cleanup gpio pins before ending
        sw2_line.release()

def printDoorStatus():   #function outputs message whether doors are open
    while 1:
        if doorsOpen == True:                    #if both doors are open
            print ("Both doors are open")
        else:                                    #if one or both doors are not open
            print("One or both doors not fully open")


def main():
    configPins()
    readPins()
    printDoorStatus()

if __name__ == "__main__":
    main()
