#Code to operate door motors and update variable to read whether doors are open
#
#Connections: Left-side motor [1]           (PUL - pin 13, DIR - pin 15)
# (Hardware)  Right-side motor [2]          (PUL - pin 8, DIR - pin 10)
#             Left motor open switch [1]    (pin 19)
#             Right motor open switch [2]   (pin 21)
#             Doors closed switch           (pin 23)
#  *note: all limit switch should be connected NO (normally open)
#-----------------------------------------------------------------------------------------------------------------------------------------------
#import dependant libraries
import gpiozero
import time

#initialize motor/driver constants
PUL_PER_REV = 200 #defined by switches on motor driver

#function to get open command
def getOpenCmd():
    #read in command to open doors
    openCmd = False #initialize open command to be False
    openCmdStr = str(input("Door commands: type 'o' to open or press ENTER to skip command   "))   #FIXME get input from user (read from GUI in future?)
    #set command based on user input
    if(openCmdStr == 'o'):
        openCmd = True
    else:
        openCmd = False
    return openCmd
  
#function to get close command
def getCloseCmd():
    #read in command to close doors
    closeCmd = False #initialize close command to be False
    closeCmdStr = str(input("Door commands: type 'c' to close or press ENTER to skip command   "))   #FIXME get input from user (read from GUI in future?)
    #set command based on user input
    if(closeCmdStr == 'c'):
        closeCmd = True
    else:
        closeCmd = False
    return closeCmd
  
#functions to get limit switch outputs (one on each door to read if open, 1 to read if both closed)
def getLimSwOpen1():    #read if left door open
    
    #initialize doors to read neither as closed or opened until switches read
    doorOneIsOpen = False
    #configure gpio pins on pi to read input from switches
    openSw1 = gpiozero.Button(10)    #set pin 19 (gpio 10) to door 1 open switch input
    #update variables to read if each switch is open
    if openSw1.is_pressed:
        doorOneIsOpen = True
    return doorOneIsOpen
    
def getLimSwOpen2():    #read if right door open
    
    #initialize doors to read neither as closed or opened until switches read
    doorTwoIsOpen = False
    #configure gpio pin on pi to read input from switch
    openSw2 = gpiozero.Button(9)   #set pin 21 (gpio 9) to door 2 open switch input
    #update variables to read if each switch is open
    if openSw2.is_pressed:
        doorTwoIsOpen = True
    return doorTwoIsOpen
  
def getLimSwClosed():   #read if doors closed
    #initialize doors to read neither as closed or opened until switches read
    doorsClosed = False
    #configure gpio pin on pi to read input from switch
    closeSw = gpiozero.Button(11)   #set pin 23 (gpio 11) to door closed switch input
    #update variables to read if each switch is open
    if (closeSw.is_pressed):
        doorsClosed = True
    return doorsClosed
  
#function to control motors to open hangar doors
def openDoors(): 
    #configure gpio pins on raspberry pi to output to motors
    m1PUL = gpiozero.LED(27) #set pin 13 (gpio27) to motor 2 PUL output
    m1DIR = gpiozero.LED(22) #set pin 15 (gpio22) to motor 2 DIR output
    m2PUL = gpiozero.LED(14) #set pin 8  (gpio14) to motor 1 PUL output
    m2DIR = gpiozero.LED(15) #set pin 10 (gpio15) to motor 1 DIR output
    
    #set direction of motors to open
    m1DIR.on()  #set motor 1 (right) to turn CCW
    m2DIR.off() #set motor 2 (left) to turn CW 
    
    #calculate number of pulses needed to turn motors 1/4 turn for open
    pulCountToOpen = int(PUL_PER_REV / 4) * 15  #**FIXME can be changed depending on final door design to be 1/2 revolution
  
    #calculate delay needed per pulse for .1 m/s opening speed
    pulsePerMin = 22.56 * PUL_PER_REV   #22.56 rpm estimated based on 25in lever arm **FIXME based on final design - recalculate rpm (formula in function in reel motor code)
    microsecPerPulse = 60000000 / pulsePerMin   #calculate microsecond delay needed per pulse
    pulseDelay = microsecPerPulse / 1000000     #convert microseconds into seconds
    print("pulse delay: " + str(pulseDelay))
  
    #pulse motor loop
    for i in range(0,pulCountToOpen):   #only pulse set number of times
        #if both doors are read as open break loop
        if((getLimSwOpen1() and getLimSwOpen2()) == True):
            print("both doors are open, ending pulse")
            break
        #if left door is read as open while still pulsing, only pulse right side
        elif (getLimSwOpen1() == True):   
            print("pulsing motor 2 only") 
            m2PUL.on()
            time.sleep(pulseDelay) #delay pulse by number of seconds
            m2PUL.off()
        #if right door is read as open while still pulsing, only pulse left side
        elif (getLimSwOpen2() == True):
            print("pulsing motor 1 only") 
            m1PUL.on()
            time.sleep(pulseDelay) #delay pulse by number of seconds
            m1PUL.off()
            
        #if both doors are still not fully open
        elif ((getLimSwOpen1() and getLimSwOpen2()) == False):
            print("pulsing")
            m1PUL.on() #pulse the signal
            m2PUL.on()
            time.sleep(pulseDelay) #delay pulse by number of seconds
            m1PUL.off()
            m2PUL.off()
    return 0
  
#function to control motors to close hangar doors
def closeDoors():
    #configure gpio pins on raspberry pi to output to motors
    m1PUL = gpiozero.LED(27) #set pin 13 (gpio27) to motor 2 PUL output
    m1DIR = gpiozero.LED(22) #set pin 15 (gpio22) to motor 2 DIR output
    m2PUL = gpiozero.LED(14) #set pin 8  (gpio14) to motor 1 PUL output
    m2DIR = gpiozero.LED(15) #set pin 10 (gpio15) to motor 1 DIR output
    #set direction of motors to close
    m1DIR.off()  #set motor 1 (right) to turn CW
    m2DIR.on() #set motor 2 (left) to turn CCW 

    #calculate number of pulses needed to turn motors 1/4 turn for open
    pulCountToClose = int(PUL_PER_REV / 4) * 15  

    #calculate delay needed per pulse for .1 m/s closing speed
    pulsePerMin = 22.56 * PUL_PER_REV   #22.56 rpm estimated based on 25in lever arm **FIXME based on final design - recalculate rpm (formula in function in reel motor code)
    microsecPerPulse = 60000000 / pulsePerMin   #calculate microsecond delay needed per pulse
    pulseDelay = microsecPerPulse / 1000000     #convert microseconds into seconds
    print("pulse delay: " + str(pulseDelay))
    #pulse motor loop
    for i in range(0, pulCountToClose): #pulse only set number of times

        #if doors are read as closed while still trying to pulse, break loop
        if(getLimSwClosed() == True):
            print("both doors are closed, ending pulse")
            break

        #if doors are not read as closed, continue
        elif (getLimSwClosed() == False):
            print("pulsing")
            m1PUL.on() #pulse the signal
            m2PUL.on()
            time.sleep(pulseDelay) #delay pulse by number of seconds
            m1PUL.off()
            m2PUL.off()

    return 0

#function to control motors to close hangar doors in the case of not closed or open error
def errorClose():
    #configure gpio pins on raspberry pi to output to motors
    m1PUL = gpiozero.LED(27) #set pin 13 (gpio27) to motor 2 PUL output
    m1DIR = gpiozero.LED(22) #set pin 15 (gpio22) to motor 2 DIR output
    m2PUL = gpiozero.LED(14) #set pin 8  (gpio14) to motor 1 PUL output
    m2DIR = gpiozero.LED(15) #set pin 10 (gpio15) to motor 1 DIR output
    #set direction of motors to close
    m1DIR.off()  #set motor 1 (right) to turn CW
    m2DIR.on() #set motor 2 (left) to turn CCW 

    #calculate delay needed per pulse for .1 m/s closing speed
    pulsePerMin = 22.56 * PUL_PER_REV   #1.504 rpm estimated based on 25in lever arm **FIXME based on final design - recalculate rpm (formula in function in reel motor code)
    microsecPerPulse = 60000000 / pulsePerMin   #calculate microsecond delay needed per pulse
    pulseDelay = microsecPerPulse / 1000000     #convert microseconds into seconds
    print("pulse delay: " + str(pulseDelay))

    #calculate number of pulses needed to turn motors 1/4 turn for open
    pulCountToClose = int(PUL_PER_REV / 4) * 15  

    for i in range(pulCountToClose):  #pulse motors for max turn (1/4) or until limit switch reads as closed 

        if (getLimSwClosed() == True):  #break pulse loop and return if doors are read as closed
            return 0

        print("pulsing")
        m1PUL.on() #pulse the signal
        m2PUL.on()
        time.sleep(pulseDelay) #delay pulse by number of seconds
        m1PUL.off()
        m2PUL.off()
    return 0

#function to clear commands
def clearDoorCmd(openOrCloseCmd):
    openOrCloseCmd = False
    return openOrCloseCmd
  
#main function to control door motors
def main():
    #initialize door status based on limit switch inputs
    isOpen = (getLimSwOpen1() and getLimSwOpen2())
    isClosed = getLimSwClosed()
    #handle case where no limit switches pressed (not open or closed)
    if (not(isOpen or isClosed)):
        errorClose()    #reset doors to closed
        isClosed = getLimSwClosed()   #update closed variable
            
    while(1):   #continually loop to see if doors should open or close
        #check whether doors open or closed at each loop
        isOpen = (getLimSwOpen1() and getLimSwOpen2())
        isClosed = getLimSwClosed()
        #print status update at beginning of each loop iteration
        print("Open Status: " + str(isOpen) + ", Closed Status: " + str(isClosed))
        #at the beginning of each loop iteration, read commands to determine operation
        openCmd = getOpenCmd()
        closeCmd = getCloseCmd()
        #recheck whether doors open or closed before executing command
        isOpen = (getLimSwOpen1() and getLimSwOpen2())
        isClosed = getLimSwClosed()
        #error case, trying to both close and open simultaneously
        if ((openCmd and closeCmd) == True):    
            print("Error: trying to close and open at the same time") #print error message
        #open case, command sent to open hangar doors
        elif(openCmd == True):
            if((isOpen and isClosed) == True):    #error where switches are reading both open and closed
                print("Error: reading both open and closed, please check switches")
            elif(isOpen == True): #check if doors are already open
                print("Doors already open")
            else:   #if doors are not already open, then open doors
                print("Opening doors")
                openDoors() #call function to open doors
        #close case, command sent to close hangar doors
        elif(closeCmd == True):
            if((isOpen and isClosed) == True):    #error where switches are reading both open and closed
                print("Error: reading both open and closed, please check switches")
            elif(isClosed == True):   #check if doors are already closed
                print("Doors are already closed")
            else:   #if doors are not already closed, then close doors
                print("Closing doors")
                closeDoors() #call function to close doors
        
        #in other cases, no command is given, so continue
        else:
            print("No command recieved, continue")
        #no matter case, clear open and close commands at the end of each loop iteration
        clearDoorCmd(openCmd)
        clearDoorCmd(closeCmd)
        print("")   #format output spacing
    return 0    #main function returned successfully

main()
