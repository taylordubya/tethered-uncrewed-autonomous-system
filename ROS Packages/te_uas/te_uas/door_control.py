#Code to operate door motors and update variable to read whether doors are open
#
#Connections: Left-side motor [1]           (PUL - pin 13, DIR - pin 11)
# (Hardware)  Right-side motor [2]          (PUL - pin 10, DIR - pin 8)
#             Left door open switch [1]     (pin 15)
#             Right door open switch [2]    (pin 21)
#             Left door closed switch [1]   (pin 19)
#             Right door closed switch [2]  (pin 23)
# *note: all limit switches should be connected: NO (normally open) -> gpio input pin
#                                                GND (ground) -> raspberry pi GND 
#-----------------------------------------------------------------------------------------------------------------------------------------------

#import dependant libraries
import gpiozero
import time

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from te_uas_msgs.msg import DoorStatus, Log
from te_uas_msgs.srv import ControlDoor
import rclpy.qos
import rclpy.timer

class DoorControl(Node):
    def __init__(self):

        #ROS Node
        super().__init__('door_control')
    
        # Constants
        self.PUL_PER_REV = 200 #defined by switches on motor driver

        # Door status attributes (initialized to false until read)
        self.door_status = DoorStatus()
        self.door_status.opened = False
        self.door_status.right_opened = False
        self.door_status.left_opened = False
        self.door_status.right_closed = False
        self.door_status.left_closed = False


        self.closed = False

        qos_profile = rclpy.qos.qos_profile_sensor_data

        # Define Publisher 
        self.hangar_pub = self.create_publisher(DoorStatus, 'hangar/status', 10)

        # Define Service
        self.create_service(ControlDoor, 'hangar/doors_srvc', self.doors_cb)

        self.create_rate(20)    #read at 20Hz frequency

        # Start Publisher
        self.publish_door_status()


    # Callbacks
    def doors_cb(self, request, response):
        self.mainDoorControl(request.open) #call function to control door motors when toggle request sent

        response.opened = self.door_status.opened
        response.right_opened = self.door_status.right_opened
        response.door_status.left_opened = self.door_status.left_opened
        return response
    
    #functions to get limit switch outputs (one on each door to read if open, 1 to read if both closed)
    def getLeftOpen(self):    #read if left door open
        
        #initialize doors to read neither as closed or opened until switches read
        doorOneIsOpen = False
    
        #configure gpio pins on pi to read input from switches
        openSw1 = gpiozero.Button(22)    #set pin 15 (gpio 22) to door 1 open switch input
    
        #update variables to read if each switch is open
        if openSw1.is_pressed:
            doorOneIsOpen = True
    
        self.door_status.left_opened = doorOneIsOpen    #update class attribute
        return doorOneIsOpen
        
    def getRightOpen(self):    #read if right door open
        
        #initialize doors to read neither as closed or opened until switches read
        doorTwoIsOpen = False
    
        #configure gpio pin on pi to read input from switch
        openSw2 = gpiozero.Button(9)   #set pin 21 (gpio 9) to door 2 open switch input
    
        #update variables to read if each switch is open
        if openSw2.is_pressed:
            doorTwoIsOpen = True
    
        self.door_status.right_opened = doorTwoIsOpen    #update class attribute
        return doorTwoIsOpen
    
    def getLeftClosed(self):   #read if left door closed
    
        #initialize doors to read neither as closed or opened until switches read
        doorOneIsClosed = False
    
        #configure gpio pin on pi to read input from switch
        closeSw1 = gpiozero.Button(10)   #set pin 19 (gpio 10) to door closed switch input
    
        #update variables to read if each switch is open
        if (closeSw1.is_pressed):
            doorOneIsClosed = True
    
        self.door_status.left_closed = doorOneIsClosed    #update class attribute
        return doorOneIsClosed
    
    def getRightClosed(self):   #read if right door closed

        #initialize doors to read neither as closed or opened until switches read
        doorTwoIsClosed = False

        #configure gpio pin on pi to read input from switch
        closeSw2 = gpiozero.Button(11)   #set pin 23 (gpio 11) to door closed switch input

        #update variables to read if each switch is open
        if (closeSw2.is_pressed):
            doorTwoIsClosed = True

        self.door_status.right_closed = doorTwoIsClosed     #update class attribute
        return doorTwoIsClosed
    
    
    #function to control motors to open hangar doors
    def openDoors(self): 
    
        #configure gpio pins on raspberry pi to output to motors
        leftPUL = gpiozero.LED(27) #set pin 13 (gpio27) to left motor PUL output
        leftDIR = gpiozero.LED(17) #set pin 11 (gpio17) to left motor DIR output
        rightPUL = gpiozero.LED(15) #set pin 10  (gpio15) to right motor PUL output
        rightDIR = gpiozero.LED(14) #set pin 8 (gpio14) to right motor DIR output
        
        #set direction of motors to open
        leftDIR.on()  #set left motor to turn CCW
        rightDIR.off() #set right motor to turn CW 
        
        #calculate number of pulses needed to turn motors 1/4 turn for open
        pulCountToOpen = int(self.PUL_PER_REV / 4) * 13  #13 accounts for added gear ratio
    
        #calculate delay needed per pulse for .1 m/s opening speed
        pulsePerMin = 22.56 * self.PUL_PER_REV   #22.56 rpm estimated based on 25in lever arm
        microsecPerPulse = 60000000 / pulsePerMin   #calculate microsecond delay needed per pulse
        pulseDelay = microsecPerPulse / 1000000     #convert microseconds into seconds
        print("pulse delay: " + str(pulseDelay))
    
        #pulse motor loop
        for i in range(0,pulCountToOpen):   #pulse motors for max turn (1/4) or until limit switches read open
            #if both doors are read as open break loop
            if((self.getLeftOpen() and self.getRightOpen()) == True):
                print("both doors are open, ending pulse")
                return 0
            #if left door is read as open while still pulsing, only pulse right side
            elif (self.getLeftOpen() == True):   
                print("pulsing right motor only") 
                rightPUL.on()
                time.sleep(pulseDelay) #delay pulse by number of seconds
                rightPUL.off()
            #if right door is read as open while still pulsing, only pulse left side
            elif (self.getRightOpen() == True):
                print("pulsing left motor only") 
                leftPUL.on()
                time.sleep(pulseDelay) #delay pulse by number of seconds
                leftPUL.off()
            #if both doors are still not fully open
            elif ((self.getLeftOpen() and self.getRightOpen()) == False):
                print("pulsing")
                leftPUL.on() #pulse the signal
                rightPUL.on()
                time.sleep(pulseDelay) #delay pulse by number of seconds
                leftPUL.off()
                rightPUL.off()
        return 0
    
    #function to control motors to close hangar doors
    def closeDoors(self):
    
        #configure gpio pins on raspberry pi to output to motors
        leftPUL = gpiozero.LED(27) #set pin 13 (gpio27) to left motor PUL output
        leftDIR = gpiozero.LED(17) #set pin 11 (gpio17) to left motor DIR output
        rightPUL = gpiozero.LED(15) #set pin 10  (gpio15) to right motor PUL output
        rightDIR = gpiozero.LED(14) #set pin 8 (gpio14) to right motor DIR output
    
        #set direction of motors to close
        leftDIR.off()  #set motor 1 (right) to turn CW
        rightDIR.on() #set motor 2 (left) to turn CCW 
        
        #calculate number of pulses needed to turn motors 1/4 turn for open
        pulCountToClose = int(self.PUL_PER_REV / 4) * 13  #13 accounts for added gear ratio
    
        #calculate delay needed per pulse for .1 m/s closing speed
        pulsePerMin = 22.56 * self.PUL_PER_REV   #22.56 rpm estimated based on 25in lever arm
        microsecPerPulse = 60000000 / pulsePerMin   #calculate microsecond delay needed per pulse
        pulseDelay = microsecPerPulse / 1000000     #convert microseconds into seconds
        print("pulse delay: " + str(pulseDelay))
    
        #pulse motor loop
        for i in range(0,pulCountToClose):   #pulse motors for max turn (1/4) or until limit switches read closed
            #if both doors are read as closed break loop
            if((self.getLeftClosed() and self.getRightClosed()) == True):
                print("both doors are closed, ending pulse")
                return 0
            #if left door is read as closed while still pulsing, only pulse right side
            elif (self.getLeftClosed() == True):   
                print("pulsing right motor only") 
                rightPUL.on()
                time.sleep(pulseDelay) #delay pulse by number of seconds
                rightPUL.off()
            #if right door is read as closed while still pulsing, only pulse left side
            elif (self.getRightClosed() == True):
                print("pulsing left motor only") 
                leftPUL.on()
                time.sleep(pulseDelay) #delay pulse by number of seconds
                leftPUL.off()
            #if both doors are still not fully closed
            elif ((self.getLeftClosed() and self.getRightClosed()) == False):
                print("pulsing")
                leftPUL.on() #pulse the signal
                rightPUL.on()
                time.sleep(pulseDelay) #delay pulse by number of seconds
                leftPUL.off()
                rightPUL.off()
        return 0
        
    #function to control motors to close hangar doors in the case of not closed or open error
    def errorClose(self):
    
        #configure gpio pins on raspberry pi to output to motors
        leftPUL = gpiozero.LED(27) #set pin 13 (gpio27) to left motor PUL output
        leftDIR = gpiozero.LED(17) #set pin 11 (gpio17) to left motor DIR output
        rightPUL = gpiozero.LED(15) #set pin 10  (gpio15) to right motor PUL output
        rightDIR = gpiozero.LED(14) #set pin 8 (gpio14) to right motor DIR output
    
        #set direction of motors to close
        leftDIR.off()  #set motor 1 (right) to turn CW
        rightDIR.on() #set motor 2 (left) to turn CCW 
    
        #calculate delay needed per pulse for .1 m/s closing speed
        pulsePerMin = 22.56 * self.PUL_PER_REV   #22.56 rpm estimated based on 25in lever arm
        microsecPerPulse = 60000000 / pulsePerMin   #calculate microsecond delay needed per pulse
        pulseDelay = microsecPerPulse / 1000000     #convert microseconds into seconds
        print("pulse delay: " + str(pulseDelay))
    
        #calculate number of pulses needed to turn motors 1/4 turn for open
        pulCountToClose = int(self.PUL_PER_REV / 4) * 13    #13 accounts for added gear ratio
     
        #pulse motor loop
        for i in range(0,pulCountToClose):  #pulse motors for max turn (1/4) or until limit switches read closed
            #if both doors are read as closed break loop
            if((self.getLeftClosed() and self.getRightClosed()) == True):
                print("both doors are closed, ending pulse")
                return 0
            #if left door is read as closed while still pulsing, only pulse right side
            elif (self.getLeftClosed() == True):   
                print("pulsing right motor only") 
                rightPUL.on()
                time.sleep(pulseDelay) #delay pulse by number of seconds
                rightPUL.off()
            #if right door is read as closed while still pulsing, only pulse left side
            elif (self.getRightClosed() == True):
                print("pulsing left motor only") 
                leftPUL.on()
                time.sleep(pulseDelay) #delay pulse by number of seconds
                leftPUL.off()
            #if both doors are still not fully closed
            elif ((self.getLeftClosed() and self.getRightClosed()) == False):
                print("pulsing")
                leftPUL.on() #pulse the signal
                rightPUL.on()
                time.sleep(pulseDelay) #delay pulse by number of seconds
                leftPUL.off()
                rightPUL.off()
        return 0
    
    
    #main function to control door motors
    def mainDoorControl(self, request):
    
        #read door status from limit switch inputs
        self.door_status.opened = (self.getRightOpen() and self.getLeftOpen())
        self.closed = (self.getRightClosed() and self.getLeftClosed())
    
        #print status update at beginning of each loop iteration
        print("Open Status: " + str(self.door_status.opened) + ", Closed Status: " + str(self.closed))

        #handle case where no limit switches pressed (not open or closed)
        if (not(self.door_status.opened or self.closed)):
            self.errorClose()    #reset doors to closed

        #if reading as both open and closed, print error
        elif((self.door_status.opened and self.closed) == True):    #error where switches are reading both open and closed
            print("Error: reading both open and closed, please check switches")
            self.log_pub.publish(Log(level="Error", description="Reading both open and closed, please check switches"))

        #open case, command sent to open hangar doors
        elif(self.closed == True and request.open == True):
            print("Opening doors")
            self.log_pub.publish(Log(level="Info", description="Opening doors"))
            self.openDoors() #call function to open doors

        #close case, command sent to close hangar doors
        elif(self.door_status.opened == True and request.open == False):
            print("Closing doors")
            self.log_pub.publish(Log(level="Info", description="Closing doors"))
            self.closeDoors() #call function to close doors

        print("")   #format output spacing
        return 0    #main function returned successfully

    #publisher function to publish door status
    def publish_door_status(self):
        def door_status_callback():
            self.hangar_pub.publish(self.door_status)
        timer_period = 0.5  # Adjust as needed
        self.publish_timer = self.create_timer(timer_period, door_status_callback)
#-------------------------------------------------------------------------------------------------------------
def main():
    rclpy.init()
    node = DoorControl()
    rclpy.spin(node)

if __name__ == '__main__':
    main()