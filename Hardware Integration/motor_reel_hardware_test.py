#Code to control reel motor
#------------------------------------------------------------------------------------------------------------------------

#test inputs and constants
REEL_DIAMETER = .085 #meters
PUL_PER_REV = 200 #defined by switches on motor driver
MAX_TETHER_LENGTH = 20 #meters
MIN_TETHER_LENGTH = 0 #meters

initialAmtTetherUnspooled = 0.0 #meters

reel_velocity = 1 #m/s  - test input**************************************

#----------------------------------------------------------------------------------------------------------------------
#control loop function to calculate reel_velocity to feed into motor controls
def ControlLoop():
  pass

#function to keep track of how much tether has been released (call at every pulse to motor)
def updateTetherUnspooled(amtTetherUnspooled, direction, atMaxLength, atMinLength):
  
  #import necessary libraries for calculations
  import math
  pi = math.pi    #define variable to use number pi
  
  #calculate distance unspooled per pulse
  revsPerPulse = 1 / PUL_PER_REV  #calculate number of revolutions (fractional) moved per pulse
  metersPerPulse = (pi * REEL_DIAMETER) * revsPerPulse #circumference * (fraction of circumference rotated)

  #determine spooling or unspooling and update amount of tether length unspooled
  if (direction == 1): #CCW - unspooling
    amtTetherUnspooled += metersPerPulse  #through each pulse when function is called, will increment tether length with length unspooled per pulse
    atMinLength = False
  elif (direction == 0): #CW - spooling
    amtTetherUnspooled -= metersPerPulse  #same as CCW but decrementing tether length

  #exception handling
  if (amtTetherUnspooled > (MAX_TETHER_LENGTH - metersPerPulse)): #raise an error if calculated length is more than the max tether length
    print("Error: trying to exceed max tether length")
    atMaxLength = True
  if (amtTetherUnspooled < metersPerPulse):    #raise error is calculated length is less than 0
    print("Error: no tether left to spool")
    atMinLength = True
 
  return amtTetherUnspooled, atMaxLength, atMinLength

#--------------------------------------------------------------------------------------------------------------------------
#functions for use in the tether-reel calculations, etc

#function to convert reel_velocity in meters/second to rpm
def MStoRPM(reel_velocity):
    
  #import necessary libraries for calculations
  import math
  pi = math.pi    #define variable to use number pi

  print("reel velocity m/s: " + str(reel_velocity))
  if (reel_velocity != 0):
    reel_velocity_rpm = (abs(reel_velocity) / (pi*REEL_DIAMETER)) * 60 #calculate rpm from m/s
    #print("reel velocity rpm: " + str(reel_velocity_rpm))
  else:   #handle case where velocity is 0
    reel_velocity_rpm = 0.0
    #print("reel velocity rpm: 0")
  return reel_velocity_rpm
  
#function to calculate time delay per pulse in microseconds
def DelayPerPulse(reel_velocity):
  reel_velocity_rpm = MStoRPM(reel_velocity)
  pulsePerMin = reel_velocity_rpm * PUL_PER_REV  #calculate pulse per minute by rpm times pulse per revolution
  #print("pulse per minute: " + str(pulsePerMin))
  if (pulsePerMin != 0):
    microsecPerPulse = 60000000 / pulsePerMin #calculate microsecond delay needed per pulse
    pulseDelay = microsecPerPulse / 1000000   #convert microseconds into seconds
  else:
    pulseDelay = 0.0
  return pulseDelay

#function to set direction of spin (based on positive or negative velocity)
def DefDir(reel_velocity, dir):
  if(reel_velocity >= 0):  #if drone is ascending or at standstill, spin CCW (DIR pin ON)
    dir.on()
    direction = 1 #set direction to 1 (CCW)
  else:  #if drone is descending, spin CW (DIR pin OFF)
    dir.off()
    direction = 0 #set direction to 0 (CW)
  return direction
#---------------------------------------------------------------------------------------------------------------------------

#function to control motor through GPIO output
def GpioMotorControl(reel_velocity):

  #import libraries to use gpio outputs to control motor
  import gpiozero
  import time

  pul = gpiozero.LED(24) #set pin 18 (gpio24) to PUL output
  dir = gpiozero.LED(23)  #set pin 16 (gpio23) to DIR output

  direction = DefDir(reel_velocity, dir) #set direction of motor (off[0] = CW, on[1] = CCW)

  pulseDelay = DelayPerPulse(reel_velocity)   #get the delay per pulse based on the desired velocity (m/s) of reel
  print("pulse delay: " + str(pulseDelay) + " microseconds")

  #initialize amount of tether that is unspooled to be 0 meters
  amtTetherUnspooled = initialAmtTetherUnspooled #meters
  print("\nAmount of tether unspooled: " + str(amtTetherUnspooled) + " meters\n")

  #initialize max and min length tracking variables
  atMaxLength = False   #variable to keep track of tether, whether at max length
  atMinLength = False   #variable to keep track of tether, whether at min length (0 meters unspooled)
  if (amtTetherUnspooled <= 0): #if fully reeled in, update minimum length variable
    atMinLength = True

  while (1):  #continually loop through pulses sent to motor
    if (pulseDelay != 0.0): #if drone is moving, turn the motor
      #if (atminLength):

      print("pulsing")
      pul.on() #pulse the signal
      time.sleep(pulseDelay) #delay pulse by number of seconds
      pul.off()
      print("pulse delay: " + str(pulseDelay))
      

      tetherTuple = updateTetherUnspooled(amtTetherUnspooled, direction, atMaxLength, atMinLength) #with every pulse, update the current amount of tether unspooled
      amtTetherUnspooled = tetherTuple[0] #update amount of tether unspooled variable
      atMaxLength = tetherTuple[1] #update whether tether has reached max length
      atMinLength = tetherTuple[2]
      print("Amount of tether unspooled: " + str(amtTetherUnspooled) + " meters")
      print("atMaxLength: " + str(atMaxLength))
      print("atMinLength: " + str(atMinLength) + "\n")

      if (atMaxLength): #if max tether length reached, stop pulsing motor
        pulseDelay = 0.0

    else: #if drone is stationary, hold position
      print("velocity 0 or max tether length reached, not reeling")
      time.sleep(0.001)  #delay 1 millisecond
    
    #update pulse delay
    #pulseDelay = DelayPerPulse(reel_velocity)   #get the delay per pulse based on the desired velocity (m/s) of reel
  
  print("Reel motor rotation ended")  #exited pulse loop

  return amtTetherUnspooled


GpioMotorControl(reel_velocity)
