#test input and output to limit switch circuit using gpiozero library
#VERIFIED WORKS WITH HARDWARE

import gpiozero    #import libraries for gpio and delays
import time

##test switches as input gpio##-------------------------------------
sw1 = gpiozero.Button(23)    #read input from gpio pins 23 and 24
sw2 = gpiozero.Button(24)

while 1:    #continuously loop to read input from switches
    if sw1.is_pressed and sw2.is_pressed:    #if both doors open (switches pressed) print clear to launch
        print("CLEAR TO LAUNCH")
    if sw1.is_pressed:
        print("switch 1 pressed")
    if sw2.is_pressed:
        print("switch 2 pressed")
    print("\n")
    time.sleep(.5)
#-------------------------------------------------------------------

#test led as output-------------------------------------------------
led = gpiozero.led(24)

while 1:
  led.on()
  sleep(1)
  led.off()
  sleep(1)
