#new reel motor script (improved readability and coordination with drone data)

#imports
import time
import math
import gpiozero

#ROS imports
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy.qos
import rclpy.timer
from std_msgs.msg import Int32, Float64

import matplotlib.pyplot as plt

#ROS2 NODE
class HangarControl(Node):

    #Self
    def __init__(self):

        #constants
        self.REEL_DIAMETER = .1801 #meters - .1801 is exact for diameter, smaller diameter creates some slack (.1799 ~ 0.68m slack at 18m)
        self.PUL_PER_REV = 200 #defined by switches on motor driver
        self.MAX_TETHER_LENGTH = 18 #meters
        self.MIN_TETHER_LENGTH = 0 #meters

        #attributes for function use - initialized values
        self.tether_unspooled = 0.0  #0 meters
        self.at_min_length = True    #at minimum length
        self.at_max_length = False   #not at maximum length

        self.direction = 1           #unspool direction (CCW)
        self.reel_velocity = 0.0     #0 m/s

        self.corrected_alt = 0.0     #0 meters
        self.calculated_alt = 0.0    #0 meters
        self.alt_diff = 0            #0 m/s
        self.loop_count = 0          #0 number of times reel control function has looped
        
        self.vel_queue = []          #queue of most recent velocities (initialize all to 0)
        self.hovering_alt = 0        #0 meters
        self.last_state = ""         #null last state initial
        
        #initialize queue to set number of elements for velocity moving average function
        num_elements = 10   #change this value to tune reactivity of script to changes in drone velocity
        for i in range(num_elements):
            self.vel_queue.append(0.0)

        #ROS Node
        super().__init__('hangar_control')

        #attributes for subscribed data
        self.current_velocity = float()
        self.current_altitude = float()
        self.current_state = State()

        qos_profile = rclpy.qos.qos_profile_sensor_data
        
        # Define Subscribers
        self.local_vel_sub = self.create_subscription(TwistStamped, 'mavros/local_position/velocity_local', self.vel_cb, qos_profile)
        self.local_pose_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.alt_cb, qos_profile)
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, qos_profile)

        # Define Publishers
        self.tether_unspooled_pub = self.create_publisher(Float64, 'hangar/tether_unspooled', 10)
        self.tether_unspool_speed_pub = self.create_publisher(Float64, 'hangar/tether_unspool_speed', 10)

        #set subscribe/publish rate
        self.create_rate(20)

        # #create lists of data to graph altitude and amount of tether unspooled over time
        # self.altitude_list = []
        # self.tether_unspooled_list = []

        timer_period = 0.0009   #seconds, rate at which reel control function will loop
        self.reel_loop = self.create_timer(timer_period, self.mainReelControl)

    # TELEMETRY CALLBACKS-------------------------------------------------------
    #current velocity callback function
    def vel_cb(self, vel):
        self.current_velocity = vel.twist.linear.z

    #current altitude callback function
    def alt_cb(self, alt):
        self.current_altitude = alt.pose.position.z
        if(self.loop_count == 0):     #initialize altitude to 0m on first loop
            self.alt_diff = self.current_altitude
        else:       #adjust all altitude measurements based on initialized value
            self.corrected_alt = self.current_altitude - self.alt_diff
            
    #current state callback function
    def state_cb(self, state):
        self.current_state = state

    #SETTERS----------------------------------------------------------------------
    #function to set reel_velocity (control loop function)
    def setReelVelocity(self):
        #Check current drone mode to determine reel operation
        
        #Hover state (AUTO.LOITER)
        if self.current_state.mode == "AUTO.LOITER":  
            #detect if there was change in current drone state
            if self.current_state != self.last_state:   #if changed states (just entered hover mode)
                self.hovering_alt = self.calculated_alt #set hovering altitude to current altitude (calculated)
                self.reel_velocity = 0.0 #m/s, no reel movement
                
            else: #if maintaining hover
                #if within 0.5 meters of initialized hovering altitude and above threshold velocity, going up
                if (abs(self.calculated_alt - self.hovering_alt) < 0.5) and (self.current_velocity > 0.05): 
                    self.reel_velocity = self.current_velocity #set reel to match drone velocity
                    
                #if within 0.5 meters of initialized hovering altitude and above threshold velocity, going down
                elif (abs(self.calculated_alt - self.hovering_alt) < 0.5) and (self.current_velocity < -0.05):
                    for i in range(len(self.vel_queue)):#find average of most recent velocities
                        vel_sum += self.vel_queue[i] 
                    vel_avg = vel_sum / len(self.vel_queue) #calculate average velocity
                    self.reel_velocity = vel_avg #set reel to turn at the average speed of the most recent velocities
                    
                else:   #if outside of altitude range or below threshold velocity, stop the reel 
                    self.reel_velocity = 0.0 #m/s
            
        #Landing state (AUTO.LAND)
        elif self.current_state.mode == "AUTO.LAND":    #only allow reel to spool in
            if self.current_velocity > -0.05: #if drone is moving up (positive velocity) or moving down slower than threshold (0.05 m/s), stop reel
                self.reel_velocity = 0.0 #m/s
                
            else:   #otherwise drone is moving down (at speed above threshold), so match reel speed
                if not self.at_min_length:  #as long as there is still tether to reel, continue spooling
                    self.reel_velocity = self.current_velocity
                else:   #stop tether when fully spooled in
                    self.reel_velocity = 0.0 #m/s
                    
        #Takeoff state (AUTO.TAKEOFF)
        elif self.current_state.mode == "AUTO.TAKEOFF": #only allow reel to spool out
        
            #if drone is on platform (<1.5m altitude) at velocity 0 +/-0.4 m/s (and supposed to be unreeling), then unreel slowly (give some initial slack), until reaches 10 cm slack
            if((self.corrected_alt <= 1.5) and (abs(self.current_velocity) <= 0.2) and (self.tether_unspooled < .10)):  #.10 m is initial amount of tether that will be unspooled
                self.reel_velocity = .05 #m/s, FIXME adjust .05 m/s to change initial unspool speed as needed
                
            #if drone is moving up (at speed above threshold), match reel speed
            elif self.current_velocity >= 0.05:   
                if not self.at_max_length:  #as long as there is still tether to reel out, continue unspooling
                    self.reel_velocity = self.current_velocity
                else:   #stop tether when fully unspooled
                    self.reel_velocity = 0.0 #m/s
                    
            #otherwise, drone is moving down (negative velocity) or moving up slower than threshold (0.05 m/s), so stop reel
            else:
                self.reel_velocity = 0.0 #m/s
            
        #Altitude change state, direction is variable (OFFBOARD)
        elif self.current_state.mode == "OFFBOARD": 

            #if fully spooled and in direction to spool, stop rotation (set reel velocity = 0)
            if (self.at_min_length and (self.direction == 0)):
                self.reel_velocity = 0.0 #m/s

            #if fully unspooled and in direction to unspool, stop rotation (set reel velocity = 0)
            elif(self.at_max_length and (self.direction == 1)):
                self.reel_velocity = 0.0 #m/s

            #if drone is on platform (<1.5m altitude) at velocity 0 +/-0.4 m/s (and supposed to be unreeling), then unreel slowly (give some initial slack), until reaches 10 cm slack
            if((self.corrected_alt <= 1.5) and (abs(self.current_velocity) <= 0.4) and (self.tether_unspooled < .10)):  #.10 m is initial amount of tether that will be unspooled
                self.reel_velocity = .05 #m/s, FIXME adjust .05 m/s to change initial unspool speed as needed

            #if drone is above threshold, and not trying to exceed tether bounds
            elif (abs(self.current_velocity) > (0.05)):   #0.05 is the threshold velocity, anything lower will not cause reel motion
            
                #find average of most recent velocities
                for i in range(len(self.vel_queue)):#find average of most recent velocities
                    vel_sum += self.vel_queue[i] 
                vel_avg = vel_sum / len(self.vel_queue) #calculate average velocity
                
                self.reel_velocity = vel_avg #set reel to turn at the average speed of the most recent velocities
                #self.reel_velocity = self.current_velocity #set reel to turn at same rate as drone velocity

            #otherwise, don't turn if velocity is too low
            else:
                self.reel_velocity = 0.0 #m/s
                
        #unknown or unaccounted-for state
        else:
            self.reel_velocity = 0.0 #m/s

        return self.reel_velocity

    #function to set direction of spin (based on positive or negative velocity)
    def setDirection(self):
    
        #unspool: if drone is ascending, spin CCW
        if(self.reel_velocity > 0):  
            self.direction = 1 #set direction to 1 (CCW)

        #spool: if drone is descending, spin CW
        elif(self.reel_velocity < 0):  
            self.direction = 0 #set direction to 0 (CW)

        #if fully reeled in, reset direction to unspool, spin CCW
        elif ((abs(self.reel_velocity) == 0) and self.at_min_length):
            self.direction = 1 #drone velocity is 0 (drone is at standstill or max length reached), no change in direction (will change when drone begins descent)
        return 0

    #function to update the amount of tether unspooled, after each pulse
    def updateTetherUnspooled(self):

        #calculate distance unspooled per pulse
        revs_per_pulse = 1 / self.PUL_PER_REV  #calculate number of revolutions (fractional) moved per pulse
        length_per_pulse = (math.pi * self.REEL_DIAMETER) * revs_per_pulse #meters, circumference * (fraction of circumference rotated)

        #determine spooling or unspooling and update amount of tether length unspooled
        if (self.direction == 1): #CCW - unspooling
            self.tether_unspooled += length_per_pulse  #through each pulse when function is called, will increment tether length with length unspooled per pulse
            self.at_min_length = False
        elif (self.direction == 0): #CW - spooling
            self.tether_unspooled -= length_per_pulse  #same as CW but decrementing tether length
            self.at_max_length = False

        #when max or min length reached (1cm margin), set variables
        if (self.tether_unspooled >= (self.MAX_TETHER_LENGTH - 0.01)): #set variable 1 pulse before max length, so max length is not surpassed
            print("~at maximum tether length~")
            self.at_max_length = True
        if (self.tether_unspooled <= 0.1):    #set variable 1 pulse before min length, so min length not surpassed
            print("~at minimum tether length~")
            self.at_min_length = True

        return 0


    #CALCULATIONS-------------------------------------------------------------------
    #function to calculate time delay per pulse in microseconds
    def delayPerPulse(self):

        #calculate rpm from m/s
        if (self.reel_velocity != 0):  #if velocity is nonzero
            reel_velocity_rpm = (abs(self.reel_velocity) / (math.pi * self.REEL_DIAMETER)) * 60 #calculate rpm from m/s
        else:   #handle case where velocity is 0
            reel_velocity_rpm = 0.0
        
        pulsePerMin = reel_velocity_rpm * self.PUL_PER_REV  #calculate pulse per minute: rpm times pulses per revolution
        if (pulsePerMin != 0):
            microsecPerPulse = 60000000 / pulsePerMin #calculate microsecond delay needed per pulse (microseconds per min / pulse per min
            pulseDelay = microsecPerPulse / 1000000   #convert microseconds into seconds (microseconds per pulse / microseconds per seconds)
            return (pulseDelay - .0009) #return pulse delay adjusted for time between loops (0.0009 seconds)
        else:
            pulseDelay = 0.0
            return pulseDelay
        
    #function to calculate/update the current altitude based on velocity data from drone
    def updateCalcAltitude(self):
        
        increment_time = 0.0009 + self.delayPerPulse()  #calculate the amount of time (seconds) since last altitude update (time since last pulse)
        
        altitude_change = self.current_velocity * increment_time    #calculate distance moved since last calc (distance = speed * time)
        self.calculated_alt += altitude_change    #increment calculated altitude variable by amount moved


    #CONTROLS-----------------------------------------------------------------------
    #function to pulse once (pulse delay matching velocity of reel) and track amount of tether unspooled
    def controlReelMotor(self):

        #define gpio pin outputs
        pul = gpiozero.LED(24)  #set pin 18 (gpio24) to PUL output
        dir = gpiozero.LED(23)  #set pin 16 (gpio23) to DIR output

        #get pulse delay
        pulse_delay = self.delayPerPulse()
        print("Pulse Delay: " + str(pulse_delay))

        #pulse (under the right conditions)
        if (self.at_min_length and (self.direction == 0)):    #if trying spool, but already fully spooled - no pulse
            print("Already fully spooled, no pulse")

        elif(self.at_max_length and (self.direction == 1)):   #if trying to unspool, but already fully unspooled - no pulse
            print("Already fully unspooled, no pulse")

        else:                                                 #otherwise, pulse motor
            #pulse motor if delay is greater than 0
            if (pulse_delay > 0.0):

                #send direction message to driver
                if (self.direction == 1): #unspool - pulse CCW
                    dir.on()
                else:                     #spool - pulse CW
                    dir.off()

                #send pulse message to driver 
                print("pulsing")
                pul.on() #pulse the signal
                time.sleep(pulse_delay) #delay pulse by number of seconds
                pul.off()
                return True
            else:
                print("holding")
                
        return False    #if did not pulse, return False


    #SETTERS------------------------------------------------------------------------
    #ROS Publisher - publish current reel (tether release) speed
    def publishTetherData(self):

        #Publish data to ROS topics
        published_reel_velocity = Float64()     #create variable with correct type to publish
        published_reel_velocity.data = self.reel_velocity   #put data into the variable that will be published
        
        published_tether_unspooled = Float64()      #create variable with correct type to publish
        published_tether_unspooled.data = self.tether_unspooled    #put data into the variable that will be published
        
        self.tether_unspool_speed_pub.publish(published_reel_velocity)  #actually publish the data to topics
        self.tether_unspooled_pub.publish(published_tether_unspooled)
        return 0


    #MAIN---------------------------------------------------------------------------
    #main() function - controls reel
    def mainReelControl(self):

        #exit function if drone is not armed
        if self.current_state.armed == False:
            return 0

        #output current loop number, drone mode, velocity, and altitude from drone as well as calculated current altitude
        print("Loop count: %s" % self.loop_count)
        print("Current drone mode: %s" % self.current_state.mode)
        print("Altitude (corrected): %s meters" % self.corrected_alt)
        print("Altitude (calculated): %s meters" % self.calculated_alt)
        print("UAS velocity: %s m/s" % self.current_velocity)
    
        #update queue of current velocities on each loop
        self.vel_queue.append(self.current_velocity)    #add current velocity to end of the queue
        self.vel_queue.pop(0)   #remove oldest velocity from queue
       
        #calculate desired reel velocity based on drone data
        self.reel_velocity = self.setReelVelocity()
        print("Reel velocity: %s m/s" % self.reel_velocity)

        #set direction of motor (spool or unspool)
        self.setDirection()
        
        #cause motor to turn at reel velocity
        pulsed = self.controlReelMotor() #pulse once (pulse delay matching velocity of reel)

        #update amount of tether unspooled after pulse, if pulse occured
        if (pulsed):
            self.updateTetherUnspooled()
        print("\t\t\t\t\tAmount of tether unspooled: %s meters\n" % self.tether_unspooled)

        #publish tether data and reel speed to ROS topics
        self.publishTetherData()

        #update calculated altitude
        self.updateCalcAltitude()
        
        #update loop count after every iteration
        self.loop_count += 1   
        
        #update last state variable at end of loop
        self.last_state = self.current_state
        
        print("")
        return 0
        
#-----------------------------------------------------------------------------------
def main():
    rclpy.init()    #initialize ROS node
    node = HangarControl()  #create node object
    rclpy.spin(node)    #spin node while script is running

if __name__ == "__main__":
    main()  