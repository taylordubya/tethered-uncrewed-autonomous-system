#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

# Global variable to store the publisher
vertical_velocity_pub = None

def main():
    global vertical_velocity_pub
    
    # Initializing the ROS node
    rospy.init_node('vertical_velocity_publisher', anonymous=True)
    
    # Setting up the publisher for the vertical velocity
    vertical_velocity_pub = rospy.Publisher('/drone/vertical_velocity', Float32, queue_size=10)
    
    # Subscribe to the MAVROS IMU data topic
    rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
    
    # Keep the node running
    rospy.spin()
  
# Function to extract data and publish it 
def imu_callback(data):
    vertical_velocity = vehicle_local_position.vz # vehicle_local_position.vz is the vertical velocity 
    vertical_velocity_pub.publish(vertical_velocity) # Publish the data 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
  
