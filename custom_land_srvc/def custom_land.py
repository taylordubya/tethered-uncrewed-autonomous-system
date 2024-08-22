#!/usr/bin/env python

import rospy 
from std_srvs.srv import Empty
from mavros_msgs.srv import CommandTOL
from drone_control.srv import CustomLand, CustomLandResponse 

# Function to call the hanger spool service in the hanger 
def call_hanger_spool_service():
  rospy.wait_for_service('hanger_spool_srvc/srv')
  try:   # Proxy for calling the ROS service 
     hanger_spool_service = rospy.ServiceProxy('hanger_spool_srvc/srv', Empty)
        hanger_spool_service()
        rospy.loginfo("Hanger spool service called successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
      # Lack of a return command due to the service being of type 'Empty'

# Function to land the drone using the built-in MAVROS service 
def land_drone(): 
  rospy.wait_for_service('/mavros/cmd/land')
  try:   # Proxy for calling the ROS service 
    land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        response = land_service(min_pitch=0, yaw=0, latitude=0, longitude=0, altitude=0)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False
# Handles requests to the custom land service by calling the other two functions in sequence 
def handle_custom_land(req): 
  rospy.loginfo("Custom land service called.") 
  call_hanger_spool_service()
  success = land_drone()
  return CustomLandResponse(success=success)

# Sets up the custom land service and keeps the ROS node running to handle requests 
def custom_land_service():
  rospy.init_node('custom_land_service_node')
  rospy.Service('custom_land_srvc', CustomLand, handle_custom_land)
  rospy.loginfo("Custom land service is ready.")
  rospy.spin()
# Does not need a return command 

if __name__ = "__main__":
  try: 
    custom_land_service()
  except rospy.ROSInterruptException: 
    pass
