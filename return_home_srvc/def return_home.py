import rclpy  # ROS 2 client library
from rclpy.node import Node  # Import the Node class
from sensor_msgs.msg import Image  # Import the Image message type
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL  # Import MAVROS services
from mavros_msgs.msg import PositionTarget, Altitude, State  # Import MAVROS message types
from std_srvs.srv import Trigger, TriggerResponse  # Import standard service types
from cv_bridge import CvBridge  # Import CvBridge to convert between ROS and OpenCV images
import cv2  # OpenCV library for image processing
import apriltag  # AprilTag library for marker detection

# Define a class called ReturnHomeNode that inherits from Node
class ReturnHomeNode(Node):
    def __init__(self):
        # Initialize the Node with the name 'return_home'
        super().__init__('return_home')
        
        # Create a CvBridge instance to convert ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Create an AprilTag detector instance
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        
        # Initialize the drone's altitude to 0
        self.altitude = 0.0

        # Create a publisher to send position corrections to the drone
        self.pos_pub = self.create_publisher(PositionTarget, 'mavros/setpoint_raw/local', 10)
        
        # Subscribe to the altitude topic to get the drone's altitude
        self.create_subscription(Altitude, '/mavros/altitude', self.altitude_callback, 10)

        # Create service clients to change the drone's mode, arm/disarm it, and land it
        self.set_mode_srv = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.land_srv = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.hanger_spool_srv = self.create_client(Trigger, 'hanger_spool_srvc/srv')

        # Create a service called 'return_home_srvc' that triggers the tag_detection method
        self.create_service(Trigger, 'return_home_srvc', self.tag_detection)

        # Log that we are waiting for the Pixhawk to connect
        self.get_logger().info("Waiting for Pixhawk to connect...")
        self.wait_for_pixhawk_connection()

    def wait_for_pixhawk_connection(self):
        # Loop until the Pixhawk is connected
        while rclpy.ok():
            try:
                # Wait for a message from the Pixhawk state topic
                state = self.wait_for_message('/mavros/state', State, timeout=5)
                if state.connected:
                    self.get_logger().info("Pixhawk connected.")
                    break
            except rclpy.exceptions.ROSInterruptException:
                self.get_logger().info("Waiting for Pixhawk connection...")

    def altitude_callback(self, msg):
        # Update the altitude when a new message is received
        self.altitude = msg.relative

    def call_hanger_spool_srvc(self):
        # Call the hanger spool service
        self.get_logger().info('Calling hanger spool service')
        try:
            if not self.hanger_spool_srv.wait_for_service(timeout_sec=10.0):
                self.get_logger().error('Hanger spool service not available')
                return False
            future = self.hanger_spool_srv.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None and future.result().success:
                self.get_logger().info("Hanger spool service called successfully")
                return True
            else:
                self.get_logger().error("Hanger spool service call failed")
                return False
        except Exception as e:
            self.get_logger().error(f"Hanger spool service call failed: {e}")
            return False

    def call_adjust_alt_srvc(self):
        # Call the adjust altitude service
        self.get_logger().info('Calling adjust altitude service')
        try:
            adjust_alt_srvc = self.create_client(Trigger, 'adjust_alt_srvc/srv')
            if not adjust_alt_srvc.wait_for_service(timeout_sec=10.0):
                self.get_logger().error('Adjust altitude service not available')
                return False
            future = adjust_alt_srvc.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None and future.result().success:
                self.get_logger().info("Adjust altitude service called successfully")
                return self.call_hanger_spool_srvc()  # Call hanger spool service and wait for confirmation
            else:
                self.get_logger().error("Adjust altitude service call failed")
                return False
        except Exception as e:
            self.get_logger().error(f"Adjust altitude service call failed: {e}")
            return False

    def calculate_position_corrections(self, tag_center, image_center):
        # Calculate the error between the tag center and the image center
        error_x = tag_center[0] - image_center[0]
        error_y = tag_center[1] - image_center[1]
        return error_x, error_y

    def send_position_correction(self, x, y, z):
        # Send position corrections to the drone
        pos_target = PositionTarget()
        pos_target.coordinate_frame = PositionTarget.FRAME_BODY_NED
        pos_target.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
                               PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                               PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        pos_target.position.x = x
        pos_target.position.y = y
        pos_target.position.z = z
        self.pos_pub.publish(pos_target)
        # This function creates a PositionTarget message and fills in the necessary fields to tell the drone how to adjust its position. 
        # The coordinate frame is set to FRAME_BODY_NED (North-East-Down), and a type mask is used to ignore certain parameters. 
        # The position corrections are then published to the 'mavros/setpoint_raw/local' topic.

    def set_mode(self, mode):
        # Set the drone's mode
        self.get_logger().info(f"Setting mode to {mode}")
        try:
            if not self.set_mode_srv.wait_for_service(timeout_sec=10.0):
                self.get_logger().error('Service not available')
                return
            future = self.set_mode_srv.call_async(SetMode.Request(custom_mode=mode))
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f"Mode set to {mode}")
            else:
                self.get_logger().error(f"Failed to set mode to {mode}")
        except Exception as e:
            self.get_logger().error(f"Failed to set mode to {mode}: {e}")
        # This function sets the drone's flight mode to a specified mode (e.g., OFFBOARD). 
        # It waits for the '/mavros/set_mode' service to be available, then sends a request to change the mode. 
        # If successful, it logs the new mode; if not, it logs an error.

    def arm_drone(self, arm):
        # Arm or disarm the drone
        self.get_logger().info(f"{'Arming' if arm else 'Disarming'} drone")
        try:
            if not self.arm_srv.wait_for_service(timeout_sec=10.0):
                self.get_logger().error('Service not available')
                return
            future = self.arm_srv.call_async(CommandBool.Request(value=arm))
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f"Drone {'armed' if arm else 'disarmed'}")
            else:
                self.get_logger().error(f"Failed to {'arm' if arm else 'disarm'} drone")
        except Exception as e:
            self.get_logger().error(f"Failed to {'arm' if arm else 'disarm'} drone: {e}")
        # This function arms or disarms the drone. 
        # It waits for the '/mavros/cmd/arming' service to be available, then sends a request to arm or disarm the drone. 
        # If successful, it logs the action; if not, it logs an error.

    def command_descent(self, target_altitude):
        # Command the drone to descend to a specific altitude
        self.get_logger().info(f"Descending to {target_altitude} meters OFFBOARD mode")
        rate = self.create_rate(10)  # 10 Hz
        while self.altitude > target_altitude + 0.5:
            self.send_position_correction(0, 0, target_altitude)
            rate.sleep()
        self.get_logger().info("Target destination reached, switching to vision-based landing.")
        # This function commands the drone to descend to a specified altitude. 
        # It continuously sends position corrections to maintain the target altitude until the drone is within 0.5 meters of the target. 
        # Once the target altitude is reached, it logs the event and prepares for vision-based landing.

    def tag_detection(self, request, response):
        # Main function to handle tag detection and landing
        self.get_logger().info("Tag detection started")
        # Call the adjust altitude service before switching modes
        if not self.call_adjust_alt_srvc():
            response.success = False
            response.message = "Adjust altitude service call failed"
            return response

        # Set the drone to OFFBOARD mode
        self.set_mode("OFFBOARD")
        # Arm the drone
        self.arm_drone(True)
        # Command the drone to descend to 5 meters
        self.command_descent(5)
        scaling_factor = 1 / 192.00

        # Wait for the drone to reach 5 meters altitude
        self.get_logger().info("Waiting for the drone to reach 5 meters altitude...")
        while self.altitude > 5:
            rclpy.sleep(1)

        # Switch to vision-based landing
        self.get_logger().info("Drone has reached 5 meters altitude. Switching to vision-based landing.")
        cap = cv2.VideoCapture(0)
        image_center = (960, 540)  # Assuming 1920x1080 resolution

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().error("Failed to capture image.")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = self.detector.detect(gray)

            if results:
                for r in results:
                    tag_center = (r.center[0], r.center[1])
                    error_x, error_y = self.calculate_position_corrections(tag_center, image_center)

                    position_x = error_x * scaling_factor
                    position_y = error_y * scaling_factor

                    self.send_position_correction(position_x, position_y, 5)

                    if abs(error_x) < 10 and abs(error_y) < 10:
                        try:
                            if not self.land_srv.wait_for_service(timeout_sec=10.0):
                                self.get_logger().error('Service not available')
                                return TriggerResponse(success=False, message="Landing service not available")
                            future = self.land_srv.call_async(CommandTOL.Request(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0))
                            rclpy.spin_until_future_complete(self, future)
                            if future.result() is not None:
                                self.get_logger().info("Initiating landing sequence...")
                                cap.release()
                                return TriggerResponse(success=True, message="Landing initiated")
                            else:
                                self.get_logger().error("Landing service call failed")
                                return TriggerResponse(success=False, message="Landing service call failed")
                        except Exception as e:
                            self.get_logger().error(f"Landing service failed: {e}")
                            return TriggerResponse(success=False, message=f"Landing service failed: {e}")

        cap.release()
        return TriggerResponse(success=False, message="AprilTag not found")
        # This function is the main logic for handling the tag detection and landing sequence. 
        # It starts by calling the adjust altitude service, and if successful, sets the drone to OFFBOARD mode, arms it, and commands it to descend to 5 meters.
        # It then switches to vision-based landing, where it captures images, detects AprilTags, and calculates position corrections. 
        # If the tag is detected and the positional error is small enough, it initiates the landing sequence.

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    # Create an instance of the ReturnHomeNode
    node = ReturnHomeNode()
    # Keep the node running until it is interrupted
    rclpy.spin(node)
    # Shut down the ROS 2 Python client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
