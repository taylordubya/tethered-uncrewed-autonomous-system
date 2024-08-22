import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from te_uas_msgs.msg import Log
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy.qos
import rclpy.timer

class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')
        
        self.current_state = State()

        self.current_global_pose = Odometry()
        self.current_pose = PoseStamped()

        self.target_pose = PoseStamped()
        self.offset = PoseStamped()

        qos_profile = rclpy.qos.qos_profile_sensor_data

        # Define Publishers
        self.local_pos_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        self.log_pub = self.create_publisher(Log, 'te_uas/logger', 10)
        self.offset_pub = self.create_publisher(PoseStamped, 'te_uas/offset', 10)

        # Define Subscribers
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pose_cb, qos_profile)
        self.global_pose_sub = self.create_subscription(Odometry, 'mavros/global_position/local', self.global_pose_cb, qos_profile)

        # Define Services
        self.create_service(CommandTOL, 'te_uas/adjust_alt_srvc', self.adjust_alt_cb)
        
        self.create_rate(20)
      
    # Telemetry Callbacks
    def set_target_pose(self, x, y, z):
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z 

    def state_cb(self, state):
        self.current_state = state
    
    def pose_cb(self, pose):
        self.current_pose = pose
        
    def global_pose_cb(self, pose):
        self.current_global_pose = pose
        
    def adjust_alt_cb(self, request, response):
        self.get_logger().info(f'Adjusting altitude to {request.altitude}')
        self.achieve_altitude(request.altitude)
        
        response.success = True
        return response
    
    def achieve_altitude(self, target_altitude):
        # Create an offset between the global to local position and the local position
        # self.offset.pose.position.x = self.current_pose.pose.position.x - self.current_global_pose.pose.pose.position.x
        # self.offset.pose.position.y = self.current_pose.pose.position.y - self.current_global_pose.pose.pose.position.y
        self.offset.pose.position.z = self.current_pose.pose.position.z - self.current_global_pose.pose.pose.position.z

        self.get_logger().info(f'Current altitude is: {self.current_pose.pose.position.z}m')
        self.get_logger().info(f'Global altitude is: {self.current_global_pose.pose.pose.position.z}m')
        self.get_logger().info(f'Offset is: {self.offset.pose.position.z}m')
        self.set_target_pose(self.current_pose.pose.position.x,
                         self.current_pose.pose.position.y,
                         target_altitude + self.offset.pose.position.z)
        
        # Attempt to rearm and set mode to OFFBOARD up to 3 times
        arm_attempts = 0
        def pre_flight_callback():
            nonlocal arm_attempts
            # Arm drone then set mode to OFFBOARD
            if not self.current_state.armed:
                self.arm_drone()
            if self.current_state.mode != "OFFBOARD":
                self.set_mode("OFFBOARD")
            if self.current_state.armed and self.current_state.mode == "OFFBOARD" and arm_attempts >= 2:
                self.pre_flight_timer.cancel()
            arm_attempts += 1

        retry_period = 2 # Seconds
        self.pre_flight_timer = self.create_timer(retry_period, pre_flight_callback)
        
        # Publish the desired altitude on a timer until the desired altitude is reached
        def altitude_adjustment_callback(): 
            # Publish the target altitude   
            self.local_pos_pub.publish(self.target_pose)
            # If the current altitude is within a .3m margin of the target altitude, stop the timer
            if abs(self.current_pose.pose.position.z - (target_altitude + self.offset.pose.position.z)) <= 0.5:
                self.get_logger().info('Altitude achieved')
                self.custom_log("INFO", f"Altitude achieved at {round(self.current_global_pose.pose.pose.position.z, 3)}m")
                self.set_mode("AUTO.LOITER")
                self.altitude_timer.cancel()
                self.pre_flight_timer.cancel()

        timer_period = 0.025  # Adjust as needed
        self.altitude_timer = self.create_timer(timer_period, altitude_adjustment_callback)
    # This function controls the altitude of the drone
    # It gets the target altitude, then (if needed) arms and changes mode to OFFBOARD
    # Then publishes the target altitude on a timer to the '/mavros/setpoint_position/local' topic

    # This function arms the drone
    def arm_drone(self):
        # Create a service client to arm the drone
        cli = self.create_client(CommandBool, 'mavros/cmd/arming')

        # Create a request to send to the service
        req = CommandBool.Request()
        req.value = True

        # Send the request
        future = cli.call_async(req)
        future.add_done_callback(self.arm_drone_finished)
    # This function logs the drone arming status
    def arm_drone_finished(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Drone armed successfully')
                self.custom_log("INFO", f"Drone armed successfully.")
            else:
                self.get_logger().warning('Failed to arm drone: {}'.format(response.message))
                self.custom_log("WARNING", f"Failed to arm drone")
        except Exception as e:
            self.get_logger().error('Failed to arm drone: {}'.format(e))

    def set_mode(self, mode):
        # Create a service client to set the mode
        cli = self.create_client(SetMode, 'mavros/set_mode')

        # Create a request to send to the service
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode

        # Update current set mode for logging
        self.current_set_mode = mode

        # Send the request
        future = cli.call_async(req)
        future.add_done_callback(self.set_mode_finished)
    # This function logs the drone mode change
    def set_mode_finished(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                # Log the mode change
                self.custom_log("INFO", f"Changed mode to {self.current_set_mode}")
                self.get_logger().info(f'Changed mode to {self.current_set_mode}')
            else:
                self.custom_log("WARNING", f"Failed to set mode: {response.mode_sent}")
                self.get_logger().warning('Failed to set mode: {}'.format(response.mode_sent))
        except Exception as e:
            self.get_logger().error('Failed to set mode: {}'.format(e))

    # This function handles sending logs to the server and through ROS
    def custom_log(self, level, description):
        # Send log to logger node
        self.log_pub.publish(Log(level=level, description=description))

        


def main():
    rclpy.init()
    node = DroneControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()