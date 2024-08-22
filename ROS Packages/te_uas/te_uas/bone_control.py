import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from te_uas_msgs.msg import Log, DoorStatus
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
import rclpy.qos
import rclpy.timer

class BoneControl(Node):
    def __init__(self):
        super().__init__('bone_control')
        
        self.current_state = State()
        self.current_pose = NavSatFix()
        self.target_pose = GeoPoseStamped()
        self.door_status = DoorStatus()

        self.current_set_mode = ""

        qos_profile = rclpy.qos.qos_profile_sensor_data

        # Define Publishers
        self.local_pos_pub = self.create_publisher(GeoPoseStamped, 'mavros/setpoint_position/global', 10)
        self.log_pub = self.create_publisher(Log, 'te_uas/logger', 10)
        self.hangar_pub = self.create_publisher(DoorStatus, 'hangar/status', 10)

        # Define Subscribers
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, qos_profile)
        self.pose_sub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.pose_cb, qos_profile)

        # Define Services
        self.create_service(CommandTOL, 'te_uas/adjust_alt_srvc', self.adjust_alt_cb)
        
        # rate = self.create_rate(1)
        self.create_rate(20)
        self.publish_door_status()

    def set_target_pose(self, latitude, longitude, altitude):
        self.target_pose.pose.position.latitude = latitude
        self.target_pose.pose.position.longitude = longitude
        self.target_pose.pose.position.altitude = altitude

    # Update the current_state variable with the state callback
    def state_cb(self, state):
        self.current_state = state
    
    # Update the current_pose variable with the pose callback
    def pose_cb(self, pose):
        self.current_pose = pose

    # Service callback to adjust the altitude
    def adjust_alt_cb(self, request, response):
        self.custom_log("INFO", f"Adjusting altitude to {round(request.altitude)}m")
        self.get_logger().info(f'Adjusting altitude to {request.altitude}')

        self.achieve_altitude(request.altitude)
        response.success = True
        return response
    # Function to adjust the altitude of the drone
    def achieve_altitude(self, target_altitude):
        self.set_target_pose(self.current_pose.latitude,
                         self.current_pose.longitude,
                         target_altitude)
        # Arm drone then set mode to OFFBOARD
        if not self.current_state.armed:
            self.arm_drone()
        if self.current_state.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")

        self.number = 0
        def altitude_adjustment_callback(): 
            # Publish the target pose  
            self.local_pos_pub.publish(self.target_pose)

            # When the drone is within 0.3 meters of the target altitude, stop publishing and set mode to AUTO.LOITER
            if abs(self.current_pose.altitude - target_altitude) <= 0.3:
                self.number += 1
                print(self.number)
                self.get_logger().info('Altitude achieved')
                self.custom_log("INFO", f"Altitude achieved at {round(self.current_pose.altitude , 3)}m")
                self.set_mode("AUTO.LOITER")
                self.altitude_timer.cancel()
                self.altitude_timer.cancel()

        timer_period = 0.05  # Adjust as needed
        self.altitude_timer = self.create_timer(timer_period, altitude_adjustment_callback)
    # This function controls the altitude of the drone
    # It gets the target altitude, then (if needed) arms and changes mode to OFFBOARD
    # Then publishes the target altitude on a timer to the '/mavros/setpoint_position/local' topic



    def arm_drone(self):
        cli = self.create_client(CommandBool, 'mavros/cmd/arming')
        req = CommandBool.Request()
        req.value = True
        self.custom_log("INFO", f"Initial altitude is: {self.current_pose.altitude}")

        future = cli.call_async(req)
        future.add_done_callback(self.arm_drone_finished)

    def arm_drone_finished(self, future):
        try:
            response = future.result()
            if response.success:
                self.custom_log("INFO", f"Drone armed successfully.")
                self.get_logger().info('Drone armed successfully')
            else:
                self.custom_log("WARNING", f"Failed to arm drone")
                self.get_logger().warning('Failed to arm drone')

        except Exception as e:
            self.get_logger().error('Failed to arm drone: {}'.format(e))

    def set_mode(self, mode):
        cli = self.create_client(SetMode, 'mavros/set_mode')
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode

        # 
        self.current_set_mode = mode
        
        future = cli.call_async(req)
        future.add_done_callback(self.set_mode_finished)

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

    def custom_log(self, level, description):
        self.log_pub.publish(Log(level=level, description=description))

    #publisher function to publish door status
    def publish_door_status(self):
        print("T1")
        def door_status_callback():
            print("T2")
            self.door_status.opened = False
            self.door_status.left_opened = False
            self.door_status.right_opened = False
            self.door_status.left_closed = True
            self.door_status.right_closed = True
            self.hangar_pub.publish(self.door_status)
        timer_period = 0.5  # Adjust as needed
        self.altitude_timer = self.create_timer(timer_period, door_status_callback)

def main():
    rclpy.init()
    node = BoneControl()
    # rclpy.spin(node)

    while rclpy.ok():
        rclpy.spin_once(node)

if __name__ == '__main__':
    main()
