import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from te_uas_msgs.msg import Log
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import rclpy.qos
import rclpy.timer

class ArmDrone(Node):
    def __init__(self):
        super().__init__('drone_control')
        
        self.current_state = State()
        self.current_pose = NavSatFix()

        # URL for the logging service
        qos_profile = rclpy.qos.qos_profile_sensor_data

        # Define Publishers
        self.log_pub = self.create_publisher(Log, 'te_uas/logger', 10)

        # Define Subscribers
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, qos_profile)
        self.pose_sub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.pose_cb, qos_profile)

        
        self.create_rate(20)

        def takeoff_callback():
            if not self.current_state.armed:
                self.arm_drone()
            if self.current_state.mode != "AUTO.TAKEOFF":
                self.takeoff()
            if self.current_state.mode == "AUTO.LOITER":
                self.altitude_timer.cancel()
                self.altitude_timer.destroy()

        timer_period = 5  # Adjust as needed
        self.altitude_timer = self.create_timer(timer_period, takeoff_callback)

    # Update the current_state variable with the state callback
    def state_cb(self, state):
        self.current_state = state
    
    # Update the current_pose variable with the pose callback
    def pose_cb(self, pose):
        self.current_pose = pose

    def takeoff(self):
        cli = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.latitude = self.current_pose.latitude
        req.longitude = self.current_pose.longitude
        req.altitude = 2.5

        future = cli.call_async(req)
        future.add_done_callback(self.takeoff_finished)

    def takeoff_finished(self, future):
        try: 
            response = future.result()
            if response.success:
                self.custom_log("INFO", f"Drone took off successfully.")
                self.get_logger().info('Drone took off successfully')
                
            else:
                self.custom_log("WARNING", f"Failed to take off")
                self.get_logger().warning('Failed to take off')
        except Exception as e:
            self.get_logger().error('Failed to take off: {}'.format(e))
        

    def arm_drone(self):
        cli = self.create_client(CommandBool, 'mavros/cmd/arming')
        req = CommandBool.Request()
        req.value = True

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

    def custom_log(self, level, description):
        # Send log to logger node
        self.log_pub.publish(Log(level=level, description=description))

def main():
    rclpy.init()
    node = ArmDrone()
    rclpy.spin(node)
        

if __name__ == '__main__':
    main()
