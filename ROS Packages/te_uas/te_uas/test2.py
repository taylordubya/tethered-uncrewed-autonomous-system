import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from te_uas_msgs.msg import Log
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import rclpy.qos
import rclpy.timer
import requests

class Takeoff(Node):
    def __init__(self):
        super().__init__('drone_control')
        
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()

        self.current_global_pose = NavSatFix()

        self.current_set_mode = ""

        qos_profile = rclpy.qos.qos_profile_sensor_data

        # Define Publishers
        self.log_pub = self.create_publisher(Log, 'te_uas/logger', 10)

        # Define Subscribers
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, qos_profile)
        self.pose_sub = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.global_pose_cb, qos_profile)

        self.create_service(CommandTOL, 'te_uas/adjust_alt_srvc', self.adjust_alt_cb)
        self.create_rate(20)


    def set_target_pose(self, x, y, z):
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z

    # Update the current_state variable with the state callback
    def state_cb(self, state):
        self.current_state = state
    
    # Update the current_pose variable with the pose callback
    def pose_cb(self, pose):
        self.current_pose = pose

    # Update the current_global_pose variable with the pose callback
    def global_pose_cb(self, pose):
        self.current_global_pose = pose

    def adjust_alt_cb(self, request, response):
        self.custom_log("INFO", f"Adjusting altitude to {round(request.altitude)}m")
        self.get_logger().info(f'Adjusting altitude to {request.altitude}')

        self.takeoff(request.altitude)
        response.success = True
        return response

        
    def takeoff(self, target_altitude):
        cli = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.latitude = self.current_global_pose.latitude
        req.longitude = self.current_global_pose.longitude
        req.altitude = target_altitude

        self.arm_drone()
        future = cli.call_async(req)
        future.add_done_callback(self.takeoff_finished)

    def takeoff_finished(self, future):
        try:
            response = future.result()
            if response.success:
                self.custom_log("INFO", f"Drone took off successfully.")
                self.get_logger().info('Drone took off successfully')
            else:
                self.custom_log("WARNING", f"Failed to take off drone")
                self.get_logger().warning('Failed to take off drone')
        except Exception as e:
            self.get_logger().error('Failed to take off drone: {}'.format(e))



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
        # Send log to logger node
        self.log_pub.publish(Log(level=level, description=description))

def main():
    rclpy.init()
    node = Takeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()