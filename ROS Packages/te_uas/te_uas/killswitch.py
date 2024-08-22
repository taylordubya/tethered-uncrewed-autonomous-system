import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from mavros_msgs.srv import CommandLong

class KillSwitch(Node):
    def __init__(self):
        super().__init__('killswitch')

        self.create_service(Empty, 'te_uas/killswitch', self.kill_cb)


    def kill_cb(self, request, response):
        cli = self.create_client(CommandLong, 'mavros/cmd/command')
        req = CommandLong.Request()
        req.command = 400
        req.param1 = 0.0
        req.param2 = 21196.0

        future = cli.call_async(req)
        future.add_done_callback(self.kill_finished)
    
    def kill_finished(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Drone killed successfully')
            else:
                self.get_logger().warning('Failed to kill drone: {}'.format(response.message))
        except Exception as e:
            self.get_logger().error('Failed to kill drone: {}'.format(e))

def main():
    rclpy.init()
    node = KillSwitch()
    rclpy.spin(node)

if __name__ == '__main__':
    main()