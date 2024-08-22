import os
import json
import rclpy
import rclpy.qos
import rclpy.timer
from rclpy.node import Node

from te_uas_msgs.msg import Log
from datetime import datetime
import subprocess

class Logger(Node):
    def __init__(self):
        super().__init__('logger')

        # Start the servers
        subprocess.Popen('node ./src/te_uas/servers/database-server.js', shell=True)
        subprocess.Popen('node ./src/te_uas/servers/auth-server.js', shell=True)

        self.db = {}
        self.log_file = os.environ.get('TE_UAS_LOG_PATH', './src/te_uas/logs/logs-database.json')
        if os.path.exists(self.log_file):
            with open(self.log_file, 'r') as f:
                self.db = json.load(f)
        else:
            self.db = {'flights': []}

        # Log Server URL
        qos_profile = rclpy.qos.qos_profile_sensor_data

        # Define Subscribers
        self.logger_sub = self.create_subscription(Log, 'te_uas/logger', self.log_cb, qos_profile)

        self.create_rate(20)

    def save_db(self):
        with open(self.log_file, 'w') as f:
            json.dump(self.db, f)
        self.get_logger().info(f"Database saved to {self.log_file}")

    def generate_unique_id(self, existing_ids):
        import random
        id = None
        while id is None or id in existing_ids:
            id = random.randint(0, 99999)
        return id
    
    # Callback function to log messages
    def log_cb(self, log):
        # Read the database
        if os.path.exists(self.log_file):
            with open(self.log_file, 'r') as f:
                self.db = json.load(f)
        # Log the message to the terminal
        self.get_logger().info(f"Logging message: {log.description}")

        # Append the log to the active flight
        active_flight = next((flight for flight in self.db['flights'] if flight.get('active', False)), None)
        if active_flight:
            log_id = self.generate_unique_id([log['logId'] for log in active_flight.get('logs', [])])
            new_log = {
                'logId': log_id,
                'level': log.level,
                'description': log.description,
                'timestamp': datetime.today().strftime('%m/%d/%Y, %I:%M:%S %p'),
                'dismissed': []
            }
            active_flight.setdefault('logs', []).append(new_log)
            self.get_logger().info(f"Appended log: {new_log}")
            self.save_db()
        else:
            self.get_logger().warning("No active flight found to log the message")

def main():
    rclpy.init()
    logger = Logger()
    rclpy.spin(logger)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
