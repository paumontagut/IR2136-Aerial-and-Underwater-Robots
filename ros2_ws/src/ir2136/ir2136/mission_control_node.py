
import rclpy
import sys
from rclpy.node import Node
import argparse
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Int32
from geometry_msgs.msg import Point

from pymavlink import mavutil


class MissionControlNode(Node):

    def __init__(self, lat_deg, lon_deg, alt_m):

        super().__init__('mission_control_node')

        self.lat_deg = lat_deg
        self.lon_deg = lon_deg
        self.alt_m = alt_m

        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        self.get_logger().info("Connected to SITL. :D")

        # Publisher de target_gps
        self.msg = Point()
        self.msg.x = self.lat_deg
        self.msg.y = self.lon_deg
        self.msg.z = self.alt_m
        self.publisher_ = self.create_publisher(Point, 'target_gps', 10)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscriber de battery_status
        self.subscription = self.create_subscription(Int32, 'battery_status', self.battery_status_callback, 10)
        self.landing = False

    def timer_callback(self):
        if self.landing:
            return
        self.publisher_.publish(self.msg)

    def battery_status_callback(self, msg):
        self.get_logger().info(f"Battery status: {msg.data}")
        if self.landing:
            return
        if int(msg.data) < 90:
            self.get_logger().warn("Battery low <90%, landing...")
            self.send_landing_command()
            self.landing = True


    def send_landing_command(self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().warn("Landing command sent")

def main(args=None):
    rclpy.init(args=args)

    # Lógica para publicar un punto GPS
    parser = argparse.ArgumentParser(description='Mission Control Node')
    parser.add_argument('lat_deg', type=float, help='Latitude in degrees')
    parser.add_argument('lon_deg', type=float, help='Longitude in degrees')
    parser.add_argument('alt_m', type=float, help='Altitude in meters')
    argv = remove_ros_args(sys.argv)  
    ns = parser.parse_args(argv[1:])
    node = MissionControlNode(ns.lat_deg, ns.lon_deg, ns.alt_m)


    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

