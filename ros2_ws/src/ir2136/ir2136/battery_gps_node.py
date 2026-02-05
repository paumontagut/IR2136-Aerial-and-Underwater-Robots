
import rclpy
import time
from rclpy.node import Node

#from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
#from std_msgs.msg import Tuple

from pymavlink import mavutil


class BatteryGpsNode(Node):

    def __init__(self):

        super().__init__('battery_gps_node')

        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        self.get_logger().info("Connected to SITL. :D")

        self.set_mode('GUIDED')
        time.sleep(5)
        self.arm_vehicle()
        time.sleep(5)
        self.takeoff(10)
        time.sleep(5)


        self.publisher_ = self.create_publisher(Int32, 'battery_status', 10)
        timer_period = 5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.last_target_gps = None
        self.subscription = self.create_subscription(Point,'target_gps',self.target_gps_callback,10)
        self.create_timer(5, self.send_target_gps)
    

    def target_gps_callback(self, msg):
        self.last_target_gps = msg
        self.get_logger().info(f"Got target_gps: {msg.x:.7f},{msg.y:.7f},{msg.z:.3f}")

    def send_target_gps(self):
        if self.last_target_gps is None:
            return
        lat_i = int(self.last_target_gps.x * 1e7)
        lon_i = int(self.last_target_gps.y * 1e7)
        alt   = float(self.last_target_gps.z)

        self.connection.mav.set_position_target_global_int_send(
        0,  # Timestamp
        self.connection.target_system,
        self.connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Relative altitude frame
        0b110111111000,  # Ignore velocity and acceleration
        int(lat_i),  # Latitude as integer
        int(lon_i),  # Longitude as integer
        alt,  # Desired altitude in meters
        0, 0, 0,  # Velocities
        0, 0, 0,  # Accelerations
        0, 0  # Yaw and Yaw rate
        )
        # self.send_count = getattr(self, "send_count", 0) + 1
        # if self.send_count % 5 == 0:
        #     self.get_logger().info("Sent SET_POSITION_TARGET_GLOBAL_INT")


    def timer_callback(self):
        msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=True, timeout=3)
        if msg:
            remaining = msg.battery_remaining  # Remaining battery percentage
            self.publisher_.publish(Int32(data=remaining))
            self.get_logger().warn(f"Battery remaining: {remaining}%")
        else:
            self.get_logger().warn("No BATTERY_STATUS received")
            return
    
    def arm_vehicle(self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1,  # Arm
            0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("Drone armed.")
    
    def set_mode(self, mode):
        self.connection.set_mode(mode)
        self.get_logger().info(f"Mode changed to {mode}.")

    def takeoff(self, altitude):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, altitude  # Desired altitude
        )
        self.get_logger().info(f"Taking off to {altitude} meters.")


def main(args=None):
    rclpy.init(args=args)
    publisher = BatteryGpsNode()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

