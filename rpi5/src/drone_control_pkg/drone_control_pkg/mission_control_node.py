import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from enum import Enum
import time, random, math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from pygeodesy.geoids import GeoidPGM


class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__("mision_planner")
        self.get_logger().info("mission planner started")
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target = Point()
        self.state = MissionState.WAIT_FOR_CONNECTIONM
        self.cmd_sent = False
        self.target_last_seen = None
        self.target_available = False
        self.search_start_time = None
        self.takeoff_height = 10.0
        self.home_altitude = None

        # GPS target Coordinates
        self.declare_parameter('target_lat', 0.0)
        self.declare_parameter('target_lon', 0.0)
        self.declare_parameter('target_altitude', 0.0)
        self.target_lat = self.get_parameter('target_lat').value
        self.target_lon = self.get_parameter('target_lon').value
        #will bw set at very first call back of the gps
        self.target_alt = None
        self.navigation_threshold = 0.5

        # current coordinates
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None

        # induce the error
        self.get_logger().info(f"TARGET RECEIVED -> lat:{self.target_lat}, lon:{self.target_lon}")
        self.target_lat, self.target_lon = self.induce_gps_error(self.target_lat, self.target_lon)
        self.get_logger().info(f"Modified:{self.target_lat}, lon:{self.target_lon}, alt: {self.target_alt}")

        # Image
        self.image_center_x = 320
        self.image_center_y = 240

        self.center_threshold = 20
        self.centered_start_time = None

        # PID Gains
        self.kp = 0.008
        self.ki = 0.0
        self.kd = 0.002

        # PID state
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        self.integral_x = 0.0
        self.integral_y = 0.0
        self.dt = 0.1

        # qos policy
        qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )

        # subscribers and publishers
        self.state_sub = self.create_subscription(State, "/mavros/state", self.state_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.pose_cb, qos)
        self.target_sub = self.create_subscription(Point, "/target_pixel", self.target_cb, 10)
        self.global_position_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        self.global_setpoint_pub = self.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', 10)


        # making few service clients
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.land_client = self.create_client(CommandTOL, "/mavros/cmd/land")
        
        # mission states
        self.is_takeoff_done = False
        self.is_tracking = False
        self.is_landing = False

        # creating timer
        self.timer = self.create_timer(0.05, self.control_loop)

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

        if self.home_altitude is None:
            self.home_altitude = msg.altitude
            self.target_alt = self.elipseoid_to_amsl(self.target_lat, self.target_lon, self.home_altitude + self.takeoff_height)
            self.get_logger().info(
                f"Home altitude set: {self.home_altitude:.2f} m"
            )

    def send_velocity(self, vx, vy, vz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.cmd_vel_pub.publish(msg)

    def control_loop(self):
        if self.state == MissionState.WAIT_FOR_CONNECTIONM:
            if self.current_state.connected:
                self.get_logger().info("FCU Connected")
                self.state = MissionState.SET_GUIDED_MODE

        elif self.state == MissionState.SET_GUIDED_MODE:
            if not self.mode_client.wait_for_service(timeout_sec = 1.0):
                self.get_logger().info("Waiting for mode service ... ")
                return
            if not self.cmd_sent:
                request = SetMode.Request()
                request.custom_mode = "GUIDED"
                future = self.mode_client.call_async(request)
                self.cmd_sent = True
            if self.current_state.mode == "GUIDED":
                self.get_logger().info("Mode set to guided")
                self.state = MissionState.ARM
                self.cmd_sent = False
            
        elif self.state == MissionState.ARM:
            if not self.arm_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for the arm service ... ")
                return
            if not self.cmd_sent:
                request = CommandBool.Request()
                request.value = True
                future = self.arm_client.call_async(request)
                self.cmd_sent = True
                self.get_logger().info("arm cmd sent ...")
            if self.current_state.armed:
                self.get_logger().info("Drone Armed")
                self.state = MissionState.TAKEOFF
                self.cmd_sent = False

        elif self.state == MissionState.TAKEOFF:
            if not self.takeoff_client.wait_for_service(timeout_sec = 1.0):
                self.get_logger().info("Waiting for takeoff service ...")
                return
            if not self.cmd_sent:
                request = CommandTOL.Request()
                request.altitude = self.takeoff_height
                future = self.takeoff_client.call_async(request)
                self.cmd_sent = True
                self.get_logger().info("takeogg cmd sent")
            if self.current_pose.pose.position.z >= self.takeoff_height - 0.2:
                self.get_logger().info("Takeoff completed")
                self.cmd_sent = False
                self.state = MissionState.NAVIGATE_TO_TARGET
                self.get_logger().info("state changed to navigate to target")
                # lock out the current orientation
                self.target_orientation = self.current_pose.pose.orientation

        elif self.state == MissionState.NAVIGATE_TO_TARGET:

            if not self.current_state.connected:
                self.get_logger().info("fcu is not connected")
                return
            if self.current_lat == None or self.current_lon == None:
                self.get_logger().info("lat or lon is none")
                return
            msg = GeoPoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.latitude = self.target_lat
            msg.pose.position.longitude = self.target_lon
            msg.pose.position.altitude = self.target_alt
            msg.pose.orientation = self.target_orientation

            self.global_setpoint_pub.publish(msg)
            self.get_logger().info(f"current altitude: {self.current_alt}")

            meters_per_lat = 111320
            meters_per_lon = (
                111320 * math.cos(math.radians(self.target_lat))
            )

            north = (
                self.target_lat - self.current_lat
            ) * meters_per_lat

            east = (
                self.target_lon - self.current_lon
            ) * meters_per_lon

            distance = math.sqrt(north**2 + east**2)


            if distance <= self.navigation_threshold:
                print("switching to search target, distance:",)
                self.state = MissionState.SEARCH_TARGET

        elif self.state == MissionState.SEARCH_TARGET:
            if self.target_available:
                self.get_logger().info("Target detected, switching to Align mode")
                self.state = MissionState.ALIGN_TARGET
            else:
                self.get_logger().info("Target not detected")
                self.send_velocity(0.0, 0.0, 0.0)

                if self.target_last_seen is None:
                    self.target_last_seen = time.time()
                if time.time() - self.target_last_seen > 5.0:
                    self.get_logger().info("Target lost for more than 5 sec")
                    if not self.takeoff_client.wait_for_service(timeout_sec = 1.0):
                        self.get_logger().info("Land service not available ")

                    if not self.cmd_sent:
                        request = SetMode.Request()
                        request.custom_mode = "LAND"
                        future = self.mode_client.call_async(request)
                        self.state = MissionState.FINISHED
                        self.cmd_sent = True

        elif self.state == MissionState.ALIGN_TARGET:
            if not self.target_available:
                self.get_logger().info("Target Lost ...")
                self.state = MissionState.SEARCH_TARGET
                return
            error_x = self.target.x - self.image_center_x
            error_y = self.target.y - self.image_center_y

            vy, self.integral_x = self.pid(error_x, self.prev_error_x, self.integral_x)
            vx, self.integral_y = self.pid(error_y, self.prev_error_y, self.integral_y)

            self.prev_error_x = error_x
            self.prev_error_y = error_y

            MAX_SPEED = 0.5

            vx = max(min(vx, MAX_SPEED), -MAX_SPEED)
            vy = max(min(vy, MAX_SPEED), -MAX_SPEED)

            self.get_logger().info(f"vx:{vx:.2f}, vy:{vy:.2f}")
            self.send_velocity(vy, -vx, 0.0)

            if (abs(error_x)< self.center_threshold and abs(error_y)< self.center_threshold):
                if self.centered_start_time is None:
                    self.centered_start_time = time.time()
                elif (time.time() - self.centered_start_time >= 1.0):
                    self.state = MissionState.DESCEND
                    self.get_logger().info("State changed to descend")
            else:
                self.centered_start_time = None
        
        elif self.state == MissionState.DESCEND:
            if not self.target_available:
                self.send_velocity(0.0, 0.0, 0.0)
                self.state = MissionState.SEARCH_TARGET
                return

            error_x = self.target.x - self.image_center_x
            error_y = self.target.y - self.image_center_y

            vy, self.integral_x = self.pid(error_x, self.prev_error_x, self.integral_x)
            vx, self.integral_y = self.pid(error_y, self.prev_error_y, self.integral_y)

            self.prev_error_x = error_x
            self.prev_error_y = error_y

            MAX_SPEED = 0.5

            vx = max(min(vx, MAX_SPEED), -MAX_SPEED)
            vy = max(min(vy, MAX_SPEED), -MAX_SPEED)

            if (abs(error_x ) > self.center_threshold and abs(error_y) > self.center_threshold):
                self.get_logger().info("Changing state to TARGET_ALIGN")
                self.state = MissionState.ALIGN_TARGET
                return

            vz = -0.20

            self.kp = 0.002
            self.send_velocity(vy, -vx, vz)

            if self.current_pose.pose.position.z < 1.0:
                self.state = MissionState.LAND
                self.get_logger().info("Mode changed to LAND")

        elif self.state == MissionState.LAND:
            if not self.takeoff_client.wait_for_service(timeout_sec = 1.0):
                self.get_logger().info("Land service not available ")

            if not self.cmd_sent:
                request = SetMode.Request()
                request.custom_mode = "LAND"
                future = self.mode_client.call_async(request)
                self.cmd_sent = True

            if self.current_pose.pose.position.z <= 0.5:
                self.state = MissionState.FINISHED
                self.cmd_sent = False
                self.get_logger().info("State changed to Finished")

        elif self.state == MissionState.FINISHED:
            self.get_logger().info("Mission Finished !!!")
        else :
            self.get_logger().info("Found invalid state ")
    
    def target_cb(self, msg):
        self.target = msg
        if msg.x == -1 and msg.y == -1:
            self.target_available = False
        else:
            self.target_available = True
            self.target_last_seen = time.time()
    def pid(self, error, prev_error, integral):
        integral += error*self.dt
        derivative = (error - prev_error) / self.dt
        output = (
            self.kp * error +
            self.ki * integral +
            self.kd * derivative
        )
        return output, integral
    def induce_gps_error(self, latitude, longitude, max_error_m=4.0):

        # Random error distance within the allowed radius
        error_distance = max_error_m * math.sqrt(random.random())

        # Random direction
        theta = random.uniform(0, 2 * math.pi)

        # Convert error into North/East displacement
        north_error = error_distance * math.cos(theta)
        east_error = error_distance * math.sin(theta)

        # Approximate metres per degree
        meters_per_degree_lat = 111_320

        meters_per_degree_lon = 111_320 * math.cos(
            math.radians(latitude)
        )

        # Convert metres to degrees
        latitude_error = north_error / meters_per_degree_lat
        longitude_error = east_error / meters_per_degree_lon

        noisy_latitude = latitude + latitude_error
        noisy_longitude = longitude + longitude_error

        return noisy_latitude, noisy_longitude
    @staticmethod
    def elipseoid_to_amsl(lat, lon, elipsoid_alt):
        geoid = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)
        N = geoid.height(lat, lon)
        return elipsoid_alt - N



class MissionState(Enum):
    WAIT_FOR_CONNECTIONM = 0
    SET_GUIDED_MODE = 1
    ARM = 2
    TAKEOFF = 3
    NAVIGATE_TO_TARGET = 4
    SEARCH_TARGET = 5
    ALIGN_TARGET = 6
    DESCEND = 7
    LAND = 8
    FINISHED = 9


def main(args = None):
    rclpy.init(args = args)

    node = MissionPlannerNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

