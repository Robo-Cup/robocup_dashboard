#!env/usr/bin python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from dashboard_interfaces.srv import SetDashboardKeys, SetSubscribedValues
from dashboard_interfaces.msg import DashboardValues

from robocup_dashboard.dashboard.exceptions import DashboardException
from robocup_dashboard.dashboard.dashboard import Dashboard as DashboardGui

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class DashboardNode(Node):
    subscribed_value_indexes: list[int] = []
    def __init__(self):
        # Ros2 initialization
        super().__init__('dashboard_node')
        self.get_logger().info("Dashboard node has been started")

        # Ros2 set up services
        self.set_dashboard_keys_service = self.create_service(SetDashboardKeys, 'set_dashboard_keys', self.set_dashboard_keys_callback)
        self.set_subscribed_values_service = self.create_service(SetSubscribedValues, 'set_subscribed_values', self.subscribe_to_values_callback)
        self.dashboard_values_subscriber = self.create_subscription(DashboardValues, 'set_dashboard_values', self.dashboard_values_callback, 10)
        self.dashboard_values_publisher = self.create_publisher(DashboardValues, 'get_dashboard_values', 10)

        # Dashboard initialization
        self.dashboard_gui: DashboardGui = DashboardGui()

        # Start the pygame loop with ros2 timer
        self.refresh_rate = 60
        self.timer_ = self.create_timer(1.0 / self.refresh_rate, self.dashboard_run)

        self.subscribed_value_timer = self.create_timer(1.0, self.subscribed_value_timer_callback)

    def dashboard_values_callback(self, msg: DashboardValues):
        self.dashboard_gui.set_values(msg.key_indexes, msg.key_values)
        # self.get_logger().info(f"Dashboard key_indexes: {msg.key_indexes}, values: {msg.key_values}")

    def set_dashboard_keys_callback(self, request: SetDashboardKeys.Request, response):
        self.dashboard_gui.set_keys(request.keys)
        self.get_logger().info(f"Dashboard keys: {request.keys}")

        return response

    def subscribe_to_values_callback(self, request: SetSubscribedValues.Request, response):
        self.subscribed_value_indexes = request.key_indexes
        self.get_logger().info(f"Dashboard subscribed to values: {request.key_indexes}")

        return response

    def get_values(self, key_indexes: list) -> list[float]:
        ret_val = []
        for index in key_indexes:
            # check if index is out of bounds
            if index >= len(self.dashboard_gui.values):
                ret_val.append(0.0)
            else:
                ret_val.append(float(self.dashboard_gui.values[index]))
        return ret_val

    def dashboard_run(self):
        self.dashboard_gui.run()

    def subscribed_value_timer_callback(self):
        if len(self.subscribed_value_indexes) > 0:
            msg = DashboardValues()
            msg.key_indexes = self.subscribed_value_indexes
            msg.key_values = self.get_values(self.subscribed_value_indexes)
            self.dashboard_values_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    
    try:
        rclpy.spin(node)
        rclpy.shutdown()
        print(bcolors.FAIL + "\n[SHUTDOWN] Dashboard shutting down " + bcolors.BOLD + "normally" + bcolors.ENDC + bcolors.FAIL + "..." + bcolors.ENDC)
    except KeyboardInterrupt:
        print(bcolors.FAIL + "\n[SHUTDOWN] Dashboard shutting down at " + bcolors.OKBLUE + "terminal " + bcolors.FAIL + "request..." + bcolors.ENDC)
    except DashboardException:
        print(bcolors.FAIL + "\n[SHUTDOWN] Dashboard shutting down at " + bcolors.OKGREEN + "dashboard " + bcolors.FAIL + "request..." + bcolors.ENDC)