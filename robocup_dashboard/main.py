#!env/usr/bin python3
import rclpy
from rclpy.node import Node

from dashboard_interfaces.srv import SetDashboardKeys
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
    def __init__(self):
        # Ros2 initialization
        super().__init__('dashboard_node')
        self.get_logger().info("Dashboard node has been started")

        # Ros2 set up services
        self.set_dashboard_keys_service = self.create_service(SetDashboardKeys, 'set_dashboard_keys', self.set_dashboard_keys_callback)
        self.dashboard_values_subscriber = self.create_subscription(DashboardValues, 'dashboard_values', self.dashboard_values_callback, 10)

        # Dashboard initialization
        self.dashboard_gui: DashboardGui = DashboardGui()

        # Start the pygame loop with ros2 timer
        self.refresh_rate = 60
        self.timer_ = self.create_timer(1.0 / self.refresh_rate, self.dashboard_run)

    def dashboard_values_callback(self, msg: DashboardValues):
        self.dashboard_gui.set_values(msg.key_indexes, msg.key_values)
        # self.get_logger().info(f"Dashboard key_indexes: {msg.key_indexes}, values: {msg.key_values}")

    def set_dashboard_keys_callback(self, request: SetDashboardKeys.Request, response):
        self.dashboard_gui.set_keys(request.keys)
        self.get_logger().info(f"Dashboard keys: {request.keys}")

        return response

    def dashboard_run(self):
        self.dashboard_gui.run()


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