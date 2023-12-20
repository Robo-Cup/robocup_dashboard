#!env/usr/bin python3

class DashboardException(Exception):
    def __init__(self, message=""):
        self.message = message
        super().__init__(self.message)