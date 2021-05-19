import rclpy
from rclpy.node import Node


class MobileBaseController(Node):
    def __init__(self) -> None:
        super().__init__('mobile_base_controller')


def main():
    rclpy.init()

    mobile_base_controller = MobileBaseController()
    rclpy.spin(mobile_base_controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
