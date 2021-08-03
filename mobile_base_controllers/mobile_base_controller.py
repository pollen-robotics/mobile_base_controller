import time
from threading import Thread
from typing import Tuple

import rclpy
from rclpy.node import Node

from numpy import sqrt

from reachy_msgs.msg import MobileBaseDirection
from reachy_msgs.srv import SetMobileBaseMode

from .hb_controller import HBMotorConfig


class MobileBaseController(Node):
    def __init__(self) -> None:
        super().__init__('mobile_base_controller')
        self.logger = self.get_logger()

        self.mobile_base_controller = HBMotorConfig(channels=[0, 1])

        self.mobile_base_controller.mode_idle(0)
        self.mobile_base_controller.mode_idle(1)
        self.current_mode = 'idle'
        self.current_speed = (0, 0)

        self.set_mobile_base_mode_srv = self.create_service(
            srv_type=SetMobileBaseMode,
            srv_name='set_mobile_base_mode',
            callback=self.set_mobile_base_mode,
        )
        self.logger.info(f'Create "{self.set_mobile_base_mode_srv}" service.')

        self.goal_direction_subscription = self.create_subscription(
            msg_type=MobileBaseDirection,
            topic='mobile_base_direction_goals',
            callback=self.on_direction_goals,
            qos_profile=5,
        )
        self.logger.info(f'Subscribe to "{self.goal_direction_subscription.topic_name}".')

        

        self.logger.info('Node ready!')

    def set_mobile_base_mode(
        self,
        request: SetMobileBaseMode.Request,
        response: SetMobileBaseMode.Response,
    ) -> SetMobileBaseMode.Response:

        if request.mode == 'idle':
            self.mobile_base_controller.mode_idle(0)
            self.mobile_base_controller.mode_idle(1)
            self.current_mode = 'idle'

            self.logger.info('Switching to idle mode.')
            response.success = True

        elif request.mode == 'close_loop':
            self.mobile_base_controller.mode_close_loop_control(0)
            self.mobile_base_controller.mode_close_loop_control(1)
            self.current_mode = 'close_loop'

            self.logger.info('Switching to close loop control mode.')
            response.success = True

        else:
            self.logger.warning(f'Unkwnown mode {request.mode}!')
            response.success = False

        return response

    def on_direction_goals(self, msg: MobileBaseDirection) -> None:
        if self.current_mode == 'close_loop':
            l, r = self.speeds_from_direction(msg.x, msg.y)

            if (l, r) == self.current_speed == (0, 0):
                # Do not resend 0 speed to keep static position control
                return

            self.mobile_base_controller.move_input_vel(0, l)
            self.mobile_base_controller.move_input_vel(1, r)
            self.current_speed = (l, r)

    def speeds_from_direction(self, x: float, y: float) -> Tuple[float, float]:
        vmax = 0.75
        dead_zone = 0.01

        if sqrt(x ** 2 + y ** 2) <= dead_zone:
            return 0, 0

        else:
            y = y * vmax
            x = x * vmax * 0.5
            return -(x + y), -(-x + y)

    def stop(self):
        self.mobile_base_controller.move_input_vel(0, 0)
        self.mobile_base_controller.move_input_vel(1, 0)
        self.current_speed = (0, 0)

    def watchdog_safety_timer(self, check_period=0.1):
        while rclpy.ok():
            if self.current_speed != (0, 0) and (time.time() - self.last_pub) > self.wdt_duration:
                self.stop()

            time.sleep(check_period)


def main():
    rclpy.init()

    mobile_base_controller = MobileBaseController()
    rclpy.spin(mobile_base_controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
