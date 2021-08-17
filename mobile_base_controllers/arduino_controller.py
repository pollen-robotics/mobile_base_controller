from serial import Serial


class ArduinoMotorController:
    def __init__(self, port):
        self.serial = Serial(port, baudrate=115200)

    def mode_idle(self, axis):
        axis = 1 - axis
        self.send_command(f'w axis{axis}.requested_state 1')

    def mode_close_loop_control(self, axis):
        axis = 1 - axis
        self.send_command(f'w axis{axis}.requested_state 8')

    def move_input_vel(self, axis, vel):
        axis = 1 - axis
        self.send_command(f'v {axis} {vel:.2f} 0')

    def send_reset(self):
        self.send_command('rr')

    def send_command(self, command):
        self.serial.write(command.encode() + b'\n')
