"""
These configurations are based off of the hoverboard tutorial found on the 
Odrive Robotics website located here:
https://docs.odriverobotics.com/hoverboard
@author: Austin Owens
@date: 3/14/2021
"""

import sys
import time
import math
import odrive
from odrive.enums import *


class HBMotorConfig:
    """
    Class for configuring an Odrive axis for a Hoverboard motor. 
    Only works with one Odrive at a time.
    """
    
    # Hoverboard Kv
    HOVERBOARD_KV = 16.0
        
    # Min/Max phase inductance of motor
    MIN_PHASE_INDUCTANCE = 0
    MAX_PHASE_INDUCTANCE = 0.001
    
    # Min/Max phase resistance of motor
    MIN_PHASE_RESISTANCE = 0
    MAX_PHASE_RESISTANCE = 0.5
    
    # Tolerance for encoder offset float
    ENCODER_OFFSET_FLOAT_TOLERANCE = 0.05

    def __init__(self, channels):
        """
        Initalizes HBMotorConfig class by finding odrive, erase its 
        configuration, and grabbing specified axis object.
        """
        self.channels = channels
            
        # Connect to Odrive
        print("Looking for ODrive...")
        self._find_odrive()
        print("Found ODrive.")
        
    def _find_odrive(self):
        # connect to Odrive
        self.odrv = odrive.find_any()

        self.odrv_axes = []
        for c in reversed(self.channels):
            odrv_axis = getattr(self.odrv, "axis{}".format(c))
            self.odrv_axes.append(odrv_axis)
    
    def configure(self):
        """
        Configures the odrive device for a hoverboard motor.
        """
        
        # Erase pre-exsisting configuration
        #print("Erasing pre-exsisting configuration...")
        #try:
        #    self.odrv.erase_configuration()
        #except ChannelBrokenException:
        #    pass
        
        self._find_odrive()
        
        for odrv_axis in self.odrv_axes:
            # Standard 6.5 inch hoverboard hub motors have 30 permanent magnet 
            # poles, and thus 15 pole pairs
            odrv_axis.motor.config.pole_pairs = 15

            # Hoverboard hub motors are quite high resistance compared to the hobby 
            # aircraft motors, so we want to use a bit higher voltage for the motor 
            # calibration, and set up the current sense gain to be more sensitive. 
            # The motors are also fairly high inductance, so we need to reduce the 
            # bandwidth of the current controller from the default to keep it 
            # stable.
            odrv_axis.motor.config.resistance_calib_max_voltage = 4
            odrv_axis.motor.config.requested_current_range      = 25
            odrv_axis.motor.config.current_control_bandwidth    = 100

            # Estimated KV but should be measured using the "drill test", which can
            # be found here:
            # https://discourse.odriverobotics.com/t/project-hoverarm/441
            odrv_axis.motor.config.torque_constant = 8.27 / self.HOVERBOARD_KV

            # Hoverboard motors contain hall effect sensors instead of incremental 
            # encorders
            odrv_axis.encoder.config.mode = ENCODER_MODE_HALL

            # The hall feedback has 6 states for every pole pair in the motor. Since
            # we have 15 pole pairs, we set the cpr to 15*6 = 90.
            odrv_axis.encoder.config.cpr = 90

            # Since hall sensors are low resolution feedback, we also bump up the 
            #offset calibration displacement to get better calibration accuracy.
            odrv_axis.encoder.config.calib_scan_distance = 150

            # Since the hall feedback only has 90 counts per revolution, we want to 
            # reduce the velocity tracking bandwidth to get smoother velocity 
            # estimates. We can also set these fairly modest gains that will be a
            # bit sloppy but shouldn’t shake your rig apart if it’s built poorly. 
            # Make sure to tune the gains up when you have everything else working 
            # to a stiffness that is applicable to your application.
            odrv_axis.encoder.config.bandwidth = 100
            odrv_axis.controller.config.pos_gain = 20
            odrv_axis.controller.config.vel_gain = 0.05 * odrv_axis.motor.config.torque_constant * odrv_axis.encoder.config.cpr
            odrv_axis.controller.config.vel_integrator_gain = 0.1 * odrv_axis.motor.config.torque_constant * odrv_axis.encoder.config.cpr
            odrv_axis.controller.config.vel_limit = 20

            # Set in position control mode so we can control the position of the 
            # wheel
            odrv_axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            odrv_axis.controller.config.vel_ramp_rate = 2
            odrv_axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP

        # In the next step we are going to start powering the motor and so we 
        # want to make sure that some of the above settings that require a 
        # reboot are applied first.
        print("Saving manual configuration and rebooting...")
        self.odrv.save_configuration()
        print("Manual configuration saved.")
        #try:
        #    self.odrv.reboot()
        #except ChannelBrokenException:
        #    pass
            
        #self._find_odrive()

        for odrv_axis in self.odrv_axes:
            input("Make sure the motor is free to move, then press enter...")
            
            print("Calibrating Odrive for hoverboard motor (you should hear a "
            "beep)...")
            
            odrv_axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
            
            # Wait for calibration to take place
            time.sleep(10)

            if odrv_axis.motor.error != 0:
                print("Error: Odrive reported an error of {} while in the state " 
                "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
                "debug:\n{}".format(odrv_axis.motor.error, 
                                    odrv_axis.motor))
                
                sys.exit(1)

            if odrv_axis.motor.config.phase_inductance <= self.MIN_PHASE_INDUCTANCE or \
            odrv_axis.motor.config.phase_inductance >= self.MAX_PHASE_INDUCTANCE:
                print("Error: After odrive motor calibration, the phase inductance "
                "is at {}, which is outside of the expected range. Either widen the "
                "boundaries of MIN_PHASE_INDUCTANCE and MAX_PHASE_INDUCTANCE (which "
                "is between {} and {} respectively) or debug/fix your setup. Printing "
                "out Odrive motor data for debug:\n{}".format(odrv_axis.motor.config.phase_inductance, 
                                                            self.MIN_PHASE_INDUCTANCE,
                                                            self.MAX_PHASE_INDUCTANCE, 
                                                            odrv_axis.motor))
                
                sys.exit(1)

            if odrv_axis.motor.config.phase_resistance <= self.MIN_PHASE_RESISTANCE or \
            odrv_axis.motor.config.phase_resistance >= self.MAX_PHASE_RESISTANCE:
                print("Error: After odrive motor calibration, the phase resistance "
                "is at {}, which is outside of the expected range. Either raise the "
                "MAX_PHASE_RESISTANCE (which is between {} and {} respectively) or "
                "debug/fix your setup. Printing out Odrive motor data for " 
                "debug:\n{}".format(odrv_axis.motor.config.phase_resistance, 
                                    self.MIN_PHASE_RESISTANCE,
                                    self.MAX_PHASE_RESISTANCE, 
                                    odrv_axis.motor))
                
                sys.exit(1)

            # If all looks good, then lets tell ODrive that saving this calibration 
            # to persistent memory is OK
            odrv_axis.motor.config.pre_calibrated = True

            # Check the alignment between the motor and the hall sensor. Because of 
            # this step you are allowed to plug the motor phases in random order and
            # also the hall signals can be random. Just don’t change it after 
            # calibration.
            print("Calibrating Odrive for encoder...")
            odrv_axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            
            # Wait for calibration to take place
            time.sleep(30)
                
            if odrv_axis.encoder.error != 0:
                print("Error: Odrive reported an error of {} while in the state "
                "AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder "
                "data for debug:\n{}".format(odrv_axis.encoder.error, 
                                            odrv_axis.encoder))
                
                sys.exit(1)
            
            # If offset_float isn't 0.5 within some tolerance, or its not 1.5 within
            # some tolerance, raise an error
            if not ((odrv_axis.encoder.config.offset_float > 0.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
            odrv_axis.encoder.config.offset_float < 0.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE) or \
            (odrv_axis.encoder.config.offset_float > 1.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
            odrv_axis.encoder.config.offset_float < 1.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE)):
                print("Error: After odrive encoder calibration, the 'offset_float' "
                "is at {}, which is outside of the expected range. 'offset_float' "
                "should be close to 0.5 or 1.5 with a tolerance of {}. Either "
                "increase the tolerance or debug/fix your setup. Printing out "
                "Odrive encoder data for debug:\n{}".format(odrv_axis.encoder.config.offset_float, 
                                                            self.ENCODER_OFFSET_FLOAT_TOLERANCE, 
                                                            odrv_axis.encoder))
                        
                sys.exit(1)
            
            # If all looks good, then lets tell ODrive that saving this calibration 
            # to persistent memory is OK
            odrv_axis.encoder.config.pre_calibrated = True
            
            print("Saving calibration configuration and rebooting...")
            self.odrv.save_configuration()
            print("Calibration configuration saved.")
            #try:
            #    self.odrv.reboot()
            #except ChannelBrokenException:
            #    pass
            #    
            #self._find_odrive()
            
            print("Odrive configuration finished.")
    
    def mode_idle(self, c):
        """
        Puts the motor in idle (i.e. can move freely).
        """
        self.odrv_axes[c].requested_state = AXIS_STATE_IDLE
    
    def mode_close_loop_control(self, c):
        """
        Puts the motor in closed loop control.
        """
        self.odrv_axes[c].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        
    def move_input_pos(self, c, angle):
        """
        Puts the motor at a certain angle.
        
        :param angle: Angle you want the motor to move.
        :type angle: int or float
        """
        #odrv_axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.odrv_axes[c].controller.input_pos = angle/360.0
        
    def move_input_vel(self, c, vel):
        """
        Puts the motor at a certain angle.
        
        :param angle: Angle you want the motor to move.
        :type angle: int or float
        """
        #odrv_axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.odrv_axes[c].controller.input_vel = vel


