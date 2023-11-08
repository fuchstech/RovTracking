    # -*- coding: utf-8 -*-
"""
Created on Sun Sep  5 04:10:40 2021

@author: KEREM
"""
from pymavlink import mavutil
from keyboard import is_pressed #pip3 install keyboard
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550') #companion connection

master.wait_heartbeat()


def arm():
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)


def disarm():
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

def set_rc_channel_pwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.

arm()
master.motors_armed_wait()
while  True:    
    # forward
    if is_pressed("w"):
        set_rc_channel_pwm(5, 1700)
    elif is_pressed("s"):
        set_rc_channel_pwm(5, 1300)
    elif is_pressed("d"):
        set_rc_channel_pwm(2, 1700) #roll
    elif is_pressed("a"):
    # Set some yaw
        set_rc_channel_pwm(2, 1300)
    elif is_pressed("k"):
        set_rc_channel_pwm(6, 1500)



