from socket import IPV6_CHECKSUM
from serial import Serial
import serial
import time
from typing import Literal, List
import math
from math import pi

ser=Serial(
port='COM6',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)

UINT16_MAX = 65535

def get_uint16_for_radians(angle: float) -> int:
        return int(UINT16_MAX * (angle / (2 * pi)))

def convert_angle_to_bytes_list(angle: float) -> List[int]:
    print(f'Angle:  {angle} radians')
    return list(get_uint16_for_radians(angle).to_bytes(2, byteorder='big'))

# return radians
def convert_bytes_list_to_angle(bytes_list: List[int]) -> float:
    return (convert_bytes_list_to_integer(bytes_list) / UINT16_MAX) * 2 * pi

# return uint16
def convert_bytes_list_to_integer(bytes_list: List[int]) -> int:
    return int.from_bytes(bytes_list, byteorder='big')

if ser.isOpen():
     print(ser.name + ' is open...')

STX = b'\x02'
ETX = b'\x03'

ISOK = 0
ERR  = 1

def add_frame_borders(cmd: Literal):
    return STX + cmd + ETX

def print_incoming_bytes(count: int):
    bytes = ser.read(count)
    return bytes

def write_cmd_indicator(cmd_ind: Literal):
    cmd_indicator = add_frame_borders(cmd_ind)
    ser.write(cmd_indicator)
    print_incoming_bytes(3)

def write_cmd(cmd: Literal):
    cmd = add_frame_borders(cmd)
    ser.write(cmd)

def reset():
    write_cmd_indicator(b'\x03')
    write_cmd(b'\x00')
    print_incoming_bytes(3)

def get_calibration_state():
    write_cmd_indicator(b'\x03')
    write_cmd(b'\x01')
    bytes = print_incoming_bytes(2)
    
    if bytes[1] == ISOK:
        bytes += print_incoming_bytes(2)
        print(bytes[2])
    else:
        print_incoming_bytes(1)
        print('Error')
    
def get_movement_state():
    write_cmd_indicator(b'\x03')
    write_cmd(b'\x02')
    bytes = print_incoming_bytes(2)
    
    if bytes[1] == ISOK:
        bytes += print_incoming_bytes(2)
        print(bytes[2])
    else:
        print_incoming_bytes(1)
        print('Error')

def get_current_location():
    write_cmd_indicator(b'\x03')
    write_cmd(b'\x03')
    print('Collecting cmd response...')
    bytes = print_incoming_bytes(2)
    
    if bytes[1] == ISOK:
        bytes += print_incoming_bytes(3)
        print(str(convert_bytes_list_to_angle(bytes[2:4])) + ' rad')
    else:
        print_incoming_bytes(1)
        print('Error')

def go_to_absolute_position(angle: int):
    write_cmd_indicator(b'\x05')
    pos_bytes = convert_angle_to_bytes_list(angle)
    write_cmd(b'\x04'+ bytes(pos_bytes))
    print('Collecting cmd response...')
    resp_bytes = print_incoming_bytes(3)
    
    if resp_bytes[1] == ISOK:
        print('Set position successfully')
    else:
        print('Failed to set position')

ANTICLOCKWISE = 0x00
CLOCKWISE = 0x01

def go_to_relative_position(angle: int, direction: int):
    write_cmd_indicator(b'\x06')
    pos_bytes = convert_angle_to_bytes_list(angle)
    dir_byte = int.to_bytes(direction,1,byteorder='big', signed=False)
    write_cmd(b'\x05'+ bytes(pos_bytes) + dir_byte)
    print('Collecting cnmd response...')
    resp_bytes = print_incoming_bytes(3)
    
    if resp_bytes[1] == ISOK:
        print('Set position successfully')
    else:
        print('Failed to set position')

def rotate_clockwise(angle_step: int):
    while True:
        go_to_relative_position(angle_step, CLOCKWISE)
        time.sleep(0.5)

def rotate_anticlockwise(angle_step: int):
    while True:
        go_to_relative_position(angle_step, ANTICLOCKWISE)
        time.sleep(0.5)

# rotate_anticlockwise(0.1)
# rotate_clockwise(0.1)
# reset()
# get_calibration_state()
# get_movement_state()
# get_current_location()
# go_to_absolute_position(3)
#go_to_relative_position(0x2FFF, CLOCKWISE)