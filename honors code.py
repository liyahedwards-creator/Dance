#!/usr/bin/env python3

"""
GoPiGo3 for the Raspberry Pi: an open source robotics platform for the Raspberry Pi.
Copyright (C) 2017  Dexter Industries

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.
"""

from easygopigo3 import EasyGoPiGo3
from gopigo3 import FirmwareVersionError
import sys
import signal
from time import sleep

DEBUG = False # if set to True, any exception that's encountered is debugged
MAX_DISTANCE = 2300 # measured in mm
MIN_DISTANCE = 150 # measured in mm
NO_OBSTACLE = 3000
ERROR = 0 # the error that's returned when the DistanceSensor is not found
MAX_SPEED = 300 # max speed of the GoPiGo3
MIN_SPEED = 100 # min speed of the GoPiGo3

SERVO_RIGHT = 0
SERVO_FORWARD = 90
SERVO_WAIT = 0.3

WALL_DISTANCE = 200 # stay from the right wall
WALL_TOLERANCE = 40 # how much error is okay before steering
WALL_LOST = 500 # if the right reading is bigger than this, the wall is gone
FRONT_MIN = 200 # if something is closer than this in front, turn left
CORNER_ROOM = 350

FORWARD_SPEED = 150
STEER_AMOUNT = 40


# variable for triggering the closing procedure of the script
# used for stopping the while loop that's in the Main() function
robot_operating = True

# handles the CTRL-C signal sent from the keyboard
# required for gracefull exits of the script
def signal_handler(signal, frame):
    global robot_operating
    print("CTRL-C combination pressed")
    robot_operating = False

# function for debugging
def debug(string):
    if DEBUG is True:
        print(f"Debug: {string}")
        
def look(head_servo, distance_sensor, angle):
    head_servo.rotate_servo(angle)
    sleep(SERVO_WAIT)
    return distance_sensor.read_mm()

def Main():
    # initializing an EasyGoPiGo3 object and a DistanceSensor object
    # used for interfacing with the GoPiGo3 and with the distance sensor
    try:
        gopigo3 = EasyGoPiGo3()
        distance_sensor = gopigo3.init_distance_sensor()
        head_servo = gopigo3.init_servo("SERVO1")
        
        

    except IOError as msg:
        print("GoPiGo3 robot not detected or DistanceSensor not installed.")
        debug(msg)
        sys.exit(1)

    except FirmwareVersionError as msg:
        print("GoPiGo3 firmware needs to updated.")
        debug(msg)
        sys.exit(1)

    except Exception as msg:
        print("Error occurred. Set debug = True to see more.")
        debug(msg)
        sys.exit(1)

    if DEBUG is True:
        distance_sensor.enableDebug()

    # variable that says whether the GoPiGo3 moves or is stationary
    # used during the runtime
    gopigo3_stationary = True

    global robot_operating

    # while the script is running
    loop_count = 0
    while robot_operating:
        loop_count = loop_count + 1
        right_distance = look(head_servo, distance_sensor, SERVO_RIGHT)
 
       
        if right_distance == ERROR:
            print("Bad reading from DistanceSensor, trying again.")
            continue
 
        if right_distance > WALL_LOST:
            print("Wall lost, turning right")
            gopigo3.stop()
            gopigo3.set_speed(FORWARD_SPEED)
            front_distance = look(head_servo, distance_sensor, SERVO_FORWARD)
            if front_distance == ERROR or front_distance > CORNER_ROOM:
                gopigo3.drive_cm(20)
 
            gopigo3.turn_degrees(90)
            gopigo3.drive_cm(15)
            continue
 
        # RULE 2: if the right is closed, check the front every 3rd loop
        if loop_count % 3 == 0:
            front_distance = look(head_servo, distance_sensor, SERVO_FORWARD)
 
            # if something is close in front of the robot, turn left
            if front_distance != ERROR and front_distance < FRONT_MIN:
                print(f"Obstacle ahead at {front_distance} mm, turning left")
                gopigo3.stop()
                gopigo3.turn_degrees(-90)
                continue
 
        # if the robot is too far from the wall, steer toward it (right)
        if right_distance > WALL_DISTANCE + WALL_TOLERANCE:
            gopigo3.set_motor_dps(gopigo3.MOTOR_LEFT, FORWARD_SPEED + STEER_AMOUNT)
            gopigo3.set_motor_dps(gopigo3.MOTOR_RIGHT, FORWARD_SPEED - STEER_AMOUNT)
 
        # if the robot is too close to the wall, steer away from it (left)
        elif right_distance < WALL_DISTANCE - WALL_TOLERANCE:
            gopigo3.set_motor_dps(gopigo3.MOTOR_LEFT, FORWARD_SPEED - STEER_AMOUNT)
            gopigo3.set_motor_dps(gopigo3.MOTOR_RIGHT, FORWARD_SPEED + STEER_AMOUNT)
        else:
            gopigo3.set_speed(FORWARD_SPEED)
            gopigo3.forward()
 
    gopigo3.stop()
    head_servo.rotate_servo(SERVO_FORWARD)
 
 
if __name__ == "__main__":
    # signal handler
    # handles the CTRL-C combination of keys
    signal.signal(signal.SIGINT, signal_handler)
    Main()
 
