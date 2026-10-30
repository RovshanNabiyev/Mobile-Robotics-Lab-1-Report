#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

ev3 = EV3Brick()

# Motors and ultrasonic sensor
left = Motor(Port.B)
right = Motor(Port.C)
sensor = UltrasonicSensor(Port.S1)

# Robot dimensions (Robot Educator example)
robot = DriveBase(
    left,
    right,
    wheel_diameter=55.5,
    axle_track=104
)

# Target distance in mm
TARGET = 200

# PD controller parameters
KP = 2.0
KD = 0.3

# Safety and timing
MAX_SPEED = 150
DEAD_BAND = 10
H = 0.05
ALPHA = 0.3

previous_error = 0
filtered_derivative = 0

ev3.screen.clear()
ev3.screen.print("LAB 3 PD")
wait(1000)

try:
    while True:

        # 1. Read distance
        distance = sensor.distance()

        # 2. Invalid measurement protection
        if distance > 2500:
            robot.stop()
            previous_error = 0
            filtered_derivative = 0

            ev3.screen.clear()
            ev3.screen.print("NO TARGET")
            wait(50)
            continue

        # 3. Calculate error
        error = distance - TARGET

        # 4. Calculate derivative
        raw_derivative = (
            error - previous_error
        ) / H

        filtered_derivative = (
            ALPHA * raw_derivative
            + (1 - ALPHA) * filtered_derivative
        )

        # 5. PD control
        speed = (
            KP * error
            + KD * filtered_derivative
        )

        # 6. Dead band near 200 mm
        if abs(error) <= DEAD_BAND:
            speed = 0

        # 7. Speed limit
        speed = max(
            -MAX_SPEED,
            min(MAX_SPEED, speed)
        )

        # 8. Move forward or backward
        robot.drive(speed, 0)

        # 9. Display on EV3
        ev3.screen.clear()
        ev3.screen.print("LAB 3 - PD")
        ev3.screen.print(str(distance) + " mm")

        if speed > 0:
            ev3.screen.print("FORWARD")
        elif speed < 0:
            ev3.screen.print("BACKWARD")
        else:
            ev3.screen.print("STOP")

        previous_error = error

        # 10. Control loop: 50 ms
        wait(50)

finally:
    robot.stop()