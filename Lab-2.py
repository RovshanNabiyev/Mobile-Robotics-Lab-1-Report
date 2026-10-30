#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch

ev3 = EV3Brick()

# Motors and sensor
left = Motor(Port.B)
right = Motor(Port.C)
sensor = ColorSensor(Port.S1)

# Sensor calibration
BLACK = 6
WHITE = 55
TARGET = 30

# Normal driving
SPEED = 65
KP = 4.0
KD = 0.8

# Sharp left turn
LEFT_TRIGGER = -55
LEFT_REVERSE = -85
RIGHT_FORWARD = 105

# Line recovery
LOST_LEVEL = 43
FOUND_LEVEL = 28
LOST_LIMIT = 3
TURN_SPEED = 95

# Motor encoder limits
FIRST_ANGLE = 110
SECOND_ANGLE = 220
SEARCH_TIMEOUT = 2200

last_error = 0
last_turn = 1
lost_count = 0

clock = StopWatch()

def stop_motors():
    left.stop()
    right.stop()

def pivot(direction):
    left.run(int(direction * TURN_SPEED))
    right.run(int(-direction * TURN_SPEED))

def search(direction, angle_limit):
    start_angle = left.angle()
    clock.reset()
    dark_count = 0

    pivot(direction)

    while True:
        light = sensor.reflection()

        if light <= FOUND_LEVEL:
            dark_count += 1
        else:
            dark_count = 0

        if dark_count >= 2:
            stop_motors()
            return True

        if abs(left.angle() - start_angle) >= angle_limit:
            break

        if clock.time() >= SEARCH_TIMEOUT:
            break

        wait(10)

    stop_motors()
    return False

ev3.screen.clear()
ev3.screen.print("LINE FOLLOWER")
wait(800)

try:
    while True:

        light = sensor.reflection()

        # PD controller
        error = TARGET - light
        derivative = error - last_error
        correction = KP * error + KD * derivative

        # Remember turn direction
        if correction > 15:
            last_turn = 1
        elif correction < -15:
            last_turn = -1

        # Detect lost line
        if light >= LOST_LEVEL:
            lost_count += 1
        else:
            lost_count = 0

        if lost_count >= LOST_LIMIT:

            stop_motors()

            # Search last turn direction
            found = search(last_turn, FIRST_ANGLE)

            # Search opposite direction
            if not found:
                found = search(
                    -last_turn, SECOND_ANGLE
                )

            if not found:
                ev3.screen.clear()
                ev3.screen.print("LINE LOST")
                break

            lost_count = 0
            last_error = 0

        else:

            # Adaptive forward speed
            speed = SPEED - min(abs(error), 20)

            # Sharp left turn improvement
            if correction < LEFT_TRIGGER:
                left_speed = LEFT_REVERSE
                right_speed = RIGHT_FORWARD

            else:
                left_speed = int(speed + correction)
                right_speed = int(speed - correction)

            left.run(int(left_speed))
            right.run(int(right_speed))

            last_error = error

        wait(10)

except Exception as e:
    stop_motors()
    ev3.screen.clear()
    ev3.screen.print("ERROR")
    ev3.screen.print(str(e))
    wait(10000)

finally:
    stop_motors()