#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from random import randint

# EV3 robot setup

ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Sensors for obstacle avoidance and edge detection
ultrasonic = UltrasonicSensor(Port.S1)
color = ColorSensor(Port.S3)

robot = DriveBase(
    left_motor,
    right_motor,
    wheel_diameter=55.5,
    axle_track=104,
)

# Drive settings
SPEED = 150
SAFE_DISTANCE = 250  # mm
BLACK_THRESHOLD = 25
GAME_TIME = 120000

robot.settings(
    straight_speed=SPEED,
    straight_acceleration=300,
    turn_rate=120,
    turn_acceleration=200,
)

# Random wandering timer

timer = StopWatch()
last_wander = 0
next_wander = randint(3000, 6000)


def reset_wander_timer():
    global last_wander, next_wander
    last_wander = timer.time()
    next_wander = randint(3000, 6000)


while timer.time() < GAME_TIME:
    distance = ultrasonic.distance()
    light = color.reflection()

    # Treat invalid ultrasonic readings as "no obstacle"
    if distance >= 2500:
        distance = 2500

    # Priority 1: Escape boundary
    if light < BLACK_THRESHOLD:
        robot.stop()
        robot.straight(-150)

        angle = randint(110, 170)
        if randint(0, 1) == 0:
            angle = -angle

        robot.turn(angle)
        reset_wander_timer()

    # Priority 2: Avoid other robots
    elif distance < SAFE_DISTANCE:
        robot.stop()
        robot.straight(-100)

        angle = randint(90, 140)
        if randint(0, 1) == 0:
            angle = -angle

        robot.turn(angle)
        last_wander = timer.time()

    # Priority 3: Random wandering
    elif timer.time() - last_wander > next_wander:
        robot.turn(randint(-90, 90))
        reset_wander_timer()

    else:
        robot.drive(SPEED, 0)

    wait(20)

robot.stop()
ev3.speaker.beep()