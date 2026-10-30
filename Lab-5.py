
#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch

ev3 = EV3Brick()

# -----------------------------
# EV3 SETUP
# -----------------------------

left = Motor(Port.B)
right = Motor(Port.C)
sensor = ColorSensor(Port.S1)

BLACK = 6
WHITE = 55
TARGET = 30

SPEED = 65
KP = 4.0
KD = 0.8

LEFT_TRIGGER = -55
LEFT_REVERSE = -85
RIGHT_FORWARD = 105

LOST_LEVEL = 43
FOUND_LEVEL = 28
LOST_LIMIT = 3
TURN_SPEED = 95

FIRST_ANGLE = 110
SECOND_ANGLE = 220
SEARCH_TIMEOUT = 2200

# Distance and timing
WHEEL_DIAMETER = 56
AXLE_TRACK = 120
NODE_DISTANCE = 300

last_error = 0
last_turn = 1
lost_count = 0

clock = StopWatch()
run_clock = StopWatch()

# -----------------------------
# MAZE GRAPH
# -----------------------------

# Replace with actual measured maze.
# Values represent edge distances in mm.

graph = {
    "S": {"A": 300},
    "A": {"S": 300, "B": 400, "C": 350},
    "B": {"A": 400, "G1": 300},
    "C": {"A": 350, "G1": 450},
    "G1": {"B": 300, "C": 450}
}

# Record explored nodes and edges
explored_nodes = []
explored_edges = []

# -----------------------------
# MOTOR FUNCTIONS
# -----------------------------

def stop_motors():
    left.stop()
    right.stop()


def pivot(direction):
    left.run(int(direction * TURN_SPEED))
    right.run(int(-direction * TURN_SPEED))


def drive_forward(speed):
    left.run(speed)
    right.run(speed)


def reset_distance():
    left.reset_angle(0)
    right.reset_angle(0)


def get_distance():
    l = abs(left.angle())
    r = abs(right.angle())

    average = (l + r) / 2

    return (average / 360) * 3.14159 * WHEEL_DIAMETER


# -----------------------------
# LINE RECOVERY
# -----------------------------

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


# -----------------------------
# LINE FOLLOWING
# -----------------------------

def follow_line():
    global last_error
    global last_turn
    global lost_count

    light = sensor.reflection()

    error = TARGET - light
    derivative = error - last_error

    correction = KP * error + KD * derivative

    if correction > 15:
        last_turn = 1
    elif correction < -15:
        last_turn = -1

    if light >= LOST_LEVEL:
        lost_count += 1
    else:
        lost_count = 0

    if lost_count >= LOST_LIMIT:

        stop_motors()

        found = search(last_turn, FIRST_ANGLE)

        if not found:
            found = search(-last_turn, SECOND_ANGLE)

        if not found:
            return False

        lost_count = 0
        last_error = 0

    else:

        speed = SPEED - min(abs(error), 20)

        if correction < LEFT_TRIGGER:
            left_speed = LEFT_REVERSE
            right_speed = RIGHT_FORWARD
        else:
            left_speed = int(speed + correction)
            right_speed = int(speed - correction)

        left.run(left_speed)
        right.run(right_speed)

        last_error = error

    return True


# -----------------------------
# EXPLORE AND LOG
# -----------------------------

def record_node(node):
    if node not in explored_nodes:
        explored_nodes.append(node)
        ev3.screen.clear()
        ev3.screen.print("Node:")
        ev3.screen.print(node)


def record_edge(start, end, distance):
    explored_edges.append(
        (start, end, distance)
    )

    if start not in graph:
        graph[start] = {}

    if end not in graph:
        graph[end] = {}

    graph[start][end] = distance
    graph[end][start] = distance


def explore():
    ev3.screen.clear()
    ev3.screen.print("EXPLORE MODE")

    current_node = "S"
    node_number = 0

    record_node(current_node)
    reset_distance()

    while node_number < 10:

        if not follow_line():
            ev3.screen.clear()
            ev3.screen.print("LINE LOST")
            break

        distance = get_distance()

        # Example junction detection:
        # Calibrate this condition for your marker patches.
        if sensor.reflection() >= WHITE - 5:

            stop_motors()
            wait(300)

            node_number += 1

            next_node = "N" + str(node_number)

            record_node(next_node)

            record_edge(
                current_node,
                next_node,
                int(distance)
            )

            current_node = next_node

            reset_distance()

            wait(500)

    stop_motors()

    ev3.screen.clear()
    ev3.screen.print("Explore complete")


# -----------------------------
# SHORTEST PATH (DIJKSTRA)
# -----------------------------

def shortest_path(start, goal):

    distances = {node: float("inf") for node in graph}
    previous = {}

    distances[start] = 0
    unvisited = list(graph.keys())

    while unvisited:

        current = min(
            unvisited,
            key=lambda node: distances[node]
        )

        if distances[current] == float("inf"):
            break

        unvisited.remove(current)

        if current == goal:
            break

        for neighbour, weight in graph[current].items():

            new_distance = (
                distances[current] + weight
            )

            if new_distance < distances[neighbour]:

                distances[neighbour] = new_distance
                previous[neighbour] = current

    if goal not in previous and start != goal:
        return None

    path = []
    current = goal

    while current != start:
        path.append(current)
        current = previous[current]

    path.append(start)
    path.reverse()

    return path


# -----------------------------
# PLAN
# -----------------------------

def plan():

    ev3.screen.clear()
    ev3.screen.print("PLANNING")

    route = shortest_path("S", "G1")

    if route is None:
        ev3.screen.print("No route found")
        return None

    ev3.screen.clear()
    ev3.screen.print("Shortest route:")

    for node in route:
        ev3.screen.print(node)

    return route


# -----------------------------
# RUN
# -----------------------------

def run_route(route):

    if not route:
        return

    ev3.screen.clear()
    ev3.screen.print("RUN MODE")

    run_clock.reset()

    # Each edge is traversed using the line follower.
    for i in range(len(route) - 1):

        start = route[i]
        end = route[i + 1]

        distance = graph[start][end]

        reset_distance()

        while get_distance() < distance:

            if not follow_line():
                stop_motors()
                ev3.screen.print("Route error")
                return

            wait(10)

        stop_motors()
        wait(300)

    stop_motors()

    total_time = run_clock.time() / 1000

    ev3.screen.clear()
    ev3.screen.print("FINISHED!")
    ev3.screen.print("Time:")
    ev3.screen.print(total_time)
    ev3.speaker.beep()


# -----------------------------
# MAIN PROGRAM
# -----------------------------

try:

    ev3.screen.clear()
    ev3.screen.print("TASK 5")
    ev3.screen.print("EXPLORE PLAN RUN")

    wait(1500)

    # Stage 1
    explore()

    # Stage 2
    route = plan()

    # Stage 3
    if route:
        run_route(route)

except Exception as e:

    stop_motors()

    ev3.screen.clear()
    ev3.screen.print("ERROR")
    ev3.screen.print(str(e))

    wait(5000)

finally:
    stop_motors()
