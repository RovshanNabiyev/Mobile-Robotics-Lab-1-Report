```markdown
# ==========================================
# Mobile Robotics Lab Report 1
# Author: Nabiyev Rovshan
# Institution: ADA UNIVERSITY
# Course: ENCE-4833 Mobile Robotics
# ==========================================

# ------------------------------------------
# Task 1: EV3 Robot Motion Control (50x50 cm Square Frame)
# ------------------------------------------
# Log A: Floor Geometry  -> Tracks physical position relative to 50 cm markers (measured 51.6–51.8 cm per side, ~3.4% systematic bias)
# Log B: Stopwatch       -> Measures run times vs kinematic predictions (~8% slower due to motor saturation under load)
# Log C: Marshal's Sheet -> Records heading errors at corners (-6° to -14° on early turns, ~9 cm cumulative drift)

# ------------------------------------------
# Task 2: PD Line Follower Controller
# ------------------------------------------
# Log A: Reflection Trace -> Reflected light sensor intensity sampled every 10 ms (Target: 30, Black: 6, White: 55)
# Log B: Floor Geometry   -> Maximum lateral deviation from line center measured with steel ruler (±0.5 mm)
# Log C: Stopwatch        -> Total lap completion time and duration spent in recovery/search states
# Log D: Marshal's Sheet  -> Observational log tracking line loss events and recovery success rates

# ------------------------------------------
# Task 3: Ultrasonic Distance Control (200 mm Target)
# ------------------------------------------
# Log A: Distance Trace  -> Continuous ultrasonic range measurements logged every 50 ms (Kp=2.0, Kd=0.3)
# Log B: Floor Geometry   -> Recorded settling positions, maximum overshoot (12 mm), and average error (~1.8 mm)
# Log C: Timer/Stopwatch  -> Approach time (2.63 s average) and loop execution timing
# Log D: Marshal's Sheet  -> Tracks dead-band entry (10 mm), target loss events (>2500 mm threshold), and stability

# ------------------------------------------
# Task 4: Autonomous Navigation & Obstacle Avoidance (Battle Arena)
# ------------------------------------------
# Log A: Event Log       -> Priority state triggers sampled every 20 ms (P1: Boundary Escape, P2: Obstacle Avoidance, P3: Wandering)
# Log B: Floor Geometry   -> Trajectory map, final position, and maximum excursion past boundary line
# Log C: Stopwatch        -> State breakdown tracking operational time (55% forward motion, 45% maneuvering)
# Log D: Marshal's Sheet  -> Independent count of boundary line crossings and near-contact events
# Log E: Data Locations   -> Spatial mapping of trigger events across the 3m battle arena
```
