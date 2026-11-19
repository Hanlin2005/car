# All tunable numbers in one place.
# The microcontroller pins are in sketch/pins.h.

# --- What to run ---
# Modes are picked and started on the web page (http://10.194.91.71:8080).
# This only chooses which one is selected when the program starts.
# "web" (manual driving) also starts running right away; the others wait
# for the Run button.
# "web" | "drive" | "heading_align" | "straight_test" | "duty_test" | "motor_test" | "lidar_test"
MODE = "web"

# --- Robot geometry ---
WHEEL_BASE_M = 0.163         # effective distance between the driven wheels. Two turn-test runs
                             # 2026-11-19 (six 1-turn spins each, camera residual): medians 16.6
                             # and 16.1 cm, scatter ~1 cm. In-place spins depend on the caster.
METERS_PER_TICK = 0.00098    # wheel circumference / encoder ticks per revolution.
                             # Calibrated 2026-11-19 from two tape-measured straight runs.
                             # Speeds before that date were really 1.9x what the numbers said.

# --- Speeds (m/s) ---
# The page's speed slider sets these while a run is going.
# SLOW_SPEED stays half of CRUISE_SPEED.
CRUISE_SPEED = 0.30
SLOW_SPEED = 0.15

# --- Lane following (controller.py: encoder pose + path pure pursuit) ---
# The web page can change these two while a run is going.
VISION_CONTROL = True       # off: drive straight, ignore the lane lines
VISION_GAIN = 1.0           # scales pure-pursuit curvature. Page Strength can raise this
# How far ahead on the lane-center path to aim (meters).
PURE_PURSUIT_LOOKAHEAD_M = 0.45  # was 0.60; shorter reacts faster to the caster yawing the car
# Kept for reference / old notes. Pure pursuit no longer uses these P gains.
KP_OFFSET = 5.0
KP_HEADING = 1.5
LANE_FILTER = 0.2            # unused by pose tracker; camera blend is ODOM_CAMERA_BLEND
MAX_OMEGA = 0.8              # rad/s, hardest turn allowed. Was 1.2: at 0.2 m/s that stalled the
                             # inner wheel (below ~0.08 m/s it sticks) and the car pivoted on it.
MAX_OMEGA_RATE = 6.0         # rad/s per second, how fast the turn rate can change
TURN_SLOWDOWN = 0.40         # at the hardest turn, drive this fraction slower (was 0.70; keeps the inner wheel rolling)
PURE_PURSUIT_DEADZONE_M = 0.02  # ignore cross-track smaller than this (reduces fidgeting)
# Heading-rate damping: steer against the car's heading CHANGING, not just
# its error. The caster sticks and then lets go, swinging the nose 30 deg in
# half a second; this counters the swing as it starts. 0 = off.
HEADING_DAMPING = 0.7        # rad/s of turn per rad/s of heading rate (0.5 helped; 0.7 next step)
HEADING_RATE_FILTER = 0.4    # 0-1, how much each new heading-rate sample counts
# How hard each good camera frame pulls the pose (1 = snap, 0 = encoders only).
ODOM_CAMERA_BLEND = 0.6      # was 0.25. The caster yaws the car without the encoders seeing it,
                             # so the camera has to carry more of the heading estimate.
LANE_MAX_HEADING = 0.5       # rad (~30 deg). Larger headings are clamped, not discarded
LANE_JUMP_M = 0.08           # offset jumping more than this in one frame is ignored...
LANE_JUMP_CONFIRM = 4        # ...unless it shows up this many frames in a row
LANE_LOST_TIMEOUT_S = 0.5    # camera gone this long -> UI "LANE_LOST" (still may roll on odom)
ODOM_BLIND_TIMEOUT_S = 1.2   # camera gone this long -> stop (odometry-only limit)
DRIVE_START_DELAY_S = 3.0    # lane following waits this long after Run before moving
DRIVE_END_LOST_S = 2.5       # no lane this long while following -> the run is over (end of the tape, or picked up)
DRIVE_RAMP_S = 1.5           # reach cruise speed over this long after GO, so the caster lines up at low speed
DRIVE_RAMP_START = 0.4       # fraction of cruise speed at the first instant of the ramp
RECORD_DRIVE = True          # save camera frames + a CSV of each lane-following run in runs/
RECORD_MAX_FRAMES = 3000     # about 100 s at 30 fps, roughly 30 MB
# --- Heading align (modes.py: spin until the nose matches the lane) ---
# heading > 0 means the car points right of the lane; spin left until ~0.
HEADING_ALIGN_KP = 1.2       # m/s of wheel spin per rad of heading error
HEADING_ALIGN_MAX_SPEED = 0.25  # m/s each wheel, hardest spin
HEADING_ALIGN_DONE = 0.06    # rad (~3.5 deg). Closer than this = facing the lines
HEADING_ALIGN_SETTLE_S = 0.5 # must stay inside DONE this long before stopping
HEADING_ALIGN_FILTER = 0.35  # 0-1, how much each new camera heading counts
HEADING_ALIGN_LOST_S = 1.0   # no lane this long while spinning -> stop
# --- Turn test (turntest.py: effective wheel base and turn response) ---
# Spins are whole turns by encoder count; the camera's residual heading when
# the lane comes back into view says how far off WHEEL_BASE_M is. Breakaway
# slip is a fixed few cm per wheel, so whole turns make it negligible.
TURN_TEST_SPIN_TURNS = 1     # full rotations per spin sample (1 keeps the lane in view afterward)
TURN_TEST_SPIN_REPEATS = 0   # right+left pairs of spins (0 = arcs only; wheel base is already measured)
TURN_TEST_SPIN_SPEED = 0.10  # m/s each wheel while spinning in place
# Arcs run for a fixed time, gently, so the camera heading stays under its
# clamp (~29 deg): 0.20 rad/s x 2 s = 23 deg.
TURN_TEST_ARC_REPEATS = 2    # right+left pairs of arcs (4 arcs ~ 1.6 m of track)
TURN_TEST_ARC_S = 2.0        # seconds per arc
TURN_TEST_ARC_SPEED = 0.20   # m/s forward during the arcs
TURN_TEST_ARC_OMEGA = 0.20   # rad/s commanded turn rate during the arcs
# --- Obstacle distances (m) ---
STOP_DISTANCE = 0.30         # something this close ahead -> STOP
SLOW_DISTANCE = 0.80         # something this close ahead -> drive slower
SIDE_CLEAR_DISTANCE = 1.00   # side must be clear this far to change lanes
LANE_COUNT = 3               # marked lanes on the course (RED|YEL|YEL|RED)
START_LANE = 0               # 0 = leftmost (red|yellow). The car starts here.
LANE_CHANGE_TIMEOUT_S = 8.0  # stop if a lane change has not finished by then (was 4; too short at these speeds)
LANE_CHANGE_RAMP_S = 0.6     # the target slides over to the next lane over this long. 1.0 crossed at ~20 deg in
                             # 4 s / 1 m of track; 0.6 aims for ~25-30 deg, the most the camera reads cleanly.
LANE_CHANGE_SPEED_FRAC = 1.0 # lane changes run at this fraction of cruise. Pure pursuit's turn rate scales
                             # with speed, so at half speed the car only crabbed ~3 cm/s and timed out.
LANE_CHANGE_SETTLE_S = 0.2   # new lane must stay centered at least this long (lane following finishes the centring)
LANE_CHANGE_DONE_M = 0.08    # centered: offset smaller than this
LANE_CHANGE_DONE_HEADING = 0.20  # and heading smaller than this (radians, ~11 deg)
# Last-resort stop inside the motor loop, closer than STOP_DISTANCE.
# Keep this equal to STOP_DISTANCE_M in sketch/sketch.ino. The VL53L1X and the lidar both use it.
ESTOP_DISTANCE_M = 0.15
# Bench tests leave the lidar staring at a wall, so the emergency stop
# stays off for those. "drive", "heading_align", and "straight_test" still stop.
# "web" blocks forward itself when the front is too close, so reverse
# still works. It is not in this list.
ESTOP_MODES = ("drive", "heading_align", "straight_test")
CMD_TIMEOUT_S = 0.30         # coast if wheel commands stop arriving

# --- Camera (MIPI-CSI camera on the UNO Media Carrier) ---
CAMERA_SOURCE = "csi"        # "csi" = Media Carrier camera via libcamera, "usb" = USB webcam
CAMERA_INDEX = 0             # /dev/video0, only used when CAMERA_SOURCE = "usb"
# "none" | "180" | "horizontal" | "vertical". Use "180" if the ribbon cable
# forces the camera to be mounted upside down. Lanes must look right on the page.
CAMERA_FLIP = "180"
# Size asked of libcamera before scaling down to FRAME_WIDTH x FRAME_HEIGHT.
# 816x612 makes libcamera read the IMX219's full-view 1640x1232 mode; smaller
# requests pick the 640x480 mode, which crops the middle (zoomed in). Larger
# outputs (1632x1224 and up) run out of the board's 32 MB of camera memory.
CAMERA_SENSOR_WIDTH = 816
CAMERA_SENSOR_HEIGHT = 612
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
# Automatic white balance on every frame. Removes the CSI camera's purple
# cast so the HSV ranges below see the tape's real colors.
CAMERA_WHITE_BALANCE = True

# --- Lane detection (lane_detector.py) ---
# Three lanes: RED | YELLOW | YELLOW | RED
#   lane 0 (left)   = red left    + yellow right
#   lane 1 (middle) = yellow left + yellow right
#   lane 2 (right)  = yellow left + red right
# The detector tells the lanes apart by the colors of the pair it follows.
# OpenCV HSV: H 0-179, S 0-255, V 0-255. Red wraps around hue 0.
# Measured on the CSI camera with white balance on: red tape H 6-17,
# yellow tape H 35-40, both S 72-106. The gray floor is S ~13.
LEFT_LINE_HSV_RANGES = [((0, 50, 80), (19, 255, 255)),
                        ((165, 50, 80), (179, 255, 255))]        # red
RIGHT_LINE_HSV_RANGES = [((26, 50, 100), (45, 255, 255))]        # yellow
LANE_WIDTH_M = 0.30          # distance between adjacent lane lines (TODO: measure)
# Which lane the car is in is decided on the floor, not on the image: each
# tape line is mapped to the floor and its sideways distance from the car is
# matched against the RED|YEL|YEL|RED pattern. That works at any heading.
LANE_LOCATE_TOL = 0.35       # lane widths; how far a line may miss its slot and still count
LANE_SWITCH_HYST = 0.15      # lane widths past a line before the detector calls it the next lane (no flip-flop on the line)
LANE_REF_BEHIND_CAMERA_M = 0.0  # judge the lane at this point behind the camera (0 = camera; set to the camera-to-axle distance to judge at the wheels)
LANE_ROI_TOP = 0.55          # ignore the top 55% of the image (above the road; the floor's horizon is ~52%)
LANE_BANDS = 8               # row bands used to fit each line
LANE_MIN_PIXELS = 60         # fewer matching pixels than this = line not seen
LANE_MIN_BAND_PIXELS = 4
LANE_DEFAULT_WIDTH_FRAC = 0.8   # lane width guess (fraction of image width) before both lines seen
LANE_MIN_WIDTH_FRAC = 0.4       # narrower than this = bad detection, don't learn it (a real lane is ~0.7-0.9 here)
LANE_HORIZON_FRAC = 0.4      # horizon row guess (fraction of image height); learned once both lines are seen
CAMERA_HFOV_DEG = 62         # camera horizontal field of view (TODO: set from the CSI camera's spec; 62 was the C920)
SHOW_LANES = True            # draw the detected lines on the camera video
# Top-down view on the web page (birdseye.py). Only drawn while the page shows it.
BIRDSEYE_RANGE_M = 1.2       # how far ahead of the camera it shows
BIRDSEYE_HALF_WIDTH_M = 0.45 # how far to each side
BIRDSEYE_PX_PER_M = 250

# --- Web remote (MODE = "web") ---
# Open http://10.194.91.71:8080 on a computer on the same network.
CAR_IP = "10.194.91.71"      # the Uno Q's address; the server still listens on every interface
WEB_PORT = 8080
WEB_MAX_SPEED = 0.75         # m/s, each wheel, manual driving (100% duty in the air; less on the floor)
# Manual Left/Right (see web.py speeds): tank steer — one wheel forward, one reverse.
WEB_CMD_TIMEOUT_S = 0.75     # coast if the page stops sending commands
WEB_HEARTBEAT_S = 3.0        # self-driving modes stop if the page is gone this long
KP_STRAIGHT_HOLD = 4.0       # m/s of wheel trim per meter the wheels differ, driving straight

# --- Lidar (STL19P / LD19) ---
# Wired to the microcontroller: lidar TX -> D0, 5V -> Uno 5V, GND shared.
# Angle offset and sector width are in sketch/sketch.ino.
LIDAR_PORT = "D0"
LIDAR_BAUD = 230400
LIDAR_ANGLE_OFFSET_DEG = 0   # rotate if the lidar's 0 deg isn't the car's front
LIDAR_HALF_WIDTH_DEG = 15    # each direction covers +/- this many degrees

# --- Wheel speed controller ---
# Feed-forward from the duty-cycle benchmark (speed is close to linear in duty):
#   left 100% -> 0.75 m/s, right 100% -> 0.78 m/s (DRV8833, in the air, 2026-11-19)
# Same units as METERS_PER_TICK. Re-run the benchmark if you change it again.
KF_LEFT = 331.0              # PWM (0-255 scale) per m/s
KF_RIGHT = 322.0
# Foam breakaway (sketch uses these). Air was ~15 each; right is stickier on the mat.
PWM_OFFSET_LEFT = 40.0       # ~16% duty (breakaway guess for the DRV8833; the PI covers the rest)
PWM_OFFSET_RIGHT = 40.0
KP_SPEED = 300.0             # the sketch averages speed over 100 ms, so P can be firm
KI_SPEED = 250.0             # integrates only inside I_BAND (0.08 m/s) in the sketch
I_LIMIT = 50.0               # max PWM the integral can add

# --- Loop ---
LOOP_HZ = 20
