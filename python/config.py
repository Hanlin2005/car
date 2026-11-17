# All tunable numbers in one place.
# The microcontroller pins are in sketch/pins.h.

# --- What to run ---
# Modes are picked and started on the web page (http://10.194.91.71:8080).
# This only chooses which one is selected when the program starts.
# "web" (manual driving) also starts running right away; the others wait
# for the Run button.
# "web" | "drive" | "straight_test" | "duty_test" | "motor_test" | "lidar_test"
MODE = "web"

# --- Robot geometry ---
WHEEL_BASE_M = 0.15          # distance between the two driven wheels (TODO: measure)
METERS_PER_TICK = 0.0005     # wheel circumference / encoder ticks per revolution

# --- Speeds (m/s) ---
# The page's speed slider sets these while a run is going.
# SLOW_SPEED stays half of CRUISE_SPEED.
CRUISE_SPEED = 0.30
SLOW_SPEED = 0.15

# --- Lane following (controller.py LaneFollower) ---
# The web page can change these two while a run is going.
VISION_CONTROL = True       # off: drive straight, ignore the lane lines
VISION_GAIN = 1.0           # multiplies both gains below. 1.0 is the normal strength
KP_OFFSET = 5.0              # rad/s of turn per meter off center (5 cm off -> 0.25 rad/s)
KP_HEADING = 1.5             # rad/s of turn per rad pointing away from the lane (damping)
LANE_FILTER = 0.4            # 0-1, how much each new camera frame counts (lower = smoother, laggier)
MAX_OMEGA = 1.2              # rad/s, hardest turn allowed (0.3 m/s -> 25 cm turning radius)
MAX_OMEGA_RATE = 6.0         # rad/s per second, how fast the turn rate can change
TURN_SLOWDOWN = 0.4          # at the hardest turn, drive this fraction slower
LANE_MAX_HEADING = 0.5       # rad (~30 deg). A bigger heading reading is a bad frame
LANE_JUMP_M = 0.08           # offset jumping more than this in one frame is a bad frame...
LANE_JUMP_CONFIRM = 4        # ...unless it shows up this many frames in a row
LANE_LOST_TIMEOUT_S = 0.5    # lane gone this long -> stop (shorter gaps: drive straight)
DRIVE_START_DELAY_S = 3.0    # lane following waits this long after Run before moving
RECORD_DRIVE = True          # save camera frames + a CSV of each lane-following run in runs/
RECORD_MAX_FRAMES = 3000     # about 100 s at 30 fps, roughly 30 MB
# --- Obstacle distances (m) ---
STOP_DISTANCE = 0.30         # something this close ahead -> STOP
SLOW_DISTANCE = 0.80         # something this close ahead -> drive slower
SIDE_CLEAR_DISTANCE = 1.00   # side must be clear this far to change lanes
LANE_COUNT = 2               # marked lanes on the course
START_LANE = 0               # leftmost lane. The car starts here.
LANE_CHANGE_TIMEOUT_S = 4.0  # stop if a lane change has not finished by then
LANE_CHANGE_SETTLE_S = 0.4   # new lane must stay centered at least this long
LANE_CHANGE_DONE_M = 0.06    # centered: offset smaller than this
LANE_CHANGE_DONE_HEADING = 0.12  # and heading smaller than this (radians)
# Last-resort stop inside the motor loop, closer than STOP_DISTANCE.
# Keep this equal to STOP_DISTANCE_M in sketch/sketch.ino. The VL53L1X and the lidar both use it.
ESTOP_DISTANCE_M = 0.15
# Bench tests leave the lidar staring at a wall, so the emergency stop
# stays off for those. "drive" and "straight_test" still stop.
# "web" blocks forward itself when the front is too close, so reverse
# still works. It is not in this list.
ESTOP_MODES = ("drive", "straight_test")
CMD_TIMEOUT_S = 0.30         # coast if wheel commands stop arriving

# --- Camera (MIPI-CSI camera on the UNO Media Carrier) ---
CAMERA_SOURCE = "csi"        # "csi" = Media Carrier camera via libcamera, "usb" = USB webcam
CAMERA_INDEX = 0             # /dev/video0, only used when CAMERA_SOURCE = "usb"
# "none" | "180" | "horizontal" | "vertical". Use "180" if the ribbon cable
# forces the camera to be mounted upside down. Lanes must look right on the page.
CAMERA_FLIP = "none"
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# --- Lane detection (lane_detector.py) ---
# Left line is YELLOW, right line is RED. OpenCV HSV: H 0-179, S 0-255, V 0-255.
# Red wraps around hue 0, so it needs two ranges.
# Tuned on the old USB webcam. The CSI camera sees color differently: retune.
LEFT_LINE_HSV_RANGES = [((18, 90, 90), (35, 255, 255))]          # yellow
RIGHT_LINE_HSV_RANGES = [((0, 100, 70), (10, 255, 255)),
                         ((165, 100, 70), (179, 255, 255))]      # red
LANE_WIDTH_M = 0.30          # distance between the yellow and red lines (TODO: measure)
LANE_ROI_TOP = 0.45          # ignore the top 45% of the image (above the road)
LANE_BANDS = 8               # row bands used to fit each line
LANE_MIN_PIXELS = 60         # fewer matching pixels than this = line not seen
LANE_MIN_BAND_PIXELS = 4
LANE_DEFAULT_WIDTH_FRAC = 0.8   # lane width guess (fraction of image width) before both lines seen
LANE_MIN_WIDTH_FRAC = 0.2       # narrower than this = bad detection, don't learn it
LANE_HORIZON_FRAC = 0.4      # horizon row guess (fraction of image height); learned once both lines are seen
CAMERA_HFOV_DEG = 62         # camera horizontal field of view (TODO: set from the CSI camera's spec; 62 was the C920)
SHOW_LANES = True            # draw the detected lines on the camera video

# --- Web remote (MODE = "web") ---
# Open http://10.194.91.71:8080 on a computer on the same network.
CAR_IP = "10.194.91.71"      # the Uno Q's address; the server still listens on every interface
WEB_PORT = 8080
WEB_MAX_SPEED = 0.57         # m/s, each wheel, manual driving (right wheel's 100% duty speed)
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
#   left 100% -> 0.597 m/s, right 100% -> 0.571 m/s
# Same units as METERS_PER_TICK: re-run the benchmark if you change it.
KF_LEFT = 422.0              # PWM (0-255 scale) per m/s
KF_RIGHT = 441.0
PWM_OFFSET = 3.0             # PWM needed just to start moving
KP_SPEED = 200.0
KI_SPEED = 400.0
I_LIMIT = 40.0               # max PWM the integral can add

# --- Loop ---
LOOP_HZ = 20
