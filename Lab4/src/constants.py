# Constants

# region ROBOT
# Speed Related
LINEAR_MAX = 0.05
ANGULAR_MAX = 0.15

# Warning threshold in Robot.change_wheel_speed
WARN_WHEEL_LINEAR_SPEED_MAX = 0.5
WARN_WHEEL_ANGULAR_SPEED_MAX = 10

# Movement Related
LINEAR_DRIVE_THRESHOLD = 0.05       # Linear threshold to stop linear movement when togo < threshold
ANGULAR_ROTATE_THRESHOLD = 0.05     # Angular threshold to stop linear movement when togo < threshold

# Publisher/Subscriber Related
DEFAULT_PUB_RATE = 50               # 10hz
WHEEL_PUB_DELAY = 0.05

# A STAR
A_STAR_PATH_TOLERANCE = 0.1

# PID
TIME_TOLERANCE_FACTOR = 1.2
CHECK_POS_RATE_PID = 20
CHECK_POS_THRESHOLD = 0.03

# endregion

# region Utils
# Pretty print
STR_PRETTIER_LENGTH = 24  # Length of pretty print
STR_PRETTIER_FILLCHAR = "-"

# endregion