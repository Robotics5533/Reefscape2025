import math

# Math constants
pi = math.pi

# Elevator mechanical constants
elevator_gearbox_radius = 25
elevator_gear_radius = 1
elevator_sprocket = 24
elevator_sprocket_diameter = 2.866  # inches
single_rotation = (elevator_gear_radius / elevator_gearbox_radius) * (2 * pi * elevator_sprocket)

# Elevator position constants
ELEVATOR_LEVELS = ["LEVEL_1", "LEVEL_2", "LEVEL_3", "LEVEL_4"]
MAX_ELEVATOR_HEIGHT = 12  # inches
MIN_ELEVATOR_HEIGHT = 0   # inches

# Motor CAN IDs
ELEVATOR_LEADING_MOTOR_ID = 13
ELEVATOR_FOLLOWING_MOTOR_ID = 14
CLIMB_MOTOR_ID = 15
BOTTOM_WHEELS_MOTOR_ID = 16