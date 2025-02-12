import math


pi = math.pi
elevator_gearbox_radius = 25
elevator_gear_radius = 1
elevator_sprocket = 24
elevator_sprocket_diameter = 2.866 # inches
single_rotation = (elevator_gear_radius / elevator_gearbox_radius) * (2 * pi * elevator_sprocket)
ELEVATOR_LEVELS = ["LEVEL_1", "LEVEL_2", "LEVEL_3", "LEVEL_4"]
MAX_ELEVATOR_HEIGHT = 60 # inches
MIN_ELEVATOR_HEIGHT =  0 # inches