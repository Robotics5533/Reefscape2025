from utils.constants import single_rotation_inches

def inchesToRotations(inches: float) -> float:
    return inches / single_rotation_inches

def is_between(value: float, max: float, min: float):
    return min <= value <= max