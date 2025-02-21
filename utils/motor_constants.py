MAX_VOLTAGE = 12.0 

def voltage_to_percent(voltage):
    return (voltage / MAX_VOLTAGE) * 100

def percent_to_voltage(percent):
    return (percent / 100) * MAX_VOLTAGE

MOTOR_CONFIG = {
    "elevator": {
        "max_speed": 50
    },
    "climb": {
        "max_speed": 30
    },
    "wheels": {
        "max_speed": 100
    }
}