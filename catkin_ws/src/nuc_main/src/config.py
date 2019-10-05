DRIVE = {
    "WHEEL_RADIUS": 0.07,
    "DRIVE_GEARING": 26 * 3,  # 64 means a gearing of 64 to one
    "FLIPPER_GEARING": 16,
    "TRACKS_SEPARATION": 0.3,
    "CPR": 8192,
    "MAX_DRIVE_SPEED": 1000,
    "MAX_FLIPPER_SPEED": 1000,
    "MAX_CURRENT": 20
}

ODRIVES = {
    "FLIPPER": {
        "SERIAL_NO": "336631563536",
        "SERIAL_PORT": "serial:/dev/ttyACM0",
        "FRONT": {
            "AXIS": 0,
            "DIRECTION": 1
        },
        "REAR": {
            "AXIS": 1,
            "DIRECTION": 1
        }
    },
    "DRIVE": {
        "SERIAL_NO": "209137933548",
        "SERIAL_PORT": "serial:/dev/ttyACM1",
        "LEFT": {
            "AXIS": 0,
            "DIRECTION": 1
        },
        "RIGHT": {
            "AXIS": 1,
            "DIRECTION": 1
        }
    }
}

AXIS_LOG = {
    "PERIOD": 5,
    "ID_STATE": 0,
    "ID_VEL": 1,
    "ID_VEL_SET": 2,
    "ID_POS_SET": 3,
    "ID_POS": 4
}
