# poses.py

from dataclasses import dataclass

@dataclass
class ObusPose:
    x: float
    y: float
    z: float
    a: float
    b: float
    c: float

# Constantes base
POSE_B = 0
POSE_C = 179
POSE_A_LEFT = -179
POSE_A_RIGHT = 1
POSE_Z_SAFE = 1550

# Distancias
center_distance_16 = 129.38
center_distance_8 = 260
center_distance_4 = 280
center_distance_2 = 380

# Tabla de poses
poses = {
    'H16': {
        1: ObusPose(x=-260, y=-1342, z=POSE_Z_SAFE, a=-179, b=POSE_B, c=POSE_C),
        2: ObusPose(x=-260 + center_distance_16, y=-1342, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        3: ObusPose(x=-260 + 2 * center_distance_16, y=-1342, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        4: ObusPose(x=-260 + 3 * center_distance_16, y=-1342, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        5: ObusPose(x=11, y=-1342, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
        6: ObusPose(x=11 + center_distance_16, y=-1342, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
        7: ObusPose(x=11 + 2 * center_distance_16, y=-1342, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
        8: ObusPose(x=11 + 3 * center_distance_16, y=-1342, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
        9: ObusPose(x=-260, y=-1875, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        10: ObusPose(x=-260 + center_distance_16, y=-1875, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        11: ObusPose(x=-260 + 2 * center_distance_16, y=-1875, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        12: ObusPose(x=-260 + 3 * center_distance_16, y=-1875, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        13: ObusPose(x=11, y=-1875, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
        14: ObusPose(x=11 + center_distance_16, y=-1875, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
        15: ObusPose(x=11 + 2 * center_distance_16, y=-1875, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
        16: ObusPose(x=11 + 3 * center_distance_16, y=-1875, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
    },
    'H8': {
        1: ObusPose(x=1645.58, y=-412.77, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        2: ObusPose(x=1645.58 + center_distance_8, y=-412.77, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        3: ObusPose(x=1514.58, y=-412.77, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
        4: ObusPose(x=1514.58 + center_distance_8, y=-412.77, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
        5: ObusPose(x=1645.58, y=-116.31, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        6: ObusPose(x=1645.58 + center_distance_8, y=-116.31, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        7: ObusPose(x=1514.58, y=-116.31, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
        8: ObusPose(x=1514.58 + center_distance_8, y=-116.31, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
    },
    'H4': {
        1: ObusPose(x=1803.39, y=-346.9, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        2: ObusPose(x=1803.39, y=-346.9, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        3: ObusPose(x=1803.39, y=-13.90, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
        4: ObusPose(x=1803.39, y=-13.90, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
    },
    'H2': {
        1: ObusPose(x=-194.23, y=-1590.28, z=POSE_Z_SAFE, a=POSE_A_LEFT, b=POSE_B, c=POSE_C),
        2: ObusPose(x=82.23, y=-1600, z=POSE_Z_SAFE, a=POSE_A_RIGHT, b=POSE_B, c=POSE_C),
    },
    'TABLE': ObusPose(x=1620, y=-8.45, z=POSE_Z_SAFE, a=-178, b=POSE_B, c=POSE_C)
}

