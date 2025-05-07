# config.py

# Paths
PLUGIN_PATH = "/home/suippes/kuka_catkin_ws/src/rqt_kuka/"
UI_PATH = PLUGIN_PATH + "resource/RqtKuka.ui"
IMG_PATH = PLUGIN_PATH + "resource/images/"

# Estados FSM
STATE_IDLE = 0
STATE_MOVING_TO_PREPICK = 1
STATE_DOING_PICK_TEST = 2
STATE_PICKED = 3
STATE_MOVING_TO_PLACE = 4
STATE_PLACED = 5
STATE_HOMING = 6

# Otros parámetros
PREPLACE_ANGLE_LIMIT = 20
PREPICK_ANGLE_LIMIT = 90


