#!/usr/bin/python

import os
import inspect

import rospy
import rospkg
import time
import datetime 
import xacro
import subprocess
import sys
import QtCore
import QtGui
import numpy
from obuses_poses import *

from PyQt5 import QtCore, QtGui

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialog, QFileDialog, QMessageBox, QTableWidgetItem
from std_msgs.msg import Bool, Float64, Float32
from sensor_msgs.msg import JointState
from robotnik_msgs.srv import home, set_odometry, set_CartesianEuler_pose, set_digital_output, set_float_value
from robotnik_msgs.msg import Cartesian_Euler_pose, RobotnikMotorsStatus, MotorStatus
from geometry_msgs.msg import Pose, Point, Quaternion


import yaml
#from rqt_kuka_goto_dialog import RqtGoto

# states
CURRENT_STATE=0
STATE_IDLE=0
STATE_MOVING_TO_PREPICK=1
STATE_DOING_PICK_TEST=2
STATE_PICKED=3
STATE_MOVING_TO_PLACE=4
STATE_PLACED=5
STATE_HOMING=6

PATH="/home/asoriano/workspaces/kuka_catkin_ws/src/rqt_kuka/"

TOOL_HOMED=False
KUKA_AUT=False
finger_type=0
gauges_failure=False
under_voltage_tool=False
first_time_enabled=False
start_time_gauges=time.time()
#service names:
srv_name_move_abs_fast='/kuka_robot/setKukaAbsFast'
srv_name_move_abs_slow='/kuka_robot/setKukaAbs'
srv_name_move_rel_fast='/kuka_robot/setKukaRelFast'
srv_name_move_rel_slow='/kuka_robot/setKukaRel'
srv_tool_homing='/kuka_tool/robotnik_base_hw/home'
srv_finger_set_pose='/kuka_tool_finger_node/set_odometry' #robotnik_msgs.set.odometry
srv_digital_io='/kuka_tool/robotnik_base_hw/set_digital_output'
srv_limit_cont_current='/kuka_tool/robotnik_base_hw/set_continuous_current_limit'
srv_limit_peak_current='/kuka_tool/robotnik_base_hw/set_peak_current_limit'

#topic names:
topic_cart_pose_kuka='/kuka_robot/cartesian_pos_kuka'
topic_kuka_moving='/kuka_robot/kuka_moving'
topic_tool_weight='/phidget_load/load_mean'
topic_current='/kuka_tool/robotnik_base_hw/current0'
topic_horiz_force='/phidget_load/vertical_force'
topic_motor_status='/kuka_tool/robotnik_base_hw/status'

#Prepick Pose # tf.transformations.quaternion_from_euler(0, 0, th)
#Prepick_Pose=Pose(Point(100, 100, 100), Quaternion(0, 0, 0, 1))
#rosservice call /kuka_robot/setKukaAbs "{x: 1707.69, y: 235.42, z: 1435.39, A: -59.39, B: 0, C: -174}" 
#CAJA NEGRA
Preplace_Pose_x=1707.69
Preplace_Pose_y=235.42
Preplace_Pose_z=1435.39
Preplace_Pose_a_left=-71 #left
Preplace_Pose_a_right=-71 + 180 #+180 because Preplace_Pose_a_left<0 otherwise -180
Preplace_Pose_b=0#-0.21
Preplace_Pose_c=179#178.41

Preplace_angle_limit=20


#Preplace Pose
#Preplace_Pose=Pose(Point(400, 400, 100), Quaternion(0, 0, 0, 1))
#rosservice call /kuka_robot/setKukaAbs "{x: 255.69, y: 1704.42, z: 1475.39, A: -14.39, B: 0, C: 174}" 
#CAJA GRIS
Prepick_Pose_x=255.49
Prepick_Pose_y=1704.49
Prepick_Pose_z=1475.38
Prepick_Pose_a_left=-14#15.2
Prepick_Pose_a_right=Prepick_Pose_a_left+180 #+180 because Preplace_Pose_a_left<0 otherwise -180
Prepick_Pose_b=0.0#-0.12
Prepick_Pose_c=179.0#178.73

Prepick_angle_limit=90

#Homming Pose
Homming_Pose_x=1260.41
Homming_Pose_y=1284.82
Homming_Pose_z=1455.99
Homming_Pose_a=-120.03 # es importante que no este entre los limites de pick y place [20,90] si no, podria rotar en el sentido erroneo.
Homming_Pose_b=0.0
Homming_Pose_c=179.0#178.73

#RollerBench Pose
RollerBench_Pose=Pose(Point(200, 200, 100), Quaternion(0, 0, 0, 1))
pos_x_kuka=0.0
pos_y_kuka=0.0
pos_z_kuka=0.0
pos_a_kuka=0.0
pos_b_kuka=0.0
pos_c_kuka=0.0
weight_read=0.0
weight_empty=0.0
weight_reads=[0, 0, 0, 0, 0]
weight_expected_min = 9999
weight_expected_max = 9999
horiz_force_read=0.0
horiz_force_empty=0.0

class RqtKuka(Plugin):

    def __init__(self, context):
        super(RqtKuka, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RqtKuka')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_kuka'), 'resource', 'RqtKuka.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('RqtKukaUi')
        #keyboard management
        
        
        # add signals/slots
        #select obus calibre
        self._widget.calibre_comboBox.currentIndexChanged.connect(self.calibre_selected)
        #self._widget.calibre_comboBox.highlighted.connect(self.arm_activated)

        #Buttons
        self._widget.Home_Button.pressed.connect(self.press_homming_button)
        self._widget.Pick_Left_Button.pressed.connect(self.press_pick_left_button)
        self._widget.Pick_Right_Button.pressed.connect(self.press_pick_right_button)
        #self._widget.Place_Left_Button.pressed.connect(self.press_place_left_button)
        #self._widget.Place_Right_Button.pressed.connect(self.press_place_right_button)
        self._widget.Finger_Adjust_Button.pressed.connect(self.press_finger_adjust_button)
        self._widget.Tare_Button.pressed.connect(self.press_tare_button)
        self._widget.Tare_Reset_Button.pressed.connect(self.press_tare_reset_button)
        self._widget.Reset_Ext_Button.pressed.connect(self.press_reset_external_pc_button)
        self._widget.Run_Program_Button.pressed.connect(self.press_run_program_button)
        self._widget.MoveToTable_Button.pressed.connect(self.press_move_to_rotation_table_button)
        
        
        self._widget.PickTest_Button.pressed.connect(self.press_picktest_button)
        self._widget.Gripper_Homing_Button.pressed.connect(self.press_tool_homming)
        self._widget.Led_On_Button.pressed.connect(self.press_led_on_button)
        self._widget.Led_Off_Button.pressed.connect(self.press_led_off_button)
        self._widget.Light_On_Button.pressed.connect(self.press_light_on_button)
        self._widget.Light_Off_Button.pressed.connect(self.press_light_off_button)
        self._widget.Led_On_Button.setStyleSheet("color: rgb(80, 170, 80)")
        self._widget.Led_Off_Button.setStyleSheet("color: rgb(170, 80, 80)")
        self._widget.Light_On_Button.setStyleSheet("color: rgb(80, 170, 80)")
        self._widget.Light_Off_Button.setStyleSheet("color: rgb(170, 80, 80)")
        
        pixmap = QtGui.QPixmap(PATH+"resource/images/fondo_huevera_0.png")
        self._widget.background_plate.setPixmap(pixmap)
        
        self._widget.Home_Button.setEnabled(False)
        self._widget.Finger_Adjust_Button.setEnabled(False)
        self._widget.MoveToTable_Button.setEnabled(False)
                
        ##obuses buttons
        #Huevera_2
        h2o1posex=70
        h2o1posey=330
        #obus1     
        self._widget.Huevera2Obus1Button.setGeometry(h2o1posex,h2o1posey,41,111)
        self._widget.Huevera2Obus1Button.clicked.connect(self.press_obus2_1_button)
        self._widget.Huevera2Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo41x111.png"))
        self._widget.Huevera2Obus1Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo41x111.png")
        self._widget.Huevera2Obus1Button.setMask(mask.mask())
        self._widget.Huevera2Obus1Button.setMouseTracking(True)       
        self._widget.Huevera2Obus1Button.installEventFilter(self)
        #obus2
        self._widget.Huevera2Obus2Button.setGeometry(h2o1posex+70,h2o1posey,41,111)
        self._widget.Huevera2Obus2Button.clicked.connect(self.press_obus2_2_button)
        self._widget.Huevera2Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo41x111.png"))
        self._widget.Huevera2Obus2Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo41x111.png")
        self._widget.Huevera2Obus2Button.setMask(mask.mask())
        self._widget.Huevera2Obus2Button.setMouseTracking(True)       
        self._widget.Huevera2Obus2Button.installEventFilter(self)        
        
        #Huevera_4
        h4o1posex=30
        h4o1posey=320
        #obus1
        self._widget.Huevera4Obus1Button.setGeometry(h4o1posex,h4o1posey,37,102)
        self._widget.Huevera4Obus1Button.clicked.connect(self.press_obus4_1_button)
        self._widget.Huevera4Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101.png"))
        self._widget.Huevera4Obus1Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo31x101.png")
        self._widget.Huevera4Obus1Button.setMask(mask.mask())
        self._widget.Huevera4Obus1Button.setMouseTracking(True)       
        self._widget.Huevera4Obus1Button.installEventFilter(self)
        #obus2
        self._widget.Huevera4Obus2Button.setGeometry(h4o1posex+50,h4o1posey,37,102)
        self._widget.Huevera4Obus2Button.clicked.connect(self.press_obus4_2_button)
        self._widget.Huevera4Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101.png"))
        self._widget.Huevera4Obus2Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo31x101.png")
        self._widget.Huevera4Obus2Button.setMask(mask.mask())
        self._widget.Huevera4Obus2Button.setMouseTracking(True)       
        self._widget.Huevera4Obus2Button.installEventFilter(self)
        #obus3
        self._widget.Huevera4Obus3Button.setGeometry(h4o1posex+102,h4o1posey,37,102)
        self._widget.Huevera4Obus3Button.clicked.connect(self.press_obus4_3_button)
        self._widget.Huevera4Obus3Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101.png"))
        self._widget.Huevera4Obus3Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo31x101.png")
        self._widget.Huevera4Obus3Button.setMask(mask.mask())
        self._widget.Huevera4Obus3Button.setMouseTracking(True)       
        self._widget.Huevera4Obus3Button.installEventFilter(self)
        #obus4
        self._widget.Huevera4Obus4Button.setGeometry(h4o1posex+152,h4o1posey,37,102)
        self._widget.Huevera4Obus4Button.clicked.connect(self.press_obus4_4_button)
        self._widget.Huevera4Obus4Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101.png"))
        self._widget.Huevera4Obus4Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo31x101.png")
        self._widget.Huevera4Obus4Button.setMask(mask.mask())
        self._widget.Huevera4Obus4Button.setMouseTracking(True)       
        self._widget.Huevera4Obus4Button.installEventFilter(self)

        #Huevera_8
        h8o1posex=40
        h8o1posey=320
        #obus1
        self._widget.Huevera8Obus1Button.setGeometry(h8o1posex,h8o1posey,26,71)
        self._widget.Huevera8Obus1Button.clicked.connect(self.press_obus8_1_button)
        self._widget.Huevera8Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71.png"))
        self._widget.Huevera8Obus1Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo26x71.png")
        self._widget.Huevera8Obus1Button.setMask(mask.mask())
        self._widget.Huevera8Obus1Button.setMouseTracking(True)       
        self._widget.Huevera8Obus1Button.installEventFilter(self)
        #obus2
        self._widget.Huevera8Obus2Button.setGeometry(h8o1posex+38,h8o1posey,26,71)
        self._widget.Huevera8Obus2Button.clicked.connect(self.press_obus8_2_button)
        self._widget.Huevera8Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71.png"))
        self._widget.Huevera8Obus2Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo26x71.png")
        self._widget.Huevera8Obus2Button.setMask(mask.mask())
        self._widget.Huevera8Obus2Button.setMouseTracking(True)       
        self._widget.Huevera8Obus2Button.installEventFilter(self)
        #obus3
        self._widget.Huevera8Obus3Button.setGeometry(h8o1posex+86,h8o1posey,26,71)
        self._widget.Huevera8Obus3Button.clicked.connect(self.press_obus8_3_button)
        self._widget.Huevera8Obus3Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71.png"))
        self._widget.Huevera8Obus3Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo26x71.png")
        self._widget.Huevera8Obus3Button.setMask(mask.mask())
        self._widget.Huevera8Obus3Button.setMouseTracking(True)       
        self._widget.Huevera8Obus3Button.installEventFilter(self)
        #obus4
        self._widget.Huevera8Obus4Button.setGeometry(h8o1posex+132,h8o1posey,26,71)
        self._widget.Huevera8Obus4Button.clicked.connect(self.press_obus8_4_button)
        self._widget.Huevera8Obus4Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71.png"))
        self._widget.Huevera8Obus4Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo26x71.png")
        self._widget.Huevera8Obus4Button.setMask(mask.mask())
        self._widget.Huevera8Obus4Button.setMouseTracking(True)       
        self._widget.Huevera8Obus4Button.installEventFilter(self)
        #obus5
        self._widget.Huevera8Obus5Button.setGeometry(h8o1posex+18,h8o1posey+51,26,71)
        self._widget.Huevera8Obus5Button.clicked.connect(self.press_obus8_5_button)
        self._widget.Huevera8Obus5Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71.png"))
        self._widget.Huevera8Obus5Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_arriba26x71.png")
        self._widget.Huevera8Obus5Button.setMask(mask.mask())
        self._widget.Huevera8Obus5Button.setMouseTracking(True)       
        self._widget.Huevera8Obus5Button.installEventFilter(self)
        #obus6
        self._widget.Huevera8Obus6Button.setGeometry(h8o1posex+63,h8o1posey+51,26,71)
        self._widget.Huevera8Obus6Button.clicked.connect(self.press_obus8_6_button)
        self._widget.Huevera8Obus6Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71.png"))
        self._widget.Huevera8Obus6Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_arriba26x71.png")
        self._widget.Huevera8Obus6Button.setMask(mask.mask())
        self._widget.Huevera8Obus6Button.setMouseTracking(True)       
        self._widget.Huevera8Obus6Button.installEventFilter(self)
        #obus7
        self._widget.Huevera8Obus7Button.setGeometry(h8o1posex+109,h8o1posey+51,26,71)
        self._widget.Huevera8Obus7Button.clicked.connect(self.press_obus8_7_button)
        self._widget.Huevera8Obus7Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71.png"))
        self._widget.Huevera8Obus7Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_arriba26x71.png")
        self._widget.Huevera8Obus7Button.setMask(mask.mask())
        self._widget.Huevera8Obus7Button.setMouseTracking(True)       
        self._widget.Huevera8Obus7Button.installEventFilter(self)
        #obus8
        self._widget.Huevera8Obus8Button.setGeometry(h8o1posex+150,h8o1posey+51,26,71)
        self._widget.Huevera8Obus8Button.clicked.connect(self.press_obus8_8_button)
        self._widget.Huevera8Obus8Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71.png"))
        self._widget.Huevera8Obus8Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_arriba26x71.png")
        self._widget.Huevera8Obus8Button.setMask(mask.mask())
        self._widget.Huevera8Obus8Button.setMouseTracking(True)       
        self._widget.Huevera8Obus8Button.installEventFilter(self)
        
        #huevera16
        h16o1posex=40
        h16o1posey=315
        #obus1
        self._widget.Huevera16Obus1Button.setGeometry(h16o1posex,h16o1posey,19,51)
        self._widget.Huevera16Obus1Button.clicked.connect(self.press_obus16_1_button)
        self._widget.Huevera16Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus1Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus1Button.setMask(mask.mask())
        self._widget.Huevera16Obus1Button.setMouseTracking(True)       
        self._widget.Huevera16Obus1Button.installEventFilter(self)
        #obus2
        self._widget.Huevera16Obus2Button.setGeometry(h16o1posex+22,h16o1posey,19,51)
        self._widget.Huevera16Obus2Button.clicked.connect(self.press_obus16_2_button)
        self._widget.Huevera16Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus2Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus2Button.setMask(mask.mask())
        self._widget.Huevera16Obus2Button.setMouseTracking(True)       
        self._widget.Huevera16Obus2Button.installEventFilter(self)
        #obus3
        self._widget.Huevera16Obus3Button.setGeometry(h16o1posex+42,h16o1posey,19,51)
        self._widget.Huevera16Obus3Button.clicked.connect(self.press_obus16_3_button)
        self._widget.Huevera16Obus3Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus3Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus3Button.setMask(mask.mask())
        self._widget.Huevera16Obus3Button.setMouseTracking(True)       
        self._widget.Huevera16Obus3Button.installEventFilter(self)
        #obus4
        self._widget.Huevera16Obus4Button.setGeometry(h16o1posex+63,h16o1posey,19,51)
        self._widget.Huevera16Obus4Button.clicked.connect(self.press_obus16_4_button)
        self._widget.Huevera16Obus4Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus4Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus4Button.setMask(mask.mask())
        self._widget.Huevera16Obus4Button.setMouseTracking(True)       
        self._widget.Huevera16Obus4Button.installEventFilter(self)
        #obus5
        self._widget.Huevera16Obus5Button.setGeometry(h16o1posex+86,h16o1posey,19,51)
        self._widget.Huevera16Obus5Button.clicked.connect(self.press_obus16_5_button)
        self._widget.Huevera16Obus5Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus5Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus5Button.setMask(mask.mask())
        self._widget.Huevera16Obus5Button.setMouseTracking(True)       
        self._widget.Huevera16Obus5Button.installEventFilter(self)
        #obus6
        self._widget.Huevera16Obus6Button.setGeometry(h16o1posex+109,h16o1posey,19,51)
        self._widget.Huevera16Obus6Button.clicked.connect(self.press_obus16_6_button)
        self._widget.Huevera16Obus6Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus6Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus6Button.setMask(mask.mask())
        self._widget.Huevera16Obus6Button.setMouseTracking(True)       
        self._widget.Huevera16Obus6Button.installEventFilter(self)
        #obus7
        self._widget.Huevera16Obus7Button.setGeometry(h16o1posex+132,h16o1posey,19,51)
        self._widget.Huevera16Obus7Button.clicked.connect(self.press_obus16_7_button)
        self._widget.Huevera16Obus7Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus7Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus7Button.setMask(mask.mask())
        self._widget.Huevera16Obus7Button.setMouseTracking(True)       
        self._widget.Huevera16Obus7Button.installEventFilter(self)
        #obus8
        self._widget.Huevera16Obus8Button.setGeometry(h16o1posex+152,h16o1posey,19,51)
        self._widget.Huevera16Obus8Button.clicked.connect(self.press_obus16_8_button)
        self._widget.Huevera16Obus8Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus8Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus8Button.setMask(mask.mask())
        self._widget.Huevera16Obus8Button.setMouseTracking(True)       
        self._widget.Huevera16Obus8Button.installEventFilter(self)
        #obus9
        self._widget.Huevera16Obus9Button.setGeometry(h16o1posex,h16o1posey+74,19,51)
        self._widget.Huevera16Obus9Button.clicked.connect(self.press_obus16_9_button)
        self._widget.Huevera16Obus9Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus9Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus9Button.setMask(mask.mask())
        self._widget.Huevera16Obus9Button.setMouseTracking(True)       
        self._widget.Huevera16Obus9Button.installEventFilter(self)
        #obus10
        self._widget.Huevera16Obus10Button.setGeometry(h16o1posex+22,h16o1posey+74,19,51)
        self._widget.Huevera16Obus10Button.clicked.connect(self.press_obus16_10_button)
        self._widget.Huevera16Obus10Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus10Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus10Button.setMask(mask.mask())
        self._widget.Huevera16Obus10Button.setMouseTracking(True)       
        self._widget.Huevera16Obus10Button.installEventFilter(self)
        #obus11
        self._widget.Huevera16Obus11Button.setGeometry(h16o1posex+42,h16o1posey+74,19,51)
        self._widget.Huevera16Obus11Button.clicked.connect(self.press_obus16_11_button)
        self._widget.Huevera16Obus11Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus11Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus11Button.setMask(mask.mask())
        self._widget.Huevera16Obus11Button.setMouseTracking(True)       
        self._widget.Huevera16Obus11Button.installEventFilter(self)
        #obus12
        self._widget.Huevera16Obus12Button.setGeometry(h16o1posex+63,h16o1posey+74,19,51)
        self._widget.Huevera16Obus12Button.clicked.connect(self.press_obus16_12_button)
        self._widget.Huevera16Obus12Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus12Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus12Button.setMask(mask.mask())
        self._widget.Huevera16Obus12Button.setMouseTracking(True)       
        self._widget.Huevera16Obus12Button.installEventFilter(self)
        #obus13
        self._widget.Huevera16Obus13Button.setGeometry(h16o1posex+86,h16o1posey+74,19,51)
        self._widget.Huevera16Obus13Button.clicked.connect(self.press_obus16_13_button)
        self._widget.Huevera16Obus13Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus13Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus13Button.setMask(mask.mask())
        self._widget.Huevera16Obus13Button.setMouseTracking(True)       
        self._widget.Huevera16Obus13Button.installEventFilter(self)
        #obus14
        self._widget.Huevera16Obus14Button.setGeometry(h16o1posex+109,h16o1posey+74,19,51)
        self._widget.Huevera16Obus14Button.clicked.connect(self.press_obus16_14_button)
        self._widget.Huevera16Obus14Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus14Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus14Button.setMask(mask.mask())
        self._widget.Huevera16Obus14Button.setMouseTracking(True)       
        self._widget.Huevera16Obus14Button.installEventFilter(self)
        #obus15
        self._widget.Huevera16Obus15Button.setGeometry(h16o1posex+132,h16o1posey+74,19,51)
        self._widget.Huevera16Obus15Button.clicked.connect(self.press_obus16_15_button)
        self._widget.Huevera16Obus15Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus15Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus15Button.setMask(mask.mask())
        self._widget.Huevera16Obus15Button.setMouseTracking(True)       
        self._widget.Huevera16Obus15Button.installEventFilter(self)
        #obus16
        self._widget.Huevera16Obus16Button.setGeometry(h16o1posex+152,h16o1posey+74,19,51)
        self._widget.Huevera16Obus16Button.clicked.connect(self.press_obus16_16_button)
        self._widget.Huevera16Obus16Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus16Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus16Button.setMask(mask.mask())
        self._widget.Huevera16Obus16Button.setMouseTracking(True)       
        self._widget.Huevera16Obus16Button.installEventFilter(self)        
        
        
        #subscriber to robot state
        rospy.Subscriber(topic_kuka_moving, Bool, self.callback_moving)

        #subscriber to robot pose
        rospy.Subscriber(topic_cart_pose_kuka, Cartesian_Euler_pose, self.callback_robot_pose)     

        #subscriber to tool weight detected
        rospy.Subscriber(topic_tool_weight, Float64, self.callback_tool_weight)     

        #subscriber to tool current
        rospy.Subscriber(topic_current, Float32, self.callback_current) 
                
        #subscriber to vertical force
        rospy.Subscriber(topic_horiz_force, Float64, self.callback_horiz_force)
        
        #subscriber to motor status of the tool
        rospy.Subscriber(topic_motor_status, RobotnikMotorsStatus, self.callback_motor_status)
        
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        #if context.serial_number() > 1:
        #    #self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        #    self._widget.setWindowTitle("Robotnik Kuka Interface")
        self._widget.setWindowTitle(" ")
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        self._yaml_file = ""
        self._params = dict()
        self._name = "RqtKuka"
        
        self._keys_not_steps = ['arm_ip', 'arm_port', 'joint_names', 'group_name', 'action_ns']

    #filtro para detectar el raton
    def eventFilter(self, object, event):

        #huevera 16
        if finger_type == 1 :
            if event.type() == QtCore.QEvent.HoverEnter:
                #obuses hacia abajo (los de arriba [1-8])
                if object == self._widget.Huevera16Obus1Button or object == self._widget.Huevera16Obus2Button or object == self._widget.Huevera16Obus3Button or object == self._widget.Huevera16Obus4Button or object == self._widget.Huevera16Obus5Button or object == self._widget.Huevera16Obus6Button or object == self._widget.Huevera16Obus7Button or object == self._widget.Huevera16Obus8Button :
                    object.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51P.png"))
                #obuses hacia arriba
                else:
                    object.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51P.png"))
            if event.type() == QtCore.QEvent.HoverLeave:
                #obuses hacia abajo (los de arriba [1-8])
                if object == self._widget.Huevera16Obus1Button or object == self._widget.Huevera16Obus2Button or object == self._widget.Huevera16Obus3Button or object == self._widget.Huevera16Obus4Button or object == self._widget.Huevera16Obus5Button or object == self._widget.Huevera16Obus6Button or object == self._widget.Huevera16Obus7Button or object == self._widget.Huevera16Obus8Button :
                    object.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))
                #obuses hacia arriba
                else:
                    object.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))  

        #huevera 8
        if finger_type == 2 :
            if event.type() == QtCore.QEvent.HoverEnter:
                #obuses hacia abajo (los de arriba [1-4])
                if object == self._widget.Huevera8Obus1Button or object == self._widget.Huevera8Obus2Button or object == self._widget.Huevera8Obus3Button or object == self._widget.Huevera8Obus4Button :
                    object.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71P.png"))
                #obuses hacia arriba
                else:
                    object.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71P.png"))
            if event.type() == QtCore.QEvent.HoverLeave:
                #obuses hacia abajo (los de arriba [1-4])
                if object == self._widget.Huevera8Obus1Button or object == self._widget.Huevera8Obus2Button or object == self._widget.Huevera8Obus3Button or object == self._widget.Huevera8Obus4Button :
                    object.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71.png"))
                #obuses hacia arriba
                else:
                    object.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71.png"))        
        #huevera 4
        if finger_type == 3 :
            if event.type() == QtCore.QEvent.HoverEnter:
                object.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101P.png"))
            if event.type() == QtCore.QEvent.HoverLeave:
                object.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101.png"))
        #huevera 2
        if finger_type == 4 :
            if event.type() == QtCore.QEvent.HoverEnter:            
                object.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo41x111P.png"))
            if event.type() == QtCore.QEvent.HoverLeave:
                object.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo41x111.png"))
             
        return False
        

    def desactivate_buttons(self):
        self._widget.Home_Button.setEnabled(False)
        self._widget.PickTest_Button.setEnabled(False)
        self._widget.Gripper_Homing_Button.setEnabled(False)
        self._widget.Pick_Right_Button.setEnabled(False)
        self._widget.Pick_Left_Button.setEnabled(False)
        self._widget.MoveToTable_Button.setEnabled(False)
        #self._widget.Place_Right_Button.setEnabled(False)
        #self._widget.Place_Left_Button.setEnabled(False)
        self._widget.Finger_Adjust_Button.setEnabled(False)
        self._widget.Huevera2Obus1Button.setEnabled(False)
        self._widget.Huevera2Obus2Button.setEnabled(False)
        self._widget.Huevera4Obus1Button.setEnabled(False)
        self._widget.Huevera4Obus2Button.setEnabled(False)
        self._widget.Huevera4Obus3Button.setEnabled(False)
        self._widget.Huevera4Obus4Button.setEnabled(False)
        self._widget.Huevera8Obus1Button.setEnabled(False)
        self._widget.Huevera8Obus2Button.setEnabled(False)
        self._widget.Huevera8Obus3Button.setEnabled(False)
        self._widget.Huevera8Obus4Button.setEnabled(False)
        self._widget.Huevera8Obus5Button.setEnabled(False)
        self._widget.Huevera8Obus6Button.setEnabled(False)
        self._widget.Huevera8Obus7Button.setEnabled(False)
        self._widget.Huevera8Obus8Button.setEnabled(False)
        self._widget.Huevera16Obus1Button.setEnabled(False)
        self._widget.Huevera16Obus2Button.setEnabled(False)
        self._widget.Huevera16Obus3Button.setEnabled(False)
        self._widget.Huevera16Obus4Button.setEnabled(False)
        self._widget.Huevera16Obus5Button.setEnabled(False)
        self._widget.Huevera16Obus6Button.setEnabled(False)
        self._widget.Huevera16Obus7Button.setEnabled(False)
        self._widget.Huevera16Obus8Button.setEnabled(False)
        self._widget.Huevera16Obus9Button.setEnabled(False)
        self._widget.Huevera16Obus10Button.setEnabled(False)
        self._widget.Huevera16Obus11Button.setEnabled(False)
        self._widget.Huevera16Obus12Button.setEnabled(False)
        self._widget.Huevera16Obus13Button.setEnabled(False)
        self._widget.Huevera16Obus14Button.setEnabled(False)
        self._widget.Huevera16Obus15Button.setEnabled(False)
        self._widget.Huevera16Obus16Button.setEnabled(False)
        
    def activate_buttons(self):
        self._widget.Home_Button.setEnabled(True)
        self._widget.PickTest_Button.setEnabled(True)
        self._widget.Gripper_Homing_Button.setEnabled(True)
        self._widget.Pick_Right_Button.setEnabled(True)
        self._widget.Pick_Left_Button.setEnabled(True)
        self._widget.Finger_Adjust_Button.setEnabled(True)
        self._widget.MoveToTable_Button.setEnabled(True)
        #self._widget.Place_Right_Button.setEnabled(True)
        #self._widget.Place_Left_Button.setEnabled(True)
        self._widget.Huevera2Obus1Button.setEnabled(True)
        self._widget.Huevera2Obus2Button.setEnabled(True)
        self._widget.Huevera4Obus1Button.setEnabled(True)
        self._widget.Huevera4Obus2Button.setEnabled(True)
        self._widget.Huevera4Obus3Button.setEnabled(True)
        self._widget.Huevera4Obus4Button.setEnabled(True)
        self._widget.Huevera8Obus1Button.setEnabled(True)
        self._widget.Huevera8Obus2Button.setEnabled(True)
        self._widget.Huevera8Obus3Button.setEnabled(True)
        self._widget.Huevera8Obus4Button.setEnabled(True)
        self._widget.Huevera8Obus5Button.setEnabled(True)
        self._widget.Huevera8Obus6Button.setEnabled(True)
        self._widget.Huevera8Obus7Button.setEnabled(True)
        self._widget.Huevera8Obus8Button.setEnabled(True)
        self._widget.Huevera16Obus1Button.setEnabled(True)
        self._widget.Huevera16Obus2Button.setEnabled(True)
        self._widget.Huevera16Obus3Button.setEnabled(True)
        self._widget.Huevera16Obus4Button.setEnabled(True)
        self._widget.Huevera16Obus5Button.setEnabled(True)
        self._widget.Huevera16Obus6Button.setEnabled(True)
        self._widget.Huevera16Obus7Button.setEnabled(True)
        self._widget.Huevera16Obus8Button.setEnabled(True)
        self._widget.Huevera16Obus9Button.setEnabled(True)
        self._widget.Huevera16Obus10Button.setEnabled(True)
        self._widget.Huevera16Obus11Button.setEnabled(True)
        self._widget.Huevera16Obus12Button.setEnabled(True)
        self._widget.Huevera16Obus13Button.setEnabled(True)
        self._widget.Huevera16Obus14Button.setEnabled(True)
        self._widget.Huevera16Obus15Button.setEnabled(True)
        self._widget.Huevera16Obus16Button.setEnabled(True)       
        
    def callback_moving(self, data):
        global KUKA_AUT
        #print 'CB:moving_received:',data.data
        if data.data == True:
            KUKA_AUT=True
            self._widget.mode_label.setText("AUTOMATIC")
            self.desactivate_buttons()
            #selt._widget.mode_label.setStyleSheet(\ncolor: rgb(255, 0, 0))
                            
        else:
            KUKA_AUT=False
            self._widget.mode_label.setText("MANUAL")
            self.activate_buttons()
            
    def callback_motor_status(self,data):

		global under_voltage_tool, first_time_enabled, weight_empty, weight_read
		motor1=data.motor_status[1]
		driveflags_1=numpy.array(map(int,motor1.driveflags))
		under_voltage_1=driveflags_1[12]
		if(under_voltage_1==1):
			under_voltage_tool=True
			pixmap = QtGui.QPixmap(PATH+"resource/images/pinza_roja_peq2.png")
			self._widget.under_voltage_tool.setPixmap(pixmap)
		else:
			under_voltage_tool=False

		if(motor1.status=="OPERATION_ENABLED" and first_time_enabled):
			#if(weight_read-weight_empty<-10):
			ret = QMessageBox.warning(self._widget, "WARNING!", 'Weight detected', QMessageBox.Ok, QMessageBox.Cancel)
			first_time_enabled=False			
		if(motor1.status=="FAULT"):
			first_time_enabled=True
			

    def callback_robot_pose(self, data):
        global pos_x_kuka, pos_y_kuka, pos_z_kuka, pos_a_kuka, elapsed_time_gauges, gauges_failure
        #print 'CB:robot_pose_received',data
        pos_x_kuka=data.x
        pos_y_kuka=data.y
        pos_z_kuka=data.z
        pos_a_kuka=data.A
        pos_b_kuka=data.B
        pos_c_kuka=data.C
        elapsed_time_gauges=time.time()-start_time_gauges
        #print 'time between robot callback and gauges' ,elapsed_time_gauges
        if (elapsed_time_gauges>=2):
            gauges_failure=True
    def callback_horiz_force(self, data):
        global horiz_force_read, horiz_force_empty
        #print 'force_received:',data.data
        horiz_force_read = data.data
        self._widget.vertforce_lcdNumber.setDigitCount(4)
        self._widget.vertforce_lcdNumber.display(round((data.data-horiz_force_empty)*0.19,1))
        
    def callback_tool_weight(self, data):
        global weight_empty, weight_read, gauges_failure, start_time_gauges
        start_time_gauges=time.time()
        gauges_failure=False
        self._widget.weight_lcdNumber.setDigitCount(4)
        palette = self._widget.weight_lcdNumber.palette()       
        #print 'CB:tool_weight_received',data
        weight_read=data.data
        weight_no_tool=data.data-weight_empty
        weight_reads[0]=weight_no_tool
        for i in range(1, 5):
            weight_no_tool=weight_no_tool+weight_reads[i]
        weight_no_tool=weight_no_tool/5
        if(weight_no_tool<0 and weight_no_tool>-10):
            weight_no_tool=-weight_no_tool
        for i in range(1, 5):
            weight_reads[i]=weight_reads[i-1]
        self._widget.weight_lcdNumber.setDecMode()
        #self._widget.weight_lcdNumber.setNumDigits(3)
        self._widget.weight_lcdNumber.display(round(weight_no_tool,1))
        if weight_no_tool<weight_expected_min:
            palette.setColor(palette.WindowText, QtGui.QColor(10, 10, 10))
        elif weight_no_tool<weight_expected_max:
            palette.setColor(palette.WindowText, QtGui.QColor(20, 230, 20))
        else:
            palette.setColor(palette.WindowText, QtGui.QColor(255, 50, 50))
        self._widget.weight_lcdNumber.setPalette(palette)

    def callback_current(self, data):
        #print 'CB:current_received',data
        self._widget.tool_force_lcdNumber.setDigitCount(4)
        self._widget.tool_force_lcdNumber.display(round(data.data,1))
    
    def press_move_to_rotation_table_button(self):
        print 'move to rotation table'
    
    #pressing obuses
    #huevera2
    #obus1
    def press_obus2_1_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera2Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo41x111PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H2O1_Pose_x, H2O1_Pose_y, H2O1_Pose_z, H2O1_Pose_a, H2O1_Pose_b, H2O1_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera2Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo41x111.png"))
    #obus2
    def press_obus2_2_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera2Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo41x111PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H2O2_Pose_x, H2O2_Pose_y, H2O2_Pose_z, H2O2_Pose_a, H2O2_Pose_b, H2O2_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera2Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo41x111.png"))        
    #huevera4
    #obus1
    def press_obus4_1_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:           
            #cambia el color de la imagen
            self._widget.Huevera4Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H4O1_Pose_x, H4O1_Pose_y, H4O1_Pose_z, H4O1_Pose_a, H4O1_Pose_b, H4O1_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera4Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101.png"))    
    #obus2
    def press_obus4_2_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:    
            #cambia el color de la imagen
            self._widget.Huevera4Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H4O2_Pose_x, H4O2_Pose_y, H4O2_Pose_z, H4O2_Pose_a, H4O2_Pose_b, H4O2_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera4Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101.png"))
    #obus3
    def press_obus4_3_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:           
            #cambia el color de la imagen
            self._widget.Huevera4Obus3Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H4O3_Pose_x, H4O3_Pose_y, H4O3_Pose_z, H4O3_Pose_a, H4O3_Pose_b, H4O3_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera4Obus3Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101.png"))
    #obus4
    def press_obus4_4_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera4Obus4Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H4O4_Pose_x, H4O4_Pose_y, H4O4_Pose_z, H4O4_Pose_a, H4O4_Pose_b, H4O4_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera4Obus4Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo37x101.png"))    
    #huevera8
    #obus1
    def press_obus8_1_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera8Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H8O1_Pose_x, H8O1_Pose_y, H8O1_Pose_z, H8O1_Pose_a, H8O1_Pose_b, H8O1_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera8Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71.png"))    
    #obus2
    def press_obus8_2_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera8Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H8O2_Pose_x, H8O2_Pose_y, H8O2_Pose_z, H8O2_Pose_a, H8O2_Pose_b, H8O2_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera8Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71.png"))
    #obus3
    def press_obus8_3_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera8Obus3Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H8O3_Pose_x, H8O3_Pose_y, H8O3_Pose_z, H8O3_Pose_a, H8O3_Pose_b, H8O3_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera8Obus3Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71.png"))
    #obus4
    def press_obus8_4_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera8Obus4Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H8O4_Pose_x, H8O4_Pose_y, H8O4_Pose_z, H8O4_Pose_a, H8O4_Pose_b, H8O4_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera8Obus4Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo26x71.png"))    
    #obus5
    def press_obus8_5_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera8Obus5Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H8O5_Pose_x, H8O5_Pose_y, H8O5_Pose_z, H8O5_Pose_a, H8O5_Pose_b, H8O5_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera8Obus5Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71.png"))    
    #obus6
    def press_obus8_6_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:           
            #cambia el color de la imagen
            self._widget.Huevera8Obus6Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H8O6_Pose_x, H8O6_Pose_y, H8O6_Pose_z, H8O6_Pose_a, H8O6_Pose_b, H8O6_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera8Obus6Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71.png"))
    #obus7
    def press_obus8_7_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera8Obus7Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H8O7_Pose_x, H8O7_Pose_y, H8O7_Pose_z, H8O7_Pose_a, H8O7_Pose_b, H8O7_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera8Obus7Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71.png"))
    #obus8
    def press_obus8_8_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:           
            #cambia el color de la imagen
            self._widget.Huevera8Obus8Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H8O8_Pose_x, H8O8_Pose_y, H8O8_Pose_z, H8O8_Pose_a, H8O8_Pose_b, H8O8_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera8Obus8Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba26x71.png"))    
    #huevera16
    #obus1
    def press_obus16_1_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera16Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O1_Pose_x, H16O1_Pose_y, H16O1_Pose_z, H16O1_Pose_a, H16O1_Pose_b, H16O1_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))    
    #obus2
    def press_obus16_2_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera16Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O2_Pose_x, H16O2_Pose_y, H16O2_Pose_z, H16O2_Pose_a, H16O2_Pose_b, H16O2_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))
    #obus3
    def press_obus16_3_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera16Obus3Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O3_Pose_x, H16O3_Pose_y, H16O3_Pose_z, H16O3_Pose_a, H16O3_Pose_b, H16O3_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus3Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))
    #obus4
    def press_obus16_4_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera16Obus4Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O4_Pose_x, H16O4_Pose_y, H16O4_Pose_z, H16O4_Pose_a, H16O4_Pose_b, H16O4_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus4Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))    
    #obus5
    def press_obus16_5_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera16Obus5Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O5_Pose_x, H16O5_Pose_y, H16O5_Pose_z, H16O5_Pose_a, H16O5_Pose_b, H16O5_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus5Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))    
    #obus6
    def press_obus16_6_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:           
            #cambia el color de la imagen
            self._widget.Huevera16Obus6Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O6_Pose_x, H16O6_Pose_y, H16O6_Pose_z, H16O6_Pose_a, H16O6_Pose_b, H16O6_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus6Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))
    #obus7
    def press_obus16_7_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera16Obus7Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O7_Pose_x, H16O7_Pose_y, H16O7_Pose_z, H16O7_Pose_a, H16O7_Pose_b, H16O7_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus7Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png"))
    #obus8
    def press_obus16_8_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:           
            #cambia el color de la imagen
            self._widget.Huevera16Obus8Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O8_Pose_x, H16O8_Pose_y, H16O8_Pose_z, H16O8_Pose_a, H16O8_Pose_b, H16O8_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus8Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51.png")) 
    #obus9
    def press_obus16_9_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera16Obus9Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O9_Pose_x, H16O9_Pose_y, H16O9_Pose_z, H16O9_Pose_a, H16O9_Pose_b, H16O9_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus9Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))    
    #obus10
    def press_obus16_10_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera16Obus10Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O10_Pose_x, H16O10_Pose_y, H16O10_Pose_z, H16O10_Pose_a, H16O10_Pose_b, H16O10_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus10Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))
    #obus11
    def press_obus16_11_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera16Obus11Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O11_Pose_x, H16O11_Pose_y, H16O11_Pose_z, H16O11_Pose_a, H16O11_Pose_b, H16O11_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus11Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))
    #obus12
    def press_obus16_12_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera16Obus12Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O12_Pose_x, H16O12_Pose_y, H16O12_Pose_z, H16O12_Pose_a, H16O12_Pose_b, H16O12_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus12Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))    
    #obus13
    def press_obus16_13_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera16Obus13Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O13_Pose_x, H16O13_Pose_y, H16O13_Pose_z, H16O13_Pose_a, H16O13_Pose_b, H16O13_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus13Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))    
    #obus14
    def press_obus16_14_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:           
            #cambia el color de la imagen
            self._widget.Huevera16Obus14Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O14_Pose_x, H16O14_Pose_y, H16O14_Pose_z, H16O14_Pose_a, H16O14_Pose_b, H16O14_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus14Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))
    #obus15
    def press_obus16_15_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #cambia el color de la imagen
            self._widget.Huevera16Obus15Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O15_Pose_x, H16O15_Pose_y, H16O15_Pose_z, H16O15_Pose_a, H16O15_Pose_b, H16O15_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus15Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))
    #obus16
    def press_obus16_16_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:           
            #cambia el color de la imagen
            self._widget.Huevera16Obus16Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51PP.png"))
            #llamara al servicio de mover
            global KUKA_AUT
            #Call service to move robot up and then to the pre-pick pose, should be fast
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(H16O16_Pose_x, H16O16_Pose_y, H16O16_Pose_z, H16O16_Pose_a, H16O16_Pose_b, H16O16_Pose_c)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #y volvera a poner el color original
            self._widget.Huevera16Obus16Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_arriba19x51.png"))                
    def press_tool_homming(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nBe sure there is no obus picked', QMessageBox.Ok, QMessageBox.Cancel)
        #ret = QMessageBox.critical(self._widget, "WARNING!", 'The tool is activated and there is some weight \ndetected by the gauges!', QMessageBox.Ok)
        if ret == QMessageBox.Ok:
            #Call tool homing method
            global weight_empty, weight_read
            try:
                gripper_move_service = rospy.ServiceProxy(srv_finger_set_pose,set_odometry)
                homing_service = rospy.ServiceProxy(srv_tool_homing, home)           
                ret = homing_service()
                #weight_empty=weight_read
                gripper_move_service(0.02,0,0,-0.15)
                if ret == True:
                    TOOL_HOMED=True                 
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    
    def press_finger_adjust_button(self):
        
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nBe sure there is no obus picked', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            gripper_trasl_service = rospy.ServiceProxy(srv_finger_set_pose,set_odometry)
            if finger_type == 0:
                print 'No gripper selected'
            elif finger_type == 1:
                print 'Set gripper to 100mm'
                tras_from_homing=0.2-0.1;               
                ret=gripper_trasl_service(tras_from_homing,0,0,0)
            elif finger_type == 2:
                print 'Set gripper to 140mm'
                tras_from_homing=0.2-0.14;
                ret=gripper_trasl_service(tras_from_homing,0,0,0)
            elif finger_type == 3:
                print 'Set gripper to 160mm'
                tras_from_homing=0.2-0.16;
                ret=gripper_trasl_service(tras_from_homing,0,0,0)
            elif finger_type == 4:
                print 'Set gripper to 270mm'
                tras_from_homing=0.03;
                ret=gripper_trasl_service(tras_from_homing,0,0,0)
            
    def press_led_on_button(self):
        try:
            led_service = rospy.ServiceProxy(srv_digital_io, set_digital_output)
            ret = led_service(6,False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def press_led_off_button(self):
        try:
            led_service = rospy.ServiceProxy(srv_digital_io, set_digital_output)
            ret = led_service(6,True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def press_light_on_button(self):
        try:
            led_service = rospy.ServiceProxy(srv_digital_io, set_digital_output)
            ret = led_service(4,False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def press_light_off_button(self):
        try:
            led_service = rospy.ServiceProxy(srv_digital_io, set_digital_output)
            ret = led_service(4,True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def press_place_right_button(self):
        global KUKA_AUT, pos_z_kuka,pos_a_kuka
        #Call service to move robot up and then to the pre-pick pose, should be fast
        try:
            placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
            ret_rel=placed_rel_service(0, 0, Preplace_Pose_z-pos_z_kuka, 0, 0, 0)
            KUKA_AUT=True
            while KUKA_AUT: self.sleep_loop(0.1)
            placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)
            #if(pos_a_kuka<=-71 and pos_a_kuka>=-151):
            #if((pos_a_kuka<=190 and pos_a_kuka>=110) or (pos_a_kuka>=-179 and pos_a_kuka<=-151)):
            #if(pos_a_kuka <= Preplace_angle_limit and pos_a_kuka >= Preplace_Pose_a_left):
                #print "pass at -160 (-71-90)"
                #ret = placed_abs_service(Preplace_Pose_x, Preplace_Pose_y, Preplace_Pose_z, Preplace_Pose_a_left-90,Preplace_Pose_b,Preplace_Pose_c)
                #KUKA_AUT=True
                #while KUKA_AUT: self.sleep_loop(0.1)
            #print "go at 110 (-71+180)"
            ret = placed_abs_service(Preplace_Pose_x, Preplace_Pose_y, Preplace_Pose_z, Preplace_Pose_a_right,Preplace_Pose_b,Preplace_Pose_c)
            #ret=placed_rel_service(0, 0, -100, 0, 0, 0)
            #if ret == True:
            #   CURRENT_STATE=3
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def press_place_left_button(self):
        global KUKA_AUT, pos_z_kuka, pos_a_kuka
        #Call service to move robot up and then to the pre-pick pose, should be fast
        try:
            placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
            ret_rel=placed_rel_service(0, 0, Preplace_Pose_z-pos_z_kuka, 0, 0, 0)
            KUKA_AUT=True
            while KUKA_AUT: self.sleep_loop(0.1)
            placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)
            #if(pos_a_kuka >= Preplace_angle_limit and (pos_a_kuka <= Preplace_Pose_a_right)): 
                #print "pass at 160 (-71-90)"
                #ret = placed_abs_service(Preplace_Pose_x, Preplace_Pose_y, Preplace_Pose_z, Preplace_Pose_a_left-90,Preplace_Pose_b,Preplace_Pose_c)
                #KUKA_AUT=True
                #while KUKA_AUT: time.sleep(0.1)
            #print "go at -70"
            ret = placed_abs_service(Preplace_Pose_x, Preplace_Pose_y, Preplace_Pose_z, Preplace_Pose_a_left,Preplace_Pose_b,Preplace_Pose_c)
            #ret=placed_rel_service(0, 0, -100, 0, 0, 0)
            if ret == True:
                CURRENT_STATE=3
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def press_pick_left_button(self):
        global KUKA_AUT, pos_z_kuka, pos_a_kuka
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                picked_abs_service = rospy.ServiceProxy(srv_name_move_abs_fast, set_CartesianEuler_pose)
                #if((pos_a_kuka<=180 and pos_a_kuka>=90)):
                #if(pos_a_kuka>=Prepick_angle_limit and pos_a_kuka<=Prepick_Pose_a_right):
                #   print "pass at 76 (-14-90)"
                    #ret = picked_abs_service(Prepick_Pose_x, Prepick_Pose_y, Prepick_Pose_z, Prepick_Pose_a_left-90,Prepick_Pose_b,Prepick_Pose_c)
                    #KUKA_AUT=True
                    #while KUKA_AUT: time.sleep(0.1)
                ret = picked_abs_service(Prepick_Pose_x, Prepick_Pose_y, Prepick_Pose_z, Prepick_Pose_a_left,Prepick_Pose_b,Prepick_Pose_c)         
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            
    def press_pick_right_button(self):
        global KUKA_AUT, pos_z_kuka, pos_a_kuka
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:       
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                picked_abs_service = rospy.ServiceProxy(srv_name_move_abs_fast, set_CartesianEuler_pose)
                #if (pos_a_kuka<=Prepick_angle_limit and pos_a_kuka>=Prepick_Pose_a_left):
                        #ret = picked_abs_service(Prepick_Pose_x, Prepick_Pose_y, Prepick_Pose_z, Prepick_Pose_a_left-90,Prepick_Pose_b,Prepick_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: time.sleep(0.1)
                ret = picked_abs_service(Prepick_Pose_x, Prepick_Pose_y, Prepick_Pose_z, Prepick_Pose_a_right,Prepick_Pose_b,Prepick_Pose_c)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e           
            
    def press_homming_button(self):     
        global KUKA_AUT, pos_z_kuka, pos_a_kuka
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            try:
                homming_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=homming_rel_service(0, 0,Homming_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.1)
                homming_abs_service = rospy.ServiceProxy(srv_name_move_abs_fast, set_CartesianEuler_pose)
                #DE MOMENTO EL ANGULO EN EL HOMMING NO SE MODIFICA
                ret = homming_abs_service(Homming_Pose_x, Homming_Pose_y, Homming_Pose_z, Homming_Pose_a,Homming_Pose_b,Homming_Pose_c)
                #ret=placed_rel_service(0, 0, -100, 0, 0, 0)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            
    def press_picktest_button(self):
        global KUKA_AUT
        try:
            placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
            ret_rel=placed_rel_service(0, 0, 20, 0, 0, 0)
            #KUKA_AUT=True
            #while KUKA_AUT: time.sleep(0.1)
            if ret_rel == True:
                    CURRENT_STATE=STATE_DOING_PICK_TEST
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def press_tool_straighten(self):
        global KUKA_AUT,pos_a_kuka,pos_b_kuka,pos_c_kuka,pos_x_kuka,pos_y_kuka,pos_z_kuka
        print "Service called"
        try:
            #KUKA_AUT=True
            #while KUKA_AUT: time.sleep(0.1)        
            tool_straighten_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)
            ret = tool_straighten_service(pos_x_kuka, pos_y_kuka, pos_z_kuka, pos_a_kuka,0.0,179)

            #KUKA_AUT=True
            #while KUKA_AUT: time.sleep(0.1)
            #ret = tool_straighten_service(pos_x_kuka, pos_y_kuka, pos_z_kuka, pos_a_kuka,0.0,pos_c_kuka)
            #ret=placed_rel_service(0, 0, -100, 0, 0, 0)
            if ret == True:
                CURRENT_STATE=STATE_MOVING_TO_PLACE
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
                
    def press_apply_button(self):
        Prepick_Pose_x=self._widget.prepick_x.toPlainText()
        Prepick_Pose_y=self._widget.prepick_y.toPlainText()
        Prepick_Pose_z=self._widget.prepick_z.toPlainText()
        Prepick_Pose_a=self._widget.prepick_a.toPlainText()
        Prepick_Pose_b=self._widget.prepick_b.toPlainText()
        Prepick_Pose_c=self._widget.prepick_c.toPlainText()
        print "updated Prepick Pose x:", Prepick_Pose_x, " y:", Prepick_Pose_y, " z:", Prepick_Pose_z, " a:", Prepick_Pose_a, " b:", Prepick_Pose_b, " c:", Prepick_Pose_c

        Preplace_Pose_x=self._widget.preplace_x.toPlainText()
        Preplace_Pose_y=self._widget.preplace_y.toPlainText()
        Preplace_Pose_z=self._widget.preplace_z.toPlainText()
        Preplace_Pose_a=self._widget.preplace_a.toPlainText()
        Preplace_Pose_b=self._widget.preplace_b.toPlainText()
        Preplace_Pose_c=self._widget.preplace_c.toPlainText()
        print "updated Preplace Pose x:", Preplace_Pose_x, " y:", Preplace_Pose_y, " z:", Preplace_Pose_z, " a:", Preplace_Pose_a, " b:", Preplace_Pose_b, " c:", Preplace_Pose_c
    
    def press_tare_button(self):
        global weight_empty,horiz_force_empty
        weight_empty=weight_read
        horiz_force_empty=horiz_force_read

    def press_tare_reset_button(self):
        global weight_empty,horiz_force_empty
        weight_empty = 0
        horiz_force_empty=0
    #################################################CALIBRE SELECTION
    def calibre_selected(self, index):
        global finger_type, weight_expected_min, weight_expected_max
        print 'Selected:',index
        finger_type = index
        if index == 0:
            print 'No gripper selected'
            self.desactivate_buttons()
            str_weight_expected = "[N/A, N/A]"
            self._widget.weight_limited.setText("N/A");
            self._widget.weight_label_expected_var.setText(str_weight_expected)
            pixmap = QtGui.QPixmap(PATH+"resource/images/fondo_huevera_0.png")
            self._widget.background_plate.setPixmap(pixmap)
            self._widget.Huevera2Obus1Button.hide()
            self._widget.Huevera2Obus2Button.hide()            
            self._widget.Huevera4Obus1Button.hide()
            self._widget.Huevera4Obus2Button.hide()
            self._widget.Huevera4Obus3Button.hide()
            self._widget.Huevera4Obus4Button.hide()
            self._widget.Huevera8Obus1Button.hide()
            self._widget.Huevera8Obus2Button.hide()
            self._widget.Huevera8Obus3Button.hide()
            self._widget.Huevera8Obus4Button.hide()
            self._widget.Huevera8Obus5Button.hide()
            self._widget.Huevera8Obus6Button.hide()
            self._widget.Huevera8Obus7Button.hide()
            self._widget.Huevera8Obus8Button.hide()            
            self._widget.Huevera16Obus1Button.hide()
            self._widget.Huevera16Obus2Button.hide()
            self._widget.Huevera16Obus3Button.hide()
            self._widget.Huevera16Obus4Button.hide()
            self._widget.Huevera16Obus5Button.hide()
            self._widget.Huevera16Obus6Button.hide()
            self._widget.Huevera16Obus7Button.hide()
            self._widget.Huevera16Obus8Button.hide()  
            self._widget.Huevera16Obus9Button.hide()
            self._widget.Huevera16Obus10Button.hide()
            self._widget.Huevera16Obus11Button.hide()
            self._widget.Huevera16Obus12Button.hide()
            self._widget.Huevera16Obus13Button.hide()
            self._widget.Huevera16Obus14Button.hide()
            self._widget.Huevera16Obus15Button.hide()
            self._widget.Huevera16Obus16Button.hide()              
        else:
            #TODO: check if the gripper is empty. If there is some load not allow to move autonomously
            self.activate_buttons()            
        if index == 1:            
            try:
                rospy.delete_param('robot_description')
            except KeyError:
                print "value not set"
            self.load_robot_description(index)
            weight_expected_min = 4
            weight_expected_max = 9
            str_weight_expected = "["+str(weight_expected_min)+ ", "+ str(weight_expected_max)+ "]"
            self._widget.weight_label_expected_var.setText(str_weight_expected)
            self._widget.weight_limited.setText("3");
            limit_cont_current_service=rospy.ServiceProxy(srv_limit_cont_current, set_float_value)
            limit_peak_current_service=rospy.ServiceProxy(srv_limit_peak_current, set_float_value)
            limit_cont_current_service(3)
            limit_peak_current_service(3)

            pixmap = QtGui.QPixmap(PATH+"resource/images/fondo_huevera_16.png")
            self._widget.background_plate.setPixmap(pixmap)
            self._widget.Huevera16Obus1Button.show()
            self._widget.Huevera16Obus2Button.show()
            self._widget.Huevera16Obus3Button.show()
            self._widget.Huevera16Obus4Button.show()
            self._widget.Huevera16Obus5Button.show()
            self._widget.Huevera16Obus6Button.show()
            self._widget.Huevera16Obus7Button.show()
            self._widget.Huevera16Obus8Button.show()  
            self._widget.Huevera16Obus9Button.show()
            self._widget.Huevera16Obus10Button.show()
            self._widget.Huevera16Obus11Button.show()
            self._widget.Huevera16Obus12Button.show()
            self._widget.Huevera16Obus13Button.show()
            self._widget.Huevera16Obus14Button.show()
            self._widget.Huevera16Obus15Button.show()
            self._widget.Huevera16Obus16Button.show()   
            self._widget.Huevera2Obus1Button.hide()
            self._widget.Huevera2Obus2Button.hide()
            self._widget.Huevera4Obus1Button.hide()
            self._widget.Huevera4Obus2Button.hide()
            self._widget.Huevera4Obus3Button.hide()
            self._widget.Huevera4Obus4Button.hide()            
            self._widget.Huevera8Obus1Button.hide()
            self._widget.Huevera8Obus2Button.hide()
            self._widget.Huevera8Obus3Button.hide()
            self._widget.Huevera8Obus4Button.hide()
            self._widget.Huevera8Obus5Button.hide()
            self._widget.Huevera8Obus6Button.hide()
            self._widget.Huevera8Obus7Button.hide()
            self._widget.Huevera8Obus8Button.hide()

        elif index == 2:
            try:
                rospy.delete_param('robot_description')
            except KeyError:
                print "value not set"
            self.load_robot_description(index+1)
            weight_expected_min = 13
            weight_expected_max = 18
            str_weight_expected = "["+str(weight_expected_min)+ ", "+ str(weight_expected_max)+ "]"
            self._widget.weight_label_expected_var.setText(str_weight_expected)

            self._widget.weight_limited.setText("4");
            limit_cont_current_service=rospy.ServiceProxy(srv_limit_cont_current, set_float_value)
            limit_peak_current_service=rospy.ServiceProxy(srv_limit_peak_current, set_float_value)
            limit_cont_current_service(4)
            limit_peak_current_service(4)


            pixmap = QtGui.QPixmap(PATH+"resource/images/fondo_huevera_8.png")
            self._widget.background_plate.setPixmap(pixmap)
            self._widget.Huevera2Obus1Button.hide()
            self._widget.Huevera2Obus2Button.hide()
            self._widget.Huevera4Obus1Button.hide()
            self._widget.Huevera4Obus2Button.hide()
            self._widget.Huevera4Obus3Button.hide()
            self._widget.Huevera4Obus4Button.hide()                        
            self._widget.Huevera8Obus1Button.show()
            self._widget.Huevera8Obus2Button.show()
            self._widget.Huevera8Obus3Button.show()
            self._widget.Huevera8Obus4Button.show()
            self._widget.Huevera8Obus5Button.show()
            self._widget.Huevera8Obus6Button.show()
            self._widget.Huevera8Obus7Button.show()
            self._widget.Huevera8Obus8Button.show()
            self._widget.Huevera16Obus1Button.hide()
            self._widget.Huevera16Obus2Button.hide()
            self._widget.Huevera16Obus3Button.hide()
            self._widget.Huevera16Obus4Button.hide()
            self._widget.Huevera16Obus5Button.hide()
            self._widget.Huevera16Obus6Button.hide()
            self._widget.Huevera16Obus7Button.hide()
            self._widget.Huevera16Obus8Button.hide()  
            self._widget.Huevera16Obus9Button.hide()
            self._widget.Huevera16Obus10Button.hide()
            self._widget.Huevera16Obus11Button.hide()
            self._widget.Huevera16Obus12Button.hide()
            self._widget.Huevera16Obus13Button.hide()
            self._widget.Huevera16Obus14Button.hide()
            self._widget.Huevera16Obus15Button.hide()
            self._widget.Huevera16Obus16Button.hide()   

        elif index == 3:
            try:
                rospy.delete_param('robot_description')
            except KeyError:
                print "value not set"
            self.load_robot_description(index+1)
            weight_expected_min = 18
            weight_expected_max = 46
            str_weight_expected = "["+str(weight_expected_min)+ ", "+ str(weight_expected_max)+ "]"
            self._widget.weight_label_expected_var.setText(str_weight_expected)

            self._widget.weight_limited.setText("7");
            limit_cont_current_service=rospy.ServiceProxy(srv_limit_cont_current, set_float_value)
            limit_peak_current_service=rospy.ServiceProxy(srv_limit_peak_current, set_float_value)
            limit_cont_current_service(5)
            limit_peak_current_service(7)

            pixmap = QtGui.QPixmap(PATH+"resource/images/fondo_huevera_4.png")
            self._widget.background_plate.setPixmap(pixmap)
            self._widget.Huevera4Obus1Button.show()
            self._widget.Huevera4Obus2Button.show()
            self._widget.Huevera4Obus3Button.show()
            self._widget.Huevera4Obus4Button.show()
            self._widget.Huevera2Obus1Button.hide()
            self._widget.Huevera2Obus2Button.hide()
            self._widget.Huevera8Obus1Button.hide()
            self._widget.Huevera8Obus2Button.hide()
            self._widget.Huevera8Obus3Button.hide()
            self._widget.Huevera8Obus4Button.hide()
            self._widget.Huevera8Obus5Button.hide()
            self._widget.Huevera8Obus6Button.hide()
            self._widget.Huevera8Obus7Button.hide()
            self._widget.Huevera8Obus8Button.hide()              
            self._widget.Huevera16Obus1Button.hide()
            self._widget.Huevera16Obus2Button.hide()
            self._widget.Huevera16Obus3Button.hide()
            self._widget.Huevera16Obus4Button.hide()
            self._widget.Huevera16Obus5Button.hide()
            self._widget.Huevera16Obus6Button.hide()
            self._widget.Huevera16Obus7Button.hide()
            self._widget.Huevera16Obus8Button.hide()  
            self._widget.Huevera16Obus9Button.hide()
            self._widget.Huevera16Obus10Button.hide()
            self._widget.Huevera16Obus11Button.hide()
            self._widget.Huevera16Obus12Button.hide()
            self._widget.Huevera16Obus13Button.hide()
            self._widget.Huevera16Obus14Button.hide()
            self._widget.Huevera16Obus15Button.hide()
            self._widget.Huevera16Obus16Button.hide() 

        elif index == 4:
            try:
                rospy.delete_param('robot_description')
            except KeyError:
                print "value not set"
            self.load_robot_description(index+1)
            weight_expected_min = 110
            weight_expected_max = 130
            str_weight_expected = "["+str(weight_expected_min)+ ", "+ str(weight_expected_max)+ "]"
            self._widget.weight_label_expected_var.setText(str_weight_expected)
            self._widget.weight_limited.setText("8");

            limit_cont_current_service=rospy.ServiceProxy(srv_limit_cont_current, set_float_value)
            limit_peak_current_service=rospy.ServiceProxy(srv_limit_peak_current, set_float_value)
            limit_cont_current_service(5)
            limit_peak_current_service(8)
            
            pixmap = QtGui.QPixmap(PATH+"resource/images/fondo_huevera_2.png")
            self._widget.background_plate.setPixmap(pixmap)
            self._widget.Huevera2Obus1Button.show()
            self._widget.Huevera2Obus2Button.show()
            self._widget.Huevera4Obus1Button.hide()
            self._widget.Huevera4Obus2Button.hide()
            self._widget.Huevera4Obus3Button.hide()
            self._widget.Huevera4Obus4Button.hide()
            self._widget.Huevera8Obus1Button.hide()
            self._widget.Huevera8Obus2Button.hide()
            self._widget.Huevera8Obus3Button.hide()
            self._widget.Huevera8Obus4Button.hide()
            self._widget.Huevera8Obus5Button.hide()
            self._widget.Huevera8Obus6Button.hide()
            self._widget.Huevera8Obus7Button.hide()
            self._widget.Huevera8Obus8Button.hide()              
            self._widget.Huevera16Obus1Button.hide()
            self._widget.Huevera16Obus2Button.hide()
            self._widget.Huevera16Obus3Button.hide()
            self._widget.Huevera16Obus4Button.hide()
            self._widget.Huevera16Obus5Button.hide()
            self._widget.Huevera16Obus6Button.hide()
            self._widget.Huevera16Obus7Button.hide()
            self._widget.Huevera16Obus8Button.hide()  
            self._widget.Huevera16Obus9Button.hide()
            self._widget.Huevera16Obus10Button.hide()
            self._widget.Huevera16Obus11Button.hide()
            self._widget.Huevera16Obus12Button.hide()
            self._widget.Huevera16Obus13Button.hide()
            self._widget.Huevera16Obus14Button.hide()
            self._widget.Huevera16Obus15Button.hide()
            self._widget.Huevera16Obus16Button.hide() 


    def load_robot_description(self, gripper_model):
        command_string = "rosparam load ~/kuka_catkin_ws/src/kuka_experimental/kuka_robot_bringup/robot/bin/kr120toolv%d.urdf /robot_description" % gripper_model
        os.system(command_string)
        
    def press_reset_external_pc_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nExternal PC is going to reset.\n Wait 10 sec and restart the GUI.', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == True:
            command_string = "ssh vulcano@192.168.1.10 reboot"
            os.system(command_string)
            
        
    def press_run_program_button(self):
        command_string = "rosnode kill /kuka_pad/joy; sleep 1; rosnode kill /kuka_pad/robotnik_trajectory_pad_node; sleep 1; rosnode kill /kuka_robot/kuka_cartesian_hardware_interface; sleep 1; roslaunch kuka_robot_bringup kuka_robot_bringup_standalone.launch &"
        os.system(command_string)
        
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass
    def sleep_loop(self,delay):
        loop = QtCore.QEventLoop()
        timer = QtCore.QTimer()
        timer.setInterval(delay*1000)
        timer.setSingleShot(True)
        timer.timeout.connect(loop.quit)
        timer.start()
        loop.exec_()
