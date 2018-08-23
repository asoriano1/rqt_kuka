#!/usr/bin/python
# -*- coding: utf-8 -*-
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
from python_qt_binding.QtWidgets import QWidget, QDialog, QFileDialog, QMessageBox, QTableWidgetItem, QApplication, QGroupBox, QCheckBox
from std_msgs.msg import Bool, Float64, Float32
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from robotnik_msgs.srv import home, set_odometry, set_CartesianEuler_pose, set_digital_output, set_float_value
from robotnik_msgs.msg import Cartesian_Euler_pose, RobotnikMotorsStatus, MotorStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from kuka_rsi_cartesian_hw_interface.srv import set_A1_A6


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

PATH="/home/suippes/kuka_catkin_ws/src/rqt_kuka/"

TOOL_HOMED=False
KUKA_AUT=False
finger_type=0
#gauges_failure=False
under_voltage_tool=False
first_time_enabled=False
#start_time_gauges=time.time()
angle_mode=True
origin_pick=0
tool_current=0
first_time_moving_kuka=False
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
srv_angle_mode='kuka_tool_finger_node/set_angle_mode'
srv_move_A1_A6='/kuka_robot/setKukaA1A6'
srv_deadman='kuka_tool_finger_node/set_deadMan_mode'
srv_rel_tool='/kuka_robot/setMoveRelTool'
srv_tare_gauges = '/tare_weight_gauges'

#topic names:
topic_cart_pose_kuka='/kuka_robot/cartesian_pos_kuka'
topic_kuka_moving='/kuka_robot/kuka_moving'
topic_tool_weight='/phidget_load/load_mean'
topic_current='/kuka_tool/robotnik_base_hw/current0'
topic_horiz_force='/phidget_load/vertical_force'
topic_motor_status='/kuka_tool/robotnik_base_hw/status'
topic_tool_state='/kuka_tool/joint_states'

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
Prepick_Pose_z=1542.38
Prepick_Pose_a_left=-18#15.2
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

#Current limits
current_limit_0 = 2
current_limit_1 = 3
current_limit_2 = 4
current_limit_cont = 5
current_limit_3 = 7
current_limit_4 = 8
current_limit_picked = 2
#Obus already placed
#Hueveras de 2
Obus_21=False
Obus_22=False

#Hueveras de 4
Obus_41=False
Obus_42=False
Obus_43=False
Obus_44=False

#Hueveras de 8
Obus_81=False
Obus_82=False
Obus_83=False
Obus_84=False
Obus_85=False
Obus_86=False
Obus_87=False
Obus_88=False

#Hueveras de 16
Obus_161=False
Obus_162=False
Obus_163=False
Obus_164=False
Obus_165=False
Obus_166=False
Obus_167=False
Obus_168=False
Obus_169=False
Obus_1610=False
Obus_1611=False
Obus_1612=False
Obus_1613=False
Obus_1614=False
Obus_1615=False
Obus_1616=False

#Pick Positions

#Obus already placed
#Hueveras de 2
Pick_Obus_21=False
Pick_Obus_22=False
Pick_Obus_23=False
Pick_Obus_24=False
#Hueveras de 4
Pick_Obus_41=False
Pick_Obus_42=False
Pick_Obus_43=False
Pick_Obus_44=False
Pick_Obus_45=False

#Hueveras de 8
Pick_Obus_81=False
Pick_Obus_82=False
Pick_Obus_83=False
Pick_Obus_84=False
Pick_Obus_85=False
Pick_Obus_86=False
Pick_Obus_87=False
Pick_Obus_88=False
Pick_Obus_89=False
Pick_Obus_810=False
Pick_Obus_811=False
Pick_Obus_812=False
Pick_Obus_813=False
Pick_Obus_814=False

#Hueveras de 16
Pick_Obus_161=False
Pick_Obus_162=False
Pick_Obus_163=False
Pick_Obus_164=False
Pick_Obus_165=False
Pick_Obus_166=False
Pick_Obus_167=False
Pick_Obus_168=False
Pick_Obus_169=False
Pick_Obus_1610=False
Pick_Obus_1611=False
Pick_Obus_1612=False
Pick_Obus_1613=False
Pick_Obus_1614=False
Pick_Obus_1615=False
Pick_Obus_1616=False
Pick_Obus_1617=False
Pick_Obus_1618=False
Pick_Obus_1619=False
Pick_Obus_1620=False

class RqtKuka(Plugin):
        
    do_callback_motor_status = QtCore.pyqtSignal(RobotnikMotorsStatus)
    do_callback_horiz_force = QtCore.pyqtSignal(Float64)
    do_callback_current = QtCore.pyqtSignal(Float32)
    do_callback_tool_weight = QtCore.pyqtSignal(Float64)
    do_callback_moving = QtCore.pyqtSignal(Bool)

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
        #self._widget.Home_Button.pressed.connect(self.press_homming_button)
        #self._widget.Pick_Left_Button.pressed.connect(self.press_pick_left_button)
        #self._widget.Pick_Right_Button.pressed.connect(self.press_pick_right_button)
        #self._widget.Pick1_left_Button.pressed.connect(self.press_pick1_left_button)
        #self._widget.Pick3_left_Button.pressed.connect(self.press_pick3_left_button)
        #self._widget.Pick2_right_Button.pressed.connect(self.press_pick2_right_button)
        #self._widget.Pick4_right_Button.pressed.connect(self.press_pick4_right_button)
        self._widget.Finger_Adjust_Button.pressed.connect(self.press_finger_adjust_button)
        self._widget.Tare_Button.pressed.connect(self.press_tare_button)
        self._widget.Tare_Reset_Button.pressed.connect(self.press_tare_reset_button)
        self._widget.Reset_Ext_Button.pressed.connect(self.press_reset_external_pc_button)
        self._widget.Run_Program_Button.pressed.connect(self.press_run_program_button)
        self._widget.Run_Program_Button.hide()
        self._widget.MoveToTable_Button.pressed.connect(self.press_homming_button)#self.press_move_to_rotation_table_button)
        
        
        self._widget.PickTest_Button.pressed.connect(self.press_picktest_button)
        self._widget.Gripper_Homing_Button.pressed.connect(self.press_tool_homming)
        self._widget.Led_On_Button.pressed.connect(self.press_led_on_button)
        self._widget.Led_Off_Button.pressed.connect(self.press_led_off_button)
        self._widget.Light_On_Button.pressed.connect(self.press_light_on_button)
        self._widget.Light_Off_Button.pressed.connect(self.press_light_off_button)
        self._widget.resetPositions_Button.pressed.connect(self.press_reset_positions_button)
        self._widget.resetPositions_Button_pick.pressed.connect(self.press_reset_positions_button_pick)
        self._widget.press_Button.pressed.connect(self.aut_press_tool)
        
        #Checkboxes of robot settings
        self._widget.deadMan_check.clicked.connect(self.deadMan_state_changed)
        self._widget.toolAngle_check.clicked.connect(self.toolAngle_state_changed)
        self._widget.toolOrientation_check.clicked.connect(self.toolOrientation_state_changed)
        
        self._widget.Led_On_Button.setStyleSheet("color: rgb(80, 170, 80)")
        self._widget.Led_Off_Button.setStyleSheet("color: rgb(170, 80, 80)")
        self._widget.Light_On_Button.setStyleSheet("color: rgb(80, 170, 80)")
        self._widget.Light_Off_Button.setStyleSheet("color: rgb(170, 80, 80)")
        
        pixmap = QtGui.QPixmap(PATH+"resource/images/fondo_huevera_0.png")
        self._widget.background_plate.setPixmap(pixmap)
        self._widget.background_plate_pick.setPixmap(pixmap)
        #self._widget.Home_Button.setEnabled(False)
        self._widget.Finger_Adjust_Button.setEnabled(False)
        self._widget.MoveToTable_Button.setEnabled(False)
        self._widget.weightProgressBar_2.setMinimum(0)
        self._widget.weightProgressBar_2.setMaximum(15)
        
        ##Obuses buttons PICK
        #Huevera 2PICK
        #Obus 1
        self._widget.PickBox2_1.clicked.connect(self.press_pick_obus2_1_button)
        self._widget.PickBox2_1.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png")
        self._widget.PickBox2_1.setMask(mask.mask())
        self._widget.PickBox2_1.setMouseTracking(True)       
        self._widget.PickBox2_1.installEventFilter(self)
        #Obus 2
        self._widget.PickBox2_2.clicked.connect(self.press_pick_obus2_2_button)
        self._widget.PickBox2_2.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png")
        self._widget.PickBox2_2.setMask(mask.mask())
        self._widget.PickBox2_2.setMouseTracking(True)       
        self._widget.PickBox2_2.installEventFilter(self)
        #Obus 3
        self._widget.PickBox2_3.clicked.connect(self.press_pick_obus2_3_button)
        self._widget.PickBox2_3.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png")
        self._widget.PickBox2_3.setMask(mask.mask())
        self._widget.PickBox2_3.setMouseTracking(True)       
        self._widget.PickBox2_3.installEventFilter(self)
        #Obus 4
        self._widget.PickBox2_4.clicked.connect(self.press_pick_obus2_4_button)
        self._widget.PickBox2_4.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png")
        self._widget.PickBox2_4.setMask(mask.mask())
        self._widget.PickBox2_4.setMouseTracking(True)       
        self._widget.PickBox2_4.installEventFilter(self)
        
        #Huevera 4PICK
        #Obus 1
        self._widget.PickBox4_1.clicked.connect(self.press_pick_obus4_1_button)
        self._widget.PickBox4_1.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.PickBox4_1.setMask(mask.mask())
        self._widget.PickBox4_1.setMouseTracking(True)       
        self._widget.PickBox4_1.installEventFilter(self)
        #Obus 2
        self._widget.PickBox4_2.clicked.connect(self.press_pick_obus4_2_button)
        self._widget.PickBox4_2.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.PickBox4_2.setMask(mask.mask())
        self._widget.PickBox4_2.setMouseTracking(True)       
        self._widget.PickBox4_2.installEventFilter(self)
        #Obus 3
        self._widget.PickBox4_3.clicked.connect(self.press_pick_obus4_3_button)
        self._widget.PickBox4_3.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.PickBox4_3.setMask(mask.mask())
        self._widget.PickBox4_3.setMouseTracking(True)       
        self._widget.PickBox4_3.installEventFilter(self)
        #Obus 4
        self._widget.PickBox4_4.clicked.connect(self.press_pick_obus4_4_button)
        self._widget.PickBox4_4.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101.png")
        self._widget.PickBox4_4.setMask(mask.mask())
        self._widget.PickBox4_4.setMouseTracking(True)       
        self._widget.PickBox4_4.installEventFilter(self)
        #Obus 5
        self._widget.PickBox4_5.clicked.connect(self.press_pick_obus4_5_button)
        self._widget.PickBox4_5.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101.png")
        self._widget.PickBox4_5.setMask(mask.mask())
        self._widget.PickBox4_5.setMouseTracking(True)       
        self._widget.PickBox4_5.installEventFilter(self)
        
        #Huevera 8PICK
        #obus1
        self._widget.PickBox8_1.clicked.connect(self.press_pick_obus8_1_button)
        self._widget.PickBox8_1.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickBox8_1.setMask(mask.mask())
        self._widget.PickBox8_1.setMouseTracking(True)       
        self._widget.PickBox8_1.installEventFilter(self)
        #obus2
        self._widget.PickBox8_2.clicked.connect(self.press_pick_obus8_2_button)
        self._widget.PickBox8_2.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickBox8_2.setMask(mask.mask())
        self._widget.PickBox8_2.setMouseTracking(True)       
        self._widget.PickBox8_2.installEventFilter(self)
        #obus3
        self._widget.PickBox8_3.clicked.connect(self.press_pick_obus8_3_button)
        self._widget.PickBox8_3.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickBox8_3.setMask(mask.mask())
        self._widget.PickBox8_3.setMouseTracking(True)       
        self._widget.PickBox8_3.installEventFilter(self)
        #obus4
        self._widget.PickBox8_4.clicked.connect(self.press_pick_obus8_4_button)
        self._widget.PickBox8_4.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickBox8_4.setMask(mask.mask())
        self._widget.PickBox8_4.setMouseTracking(True)       
        self._widget.PickBox8_4.installEventFilter(self)
        #obus5
        self._widget.PickBox8_5.clicked.connect(self.press_pick_obus8_5_button)
        self._widget.PickBox8_5.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickBox8_5.setMask(mask.mask())
        self._widget.PickBox8_5.setMouseTracking(True)       
        self._widget.PickBox8_5.installEventFilter(self)
        #obus6
        self._widget.PickBox8_6.clicked.connect(self.press_pick_obus8_6_button)
        self._widget.PickBox8_6.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickBox8_6.setMask(mask.mask())
        self._widget.PickBox8_6.setMouseTracking(True)       
        self._widget.PickBox8_6.installEventFilter(self)
        #obus7
        self._widget.PickBox8_7.clicked.connect(self.press_pick_obus8_7_button)
        self._widget.PickBox8_7.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickBox8_7.setMask(mask.mask())
        self._widget.PickBox8_7.setMouseTracking(True)       
        self._widget.PickBox8_7.installEventFilter(self)
        #obus8
        self._widget.PickBox8_8.clicked.connect(self.press_pick_obus8_8_button)
        self._widget.PickBox8_8.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickBox8_8.setMask(mask.mask())
        self._widget.PickBox8_8.setMouseTracking(True)       
        self._widget.PickBox8_8.installEventFilter(self)
        #obus9
        self._widget.PickBox8_9.clicked.connect(self.press_pick_obus8_9_button)
        self._widget.PickBox8_9.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickBox8_9.setMask(mask.mask())
        self._widget.PickBox8_9.setMouseTracking(True)       
        self._widget.PickBox8_9.installEventFilter(self)
        #obus9
        self._widget.PickBox8_10.clicked.connect(self.press_pick_obus8_10_button)
        self._widget.PickBox8_10.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickBox8_10.setMask(mask.mask())
        self._widget.PickBox8_10.setMouseTracking(True)       
        self._widget.PickBox8_10.installEventFilter(self)
        #obus11
        self._widget.PickBox8_11.clicked.connect(self.press_pick_obus8_11_button)
        self._widget.PickBox8_11.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickBox8_11.setMask(mask.mask())
        self._widget.PickBox8_11.setMouseTracking(True)       
        self._widget.PickBox8_11.installEventFilter(self)
        #obus12
        self._widget.PickBox8_12.clicked.connect(self.press_pick_obus8_12_button)
        self._widget.PickBox8_12.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickBox8_12.setMask(mask.mask())
        self._widget.PickBox8_12.setMouseTracking(True)       
        self._widget.PickBox8_12.installEventFilter(self)
        #obus13
        self._widget.PickBox8_13.clicked.connect(self.press_pick_obus8_13_button)
        self._widget.PickBox8_13.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickBox8_13.setMask(mask.mask())
        self._widget.PickBox8_13.setMouseTracking(True)       
        self._widget.PickBox8_13.installEventFilter(self)
        #obus14
        self._widget.PickBox8_14.clicked.connect(self.press_pick_obus8_13_button)
        self._widget.PickBox8_14.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickBox8_14.setMask(mask.mask())
        self._widget.PickBox8_14.setMouseTracking(True)       
        self._widget.PickBox8_14.installEventFilter(self)
        
        #Huevera 16PICK
        #obus1
        self._widget.PickBox16_1.clicked.connect(self.press_pick_obus16_1_button)
        self._widget.PickBox16_1.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickBox16_1.setMask(mask.mask())
        self._widget.PickBox16_1.setMouseTracking(True)       
        self._widget.PickBox16_1.installEventFilter(self)
        #obus2
        self._widget.PickBox16_2.clicked.connect(self.press_pick_obus16_2_button)
        self._widget.PickBox16_2.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickBox16_2.setMask(mask.mask())
        self._widget.PickBox16_2.setMouseTracking(True)       
        self._widget.PickBox16_2.installEventFilter(self)
        #obus3
        self._widget.PickBox16_3.clicked.connect(self.press_pick_obus16_3_button)
        self._widget.PickBox16_3.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickBox16_3.setMask(mask.mask())
        self._widget.PickBox16_3.setMouseTracking(True)       
        self._widget.PickBox16_3.installEventFilter(self)
        #obus4
        self._widget.PickBox16_4.clicked.connect(self.press_pick_obus16_4_button)
        self._widget.PickBox16_4.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickBox16_4.setMask(mask.mask())
        self._widget.PickBox16_4.setMouseTracking(True)       
        self._widget.PickBox16_4.installEventFilter(self)
        #obus5
        self._widget.PickBox16_5.clicked.connect(self.press_pick_obus16_5_button)
        self._widget.PickBox16_5.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickBox16_5.setMask(mask.mask())
        self._widget.PickBox16_5.setMouseTracking(True)       
        self._widget.PickBox16_5.installEventFilter(self)
        #obus6
        self._widget.PickBox16_6.clicked.connect(self.press_pick_obus16_6_button)
        self._widget.PickBox16_6.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickBox16_6.setMask(mask.mask())
        self._widget.PickBox16_6.setMouseTracking(True)       
        self._widget.PickBox16_6.installEventFilter(self)
        #obus7
        self._widget.PickBox16_7.clicked.connect(self.press_pick_obus16_7_button)
        self._widget.PickBox16_7.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickBox16_7.setMask(mask.mask())
        self._widget.PickBox16_7.setMouseTracking(True)       
        self._widget.PickBox16_7.installEventFilter(self)
        #obus8
        self._widget.PickBox16_8.clicked.connect(self.press_pick_obus16_8_button)
        self._widget.PickBox16_8.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickBox16_8.setMask(mask.mask())
        self._widget.PickBox16_8.setMouseTracking(True)       
        self._widget.PickBox16_8.installEventFilter(self)
        #obus9
        self._widget.PickBox16_9.clicked.connect(self.press_pick_obus16_9_button)
        self._widget.PickBox16_9.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickBox16_9.setMask(mask.mask())
        self._widget.PickBox16_9.setMouseTracking(True)       
        self._widget.PickBox16_9.installEventFilter(self)
        #obus10
        self._widget.PickBox16_10.clicked.connect(self.press_pick_obus16_10_button)
        self._widget.PickBox16_10.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickBox16_10.setMask(mask.mask())
        self._widget.PickBox16_10.setMouseTracking(True)       
        self._widget.PickBox16_10.installEventFilter(self)
        #obus11
        self._widget.PickBox16_11.clicked.connect(self.press_pick_obus16_11_button)
        self._widget.PickBox16_11.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickBox16_11.setMask(mask.mask())
        self._widget.PickBox16_11.setMouseTracking(True)       
        self._widget.PickBox16_11.installEventFilter(self)
        #obus12
        self._widget.PickBox16_12.clicked.connect(self.press_pick_obus16_12_button)
        self._widget.PickBox16_12.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickBox16_12.setMask(mask.mask())
        self._widget.PickBox16_12.setMouseTracking(True)       
        self._widget.PickBox16_12.installEventFilter(self)
        #obus13
        self._widget.PickBox16_13.clicked.connect(self.press_pick_obus16_13_button)
        self._widget.PickBox16_13.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickBox16_13.setMask(mask.mask())
        self._widget.PickBox16_13.setMouseTracking(True)       
        self._widget.PickBox16_13.installEventFilter(self)
        #obus14
        self._widget.PickBox16_14.clicked.connect(self.press_pick_obus16_14_button)
        self._widget.PickBox16_14.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickBox16_14.setMask(mask.mask())
        self._widget.PickBox16_14.setMouseTracking(True)       
        self._widget.PickBox16_14.installEventFilter(self)
        #obus15
        self._widget.PickBox16_15.clicked.connect(self.press_pick_obus16_15_button)
        self._widget.PickBox16_15.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickBox16_15.setMask(mask.mask())
        self._widget.PickBox16_15.setMouseTracking(True)       
        self._widget.PickBox16_15.installEventFilter(self)
        #obus16
        self._widget.PickBox16_16.clicked.connect(self.press_pick_obus16_16_button)
        self._widget.PickBox16_16.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickBox16_16.setMask(mask.mask())
        self._widget.PickBox16_16.setMouseTracking(True)       
        self._widget.PickBox16_16.installEventFilter(self)
        #obus17
        self._widget.PickBox16_17.clicked.connect(self.press_pick_obus16_17_button)
        self._widget.PickBox16_17.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickBox16_17.setMask(mask.mask())
        self._widget.PickBox16_17.setMouseTracking(True)       
        self._widget.PickBox16_17.installEventFilter(self)
        #obus18
        self._widget.PickBox16_18.clicked.connect(self.press_pick_obus16_18_button)
        self._widget.PickBox16_18.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickBox16_18.setMask(mask.mask())
        self._widget.PickBox16_18.setMouseTracking(True)       
        self._widget.PickBox16_18.installEventFilter(self)
        #obus19
        self._widget.PickBox16_19.clicked.connect(self.press_pick_obus16_19_button)
        self._widget.PickBox16_19.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickBox16_19.setMask(mask.mask())
        self._widget.PickBox16_19.setMouseTracking(True)       
        self._widget.PickBox16_19.installEventFilter(self)
        #obus20
        self._widget.PickBox16_20.clicked.connect(self.press_pick_obus16_20_button)
        self._widget.PickBox16_20.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickBox16_20.setMask(mask.mask())
        self._widget.PickBox16_20.setMouseTracking(True)       
        self._widget.PickBox16_20.installEventFilter(self)
        ##obuses buttons PLACE
        #Huevera_2
        h2o1posex=80
        h2o1posey=600
        #obus1     
        self._widget.Huevera2Obus1Button.setGeometry(h2o1posex,h2o1posey,111,41)
        self._widget.Huevera2Obus1Button.clicked.connect(self.press_obus2_1_button)
        self._widget.Huevera2Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo41x111.png"))
        self._widget.Huevera2Obus1Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png")
        self._widget.Huevera2Obus1Button.setMask(mask.mask())
        self._widget.Huevera2Obus1Button.setMouseTracking(True)       
        self._widget.Huevera2Obus1Button.installEventFilter(self)
        #obus2
        self._widget.Huevera2Obus2Button.setGeometry(h2o1posex,h2o1posey-70,111,41)
        self._widget.Huevera2Obus2Button.clicked.connect(self.press_obus2_2_button)
        self._widget.Huevera2Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo41x111.png"))
        self._widget.Huevera2Obus2Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png")
        self._widget.Huevera2Obus2Button.setMask(mask.mask())
        self._widget.Huevera2Obus2Button.setMouseTracking(True)       
        self._widget.Huevera2Obus2Button.installEventFilter(self)        
        
        #Huevera_4
        h4o1posex=70
        h4o1posey=640
        #obus1
        self._widget.Huevera4Obus1Button.setGeometry(h4o1posex,h4o1posey,102,37)
        self._widget.Huevera4Obus1Button.clicked.connect(self.press_obus4_1_button)
        self._widget.Huevera4Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"))
        self._widget.Huevera4Obus1Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.Huevera4Obus1Button.setMask(mask.mask())
        self._widget.Huevera4Obus1Button.setMouseTracking(True)       
        self._widget.Huevera4Obus1Button.installEventFilter(self)
        #obus2
        self._widget.Huevera4Obus2Button.setGeometry(h4o1posex,h4o1posey-50,102,37)
        self._widget.Huevera4Obus2Button.clicked.connect(self.press_obus4_2_button)
        self._widget.Huevera4Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"))
        self._widget.Huevera4Obus2Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.Huevera4Obus2Button.setMask(mask.mask())
        self._widget.Huevera4Obus2Button.setMouseTracking(True)       
        self._widget.Huevera4Obus2Button.installEventFilter(self)
        #obus3
        self._widget.Huevera4Obus3Button.setGeometry(h4o1posex,h4o1posey-102,102,37)
        self._widget.Huevera4Obus3Button.clicked.connect(self.press_obus4_3_button)
        self._widget.Huevera4Obus3Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"))
        self._widget.Huevera4Obus3Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.Huevera4Obus3Button.setMask(mask.mask())
        self._widget.Huevera4Obus3Button.setMouseTracking(True)       
        self._widget.Huevera4Obus3Button.installEventFilter(self)
        #obus4
        self._widget.Huevera4Obus4Button.setGeometry(h4o1posex,h4o1posey-152,102,37)
        self._widget.Huevera4Obus4Button.clicked.connect(self.press_obus4_4_button)
        self._widget.Huevera4Obus4Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"))
        self._widget.Huevera4Obus4Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.Huevera4Obus4Button.setMask(mask.mask())
        self._widget.Huevera4Obus4Button.setMouseTracking(True)       
        self._widget.Huevera4Obus4Button.installEventFilter(self)

        #Huevera_8
        h8o1posex=70
        h8o1posey=650
        #obus1
        self._widget.Huevera8Obus1Button.setGeometry(h8o1posex,h8o1posey,71,26)
        self._widget.Huevera8Obus1Button.clicked.connect(self.press_obus8_1_button)
        self._widget.Huevera8Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"))
        self._widget.Huevera8Obus1Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.Huevera8Obus1Button.setMask(mask.mask())
        self._widget.Huevera8Obus1Button.setMouseTracking(True)       
        self._widget.Huevera8Obus1Button.installEventFilter(self)
        #obus2
        self._widget.Huevera8Obus2Button.setGeometry(h8o1posex,h8o1posey-38,71,26)
        self._widget.Huevera8Obus2Button.clicked.connect(self.press_obus8_2_button)
        self._widget.Huevera8Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"))
        self._widget.Huevera8Obus2Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.Huevera8Obus2Button.setMask(mask.mask())
        self._widget.Huevera8Obus2Button.setMouseTracking(True)       
        self._widget.Huevera8Obus2Button.installEventFilter(self)
        #obus3
        self._widget.Huevera8Obus3Button.setGeometry(h8o1posex,h8o1posey-86,71,26)
        self._widget.Huevera8Obus3Button.clicked.connect(self.press_obus8_3_button)
        self._widget.Huevera8Obus3Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"))
        self._widget.Huevera8Obus3Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.Huevera8Obus3Button.setMask(mask.mask())
        self._widget.Huevera8Obus3Button.setMouseTracking(True)       
        self._widget.Huevera8Obus3Button.installEventFilter(self)
        #obus4
        self._widget.Huevera8Obus4Button.setGeometry(h8o1posex,h8o1posey-132,71,26)
        self._widget.Huevera8Obus4Button.clicked.connect(self.press_obus8_4_button)
        self._widget.Huevera8Obus4Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"))
        self._widget.Huevera8Obus4Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.Huevera8Obus4Button.setMask(mask.mask())
        self._widget.Huevera8Obus4Button.setMouseTracking(True)       
        self._widget.Huevera8Obus4Button.installEventFilter(self)
        #obus5
        self._widget.Huevera8Obus5Button.setGeometry(h8o1posex+51,h8o1posey-18,71,26)
        self._widget.Huevera8Obus5Button.clicked.connect(self.press_obus8_5_button)
        self._widget.Huevera8Obus5Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"))
        self._widget.Huevera8Obus5Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.Huevera8Obus5Button.setMask(mask.mask())
        self._widget.Huevera8Obus5Button.setMouseTracking(True)       
        self._widget.Huevera8Obus5Button.installEventFilter(self)
        #obus6
        self._widget.Huevera8Obus6Button.setGeometry(h8o1posex+51,h8o1posey-63,71,26)
        self._widget.Huevera8Obus6Button.clicked.connect(self.press_obus8_6_button)
        self._widget.Huevera8Obus6Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"))
        self._widget.Huevera8Obus6Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.Huevera8Obus6Button.setMask(mask.mask())
        self._widget.Huevera8Obus6Button.setMouseTracking(True)       
        self._widget.Huevera8Obus6Button.installEventFilter(self)
        #obus7
        self._widget.Huevera8Obus7Button.setGeometry(h8o1posex+51,h8o1posey-109,71,26)
        self._widget.Huevera8Obus7Button.clicked.connect(self.press_obus8_7_button)
        self._widget.Huevera8Obus7Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"))
        self._widget.Huevera8Obus7Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.Huevera8Obus7Button.setMask(mask.mask())
        self._widget.Huevera8Obus7Button.setMouseTracking(True)       
        self._widget.Huevera8Obus7Button.installEventFilter(self)
        #obus8
        self._widget.Huevera8Obus8Button.setGeometry(h8o1posex+51,h8o1posey-150,71,26)
        self._widget.Huevera8Obus8Button.clicked.connect(self.press_obus8_8_button)
        self._widget.Huevera8Obus8Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"))
        self._widget.Huevera8Obus8Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.Huevera8Obus8Button.setMask(mask.mask())
        self._widget.Huevera8Obus8Button.setMouseTracking(True)       
        self._widget.Huevera8Obus8Button.installEventFilter(self)
        
        #huevera16
        h16o1posex=70
        h16o1posey=650
        #obus1
        self._widget.Huevera16Obus1Button.setGeometry(h16o1posex,h16o1posey,51,19)
        self._widget.Huevera16Obus1Button.clicked.connect(self.press_obus16_1_button)
        self._widget.Huevera16Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus1Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus1Button.setMask(mask.mask())
        self._widget.Huevera16Obus1Button.setMouseTracking(True)       
        self._widget.Huevera16Obus1Button.installEventFilter(self)
        #obus2
        self._widget.Huevera16Obus2Button.setGeometry(h16o1posex,h16o1posey-22,51,19)
        self._widget.Huevera16Obus2Button.clicked.connect(self.press_obus16_2_button)
        self._widget.Huevera16Obus2Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus2Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus2Button.setMask(mask.mask())
        self._widget.Huevera16Obus2Button.setMouseTracking(True)       
        self._widget.Huevera16Obus2Button.installEventFilter(self)
        #obus3
        self._widget.Huevera16Obus3Button.setGeometry(h16o1posex,h16o1posey-42,51,19)
        self._widget.Huevera16Obus3Button.clicked.connect(self.press_obus16_3_button)
        self._widget.Huevera16Obus3Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus3Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus3Button.setMask(mask.mask())
        self._widget.Huevera16Obus3Button.setMouseTracking(True)       
        self._widget.Huevera16Obus3Button.installEventFilter(self)
        #obus4
        self._widget.Huevera16Obus4Button.setGeometry(h16o1posex,h16o1posey-63,51,19)
        self._widget.Huevera16Obus4Button.clicked.connect(self.press_obus16_4_button)
        self._widget.Huevera16Obus4Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus4Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus4Button.setMask(mask.mask())
        self._widget.Huevera16Obus4Button.setMouseTracking(True)       
        self._widget.Huevera16Obus4Button.installEventFilter(self)
        #obus5
        self._widget.Huevera16Obus5Button.setGeometry(h16o1posex,h16o1posey-86,51,19)
        self._widget.Huevera16Obus5Button.clicked.connect(self.press_obus16_5_button)
        self._widget.Huevera16Obus5Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus5Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus5Button.setMask(mask.mask())
        self._widget.Huevera16Obus5Button.setMouseTracking(True)       
        self._widget.Huevera16Obus5Button.installEventFilter(self)
        #obus6
        self._widget.Huevera16Obus6Button.setGeometry(h16o1posex,h16o1posey-109,51,19)
        self._widget.Huevera16Obus6Button.clicked.connect(self.press_obus16_6_button)
        self._widget.Huevera16Obus6Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus6Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus6Button.setMask(mask.mask())
        self._widget.Huevera16Obus6Button.setMouseTracking(True)       
        self._widget.Huevera16Obus6Button.installEventFilter(self)
        #obus7
        self._widget.Huevera16Obus7Button.setGeometry(h16o1posex,h16o1posey-132,51,19)
        self._widget.Huevera16Obus7Button.clicked.connect(self.press_obus16_7_button)
        self._widget.Huevera16Obus7Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus7Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus7Button.setMask(mask.mask())
        self._widget.Huevera16Obus7Button.setMouseTracking(True)       
        self._widget.Huevera16Obus7Button.installEventFilter(self)
        #obus8
        self._widget.Huevera16Obus8Button.setGeometry(h16o1posex,h16o1posey-152,51,19)
        self._widget.Huevera16Obus8Button.clicked.connect(self.press_obus16_8_button)
        self._widget.Huevera16Obus8Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.Huevera16Obus8Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.Huevera16Obus8Button.setMask(mask.mask())
        self._widget.Huevera16Obus8Button.setMouseTracking(True)       
        self._widget.Huevera16Obus8Button.installEventFilter(self)
        #obus9
        self._widget.Huevera16Obus9Button.setGeometry(h16o1posex+74,h16o1posey,51,19)
        self._widget.Huevera16Obus9Button.clicked.connect(self.press_obus16_9_button)
        self._widget.Huevera16Obus9Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus9Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus9Button.setMask(mask.mask())
        self._widget.Huevera16Obus9Button.setMouseTracking(True)       
        self._widget.Huevera16Obus9Button.installEventFilter(self)
        #obus10
        self._widget.Huevera16Obus10Button.setGeometry(h16o1posex+74,h16o1posey-22,51,19)
        self._widget.Huevera16Obus10Button.clicked.connect(self.press_obus16_10_button)
        self._widget.Huevera16Obus10Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus10Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus10Button.setMask(mask.mask())
        self._widget.Huevera16Obus10Button.setMouseTracking(True)       
        self._widget.Huevera16Obus10Button.installEventFilter(self)
        #obus11
        self._widget.Huevera16Obus11Button.setGeometry(h16o1posex+74,h16o1posey-42,51,19)
        self._widget.Huevera16Obus11Button.clicked.connect(self.press_obus16_11_button)
        self._widget.Huevera16Obus11Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus11Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus11Button.setMask(mask.mask())
        self._widget.Huevera16Obus11Button.setMouseTracking(True)       
        self._widget.Huevera16Obus11Button.installEventFilter(self)
        #obus12
        self._widget.Huevera16Obus12Button.setGeometry(h16o1posex+74,h16o1posey-63,51,19)
        self._widget.Huevera16Obus12Button.clicked.connect(self.press_obus16_12_button)
        self._widget.Huevera16Obus12Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus12Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus12Button.setMask(mask.mask())
        self._widget.Huevera16Obus12Button.setMouseTracking(True)       
        self._widget.Huevera16Obus12Button.installEventFilter(self)
        #obus13
        self._widget.Huevera16Obus13Button.setGeometry(h16o1posex+74,h16o1posey-86,51,19)
        self._widget.Huevera16Obus13Button.clicked.connect(self.press_obus16_13_button)
        self._widget.Huevera16Obus13Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus13Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus13Button.setMask(mask.mask())
        self._widget.Huevera16Obus13Button.setMouseTracking(True)       
        self._widget.Huevera16Obus13Button.installEventFilter(self)
        #obus14
        self._widget.Huevera16Obus14Button.setGeometry(h16o1posex+74,h16o1posey-109,51,19)
        self._widget.Huevera16Obus14Button.clicked.connect(self.press_obus16_14_button)
        self._widget.Huevera16Obus14Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus14Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus14Button.setMask(mask.mask())
        self._widget.Huevera16Obus14Button.setMouseTracking(True)       
        self._widget.Huevera16Obus14Button.installEventFilter(self)
        #obus15
        self._widget.Huevera16Obus15Button.setGeometry(h16o1posex+74,h16o1posey-132,51,19)
        self._widget.Huevera16Obus15Button.clicked.connect(self.press_obus16_15_button)
        self._widget.Huevera16Obus15Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus15Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus15Button.setMask(mask.mask())
        self._widget.Huevera16Obus15Button.setMouseTracking(True)       
        self._widget.Huevera16Obus15Button.installEventFilter(self)
        #obus16
        self._widget.Huevera16Obus16Button.setGeometry(h16o1posex+74,h16o1posey-152,51,19)
        self._widget.Huevera16Obus16Button.clicked.connect(self.press_obus16_16_button)
        self._widget.Huevera16Obus16Button.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.Huevera16Obus16Button.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.Huevera16Obus16Button.setMask(mask.mask())
        self._widget.Huevera16Obus16Button.setMouseTracking(True)       
        self._widget.Huevera16Obus16Button.installEventFilter(self)
        
                
        
        #subscriber to robot state
        self.sub_robot_moving = rospy.Subscriber(topic_kuka_moving, Bool, self.callback_moving)
        #self.do_callback_moving.connect(self.callback_moving)

        #subscriber to robot pose
        self.sub_robot_pose = rospy.Subscriber(topic_cart_pose_kuka, Cartesian_Euler_pose, self.callback_robot_pose)     

        #subscriber to tool weight detected
        self.sub_tool_weight = rospy.Subscriber(topic_tool_weight, Float64, self.callback_tool_weight2)
        self.do_callback_tool_weight.connect(self.callback_tool_weight)     

        #subscriber to tool current
        self.sub_tool_current = rospy.Subscriber(topic_current, Float32, self.callback_current2)
        self.do_callback_current.connect(self.callback_current) 
                
        #subscriber to horiz force
        self.sub_tool_force = rospy.Subscriber(topic_horiz_force, Float64, self.callback_horiz_force2)
        self.do_callback_horiz_force.connect(self.callback_horiz_force)
        
        #subscriber to motor status of the tool
        self.sub_tool_status = rospy.Subscriber(topic_motor_status, RobotnikMotorsStatus, self.callback_motor_status2)
        self.do_callback_motor_status.connect(self.callback_motor_status)
        
        #subscriber to tool state
        self.sub_tool_state = rospy.Subscriber(topic_tool_state, JointState,  self.callback_tool_state)
        
        
        #Robot settings initialization
        #Default: deadman activated
        try:
                deadman_service=rospy.ServiceProxy(srv_deadman, SetBool)
                ret = deadman_service(True)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        #Default: angle activated
        try:
                angle_mode_service=rospy.ServiceProxy(srv_angle_mode, SetBool)
                ret = angle_mode_service(True)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        #Default: tool orientation reference deactivated
        try:
                toolOrientation_service=rospy.ServiceProxy(srv_rel_tool, SetBool)
                ret = toolOrientation_service(False)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        
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
    #filtro para detectar el raton y ponerlo verde si está el cursor encima o dejarlo blanco si no
    def eventFilter(self, object, event):
		if not KUKA_AUT :
			#huevera 16
			if finger_type == 1 :
				if event.type() == QtCore.QEvent.HoverEnter:
					#obuses hacia abajo (los de arriba [1-8])
					if (object == self._widget.Huevera16Obus1Button or object == self._widget.Huevera16Obus2Button or 
                                        object == self._widget.Huevera16Obus3Button or object == self._widget.Huevera16Obus4Button or object == self._widget.Huevera16Obus5Button or 
                                        object == self._widget.Huevera16Obus6Button or object == self._widget.Huevera16Obus7Button or object == self._widget.Huevera16Obus8Button or object ==self._widget.PickBox16_1
                                        or object ==self._widget.PickBox16_2 or object ==self._widget.PickBox16_3 or object ==self._widget.PickBox16_4 or object ==self._widget.PickBox16_5 or object ==self._widget.PickBox16_6  or object ==self._widget.PickBox16_7 
                                        or object ==self._widget.PickBox16_8 or object ==self._widget.PickBox16_9 or object ==self._widget.PickBox16_10 ) :
                                                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51P.png"))
					#obuses hacia arriba
					else:
                                                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51P.png"))
				if event.type() == QtCore.QEvent.HoverLeave:
					#obuses hacia abajo (los de arriba [1-8])
					if (object == self._widget.Huevera16Obus1Button or object == self._widget.Huevera16Obus2Button or
                                         object == self._widget.Huevera16Obus3Button or object == self._widget.Huevera16Obus4Button or object == self._widget.Huevera16Obus5Button or
                                          object == self._widget.Huevera16Obus6Button or object == self._widget.Huevera16Obus7Button or object == self._widget.Huevera16Obus8Button or object ==self._widget.PickBox16_1
                                        or object ==self._widget.PickBox16_2 or object ==self._widget.PickBox16_3 or object ==self._widget.PickBox16_4 or object ==self._widget.PickBox16_5 or object ==self._widget.PickBox16_6  or object ==self._widget.PickBox16_7 
                                        or object ==self._widget.PickBox16_8 or object ==self._widget.PickBox16_9 or object ==self._widget.PickBox16_10) :
                                                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
					#obuses hacia arriba
					else:
                                                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))  
			
			#huevera 8
			if finger_type == 2 :
				if event.type() == QtCore.QEvent.HoverEnter:
					#obuses hacia abajo (los de arriba [1-4])
					if (object == self._widget.Huevera8Obus1Button or object == self._widget.Huevera8Obus2Button or object == self._widget.Huevera8Obus3Button or
                                         object == self._widget.Huevera8Obus4Button or object == self._widget.PickBox8_1 or object == self._widget.PickBox8_2 or object == self._widget.PickBox8_3 or object == self._widget.PickBox8_4 
                                         or object == self._widget.PickBox8_5 or object == self._widget.PickBox8_6 or object == self._widget.PickBox8_7):
						object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba26x71P.png"))
					#obuses hacia arriba
					else:
						object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo26x71P.png"))
				if event.type() == QtCore.QEvent.HoverLeave:
					#obuses hacia abajo (los de arriba [1-4])
					if (object == self._widget.Huevera8Obus1Button or object == self._widget.Huevera8Obus2Button or object == self._widget.Huevera8Obus3Button or
                                         object == self._widget.Huevera8Obus4Button or object == self._widget.PickBox8_1 or object == self._widget.PickBox8_2 or object == self._widget.PickBox8_3 or object == self._widget.PickBox8_4 
                                         or object == self._widget.PickBox8_5 or object == self._widget.PickBox8_6 or object == self._widget.PickBox8_7) :
						object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"))
					#obuses hacia arriba
					else:
						object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"))        
			#huevera 4
			if finger_type == 3 :
				if event.type() == QtCore.QEvent.HoverEnter:
					object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo37x101P.png"))
				if event.type() == QtCore.QEvent.HoverLeave:
					object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"))
			#huevera 2
			if finger_type == 4 :
				if event.type() == QtCore.QEvent.HoverEnter:            
					object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo41x111P.png"))
				if event.type() == QtCore.QEvent.HoverLeave:
					object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo41x111.png"))
                                        
			
		return False
        

    def desactivate_buttons(self):
        #self._widget.Home_Button.setEnabled(False)
        self._widget.PickTest_Button.setEnabled(False)
        self._widget.Gripper_Homing_Button.setEnabled(False)
        #self._widget.Pick_Right_Button.setEnabled(False)
        #self._widget.Pick_Left_Button.setEnabled(False)
        self._widget.MoveToTable_Button.setEnabled(False)
        #self._widget.Place_Right_Button.setEnabled(False)
        #self._widget.Place_Left_Button.setEnabled(False)
        self._widget.resetPositions_Button.setEnabled(False)
        self._widget.resetPositions_Button_pick.setEnabled(False)
        self._widget.calibre_comboBox.setEnabled(False)
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
        for i in range(1, 21):
				name_method='PickBox16'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.setEnabled(False)
        for i in range(1, 15):
				name_method='PickBox8'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.setEnabled(False)
        for i in range(1, 6):
				name_method='PickBox4'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.setEnabled(False)
        for i in range(1, 5):
				name_method='PickBox2'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.setEnabled(False)
        
    def activate_buttons(self):
        #self._widget.Home_Button.setEnabled(True)
        self._widget.PickTest_Button.setEnabled(True)
        self._widget.Gripper_Homing_Button.setEnabled(True)
        #self._widget.Pick_Right_Button.setEnabled(True)
        #self._widget.Pick_Left_Button.setEnabled(True)
        self._widget.Finger_Adjust_Button.setEnabled(True)
        self._widget.MoveToTable_Button.setEnabled(True)
        #self._widget.Place_Right_Button.setEnabled(True)
        #self._widget.Place_Left_Button.setEnabled(True)
        self._widget.resetPositions_Button.setEnabled(True)
        self._widget.resetPositions_Button_pick.setEnabled(True)
        self._widget.calibre_comboBox.setEnabled(True)
        for i in range(1, 21):
				name_method='PickBox16'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.setEnabled(True)
        for i in range(1, 15):
				name_method='PickBox8'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.setEnabled(True)
        for i in range(1, 6):
				name_method='PickBox4'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.setEnabled(True)
        for i in range(1, 5):
				name_method='PickBox2'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.setEnabled(True)
        
        if(origin_pick==0):	
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
        elif((finger_type==3 or finger_type==4) and (origin_pick==2 or origin_pick==4)):
                self._widget.Huevera4Obus3Button.setEnabled(True)
                self._widget.Huevera4Obus4Button.setEnabled(True)
                self._widget.Huevera2Obus2Button.setEnabled(True)
        elif((finger_type==3 or finger_type==4) and (origin_pick==1 or origin_pick==3)):
                self._widget.Huevera4Obus1Button.setEnabled(True)
                self._widget.Huevera4Obus2Button.setEnabled(True)
                self._widget.Huevera2Obus1Button.setEnabled(True)
                
        elif(origin_pick==2 or origin_pick==3):
			self._widget.Huevera16Obus9Button.setEnabled(True)
			self._widget.Huevera16Obus10Button.setEnabled(True)
			self._widget.Huevera16Obus11Button.setEnabled(True)
			self._widget.Huevera16Obus12Button.setEnabled(True)
			self._widget.Huevera16Obus5Button.setEnabled(True)
			self._widget.Huevera16Obus6Button.setEnabled(True)
			self._widget.Huevera16Obus7Button.setEnabled(True)
			self._widget.Huevera16Obus8Button.setEnabled(True)
                        self._widget.Huevera8Obus3Button.setEnabled(True)
                        self._widget.Huevera8Obus4Button.setEnabled(True)
                        self._widget.Huevera8Obus5Button.setEnabled(True)
                        self._widget.Huevera8Obus6Button.setEnabled(True)
        elif(origin_pick==1 or origin_pick==4):
			self._widget.Huevera16Obus13Button.setEnabled(True)
			self._widget.Huevera16Obus14Button.setEnabled(True)
			self._widget.Huevera16Obus15Button.setEnabled(True)
			self._widget.Huevera16Obus16Button.setEnabled(True)
			self._widget.Huevera16Obus1Button.setEnabled(True)
			self._widget.Huevera16Obus2Button.setEnabled(True)
			self._widget.Huevera16Obus3Button.setEnabled(True)
			self._widget.Huevera16Obus4Button.setEnabled(True)
                        self._widget.Huevera8Obus1Button.setEnabled(True)
                        self._widget.Huevera8Obus2Button.setEnabled(True)
                        self._widget.Huevera8Obus7Button.setEnabled(True)
                        self._widget.Huevera8Obus8Button.setEnabled(True)
        ##PLACE
        for i in range(1,9):
                name='Obus_16'+str(i)
                if(globals()[name]==True):
                        icon=QtGui.QIcon()
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                        name_method='Huevera16Obus'+str(i)+'Button'
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        test_method.setIcon(icon)
        for i in range(9,17):
                name='Obus_16'+str(i)
                if(globals()[name]==True):
                        icon=QtGui.QIcon()
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                        name_method='Huevera16Obus'+str(i)+'Button'
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        test_method.setIcon(icon)
        for i in range(1,5):
                name='Obus_8'+str(i)
                if(globals()[name]==True):
                        icon=QtGui.QIcon()
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
                        name_method='Huevera8Obus'+str(i)+'Button'
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        test_method.setIcon(icon)
        for i in range(5,9):
                name='Obus_8'+str(i)
                if(globals()[name]==True):
                        icon=QtGui.QIcon()
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
                        name_method='Huevera8Obus'+str(i)+'Button'
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        test_method.setIcon(icon)
        for i in range(1,5):
                name='Obus_4'+str(i)
                if(globals()[name]==True):
                        icon=QtGui.QIcon()
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
                        name_method='Huevera4Obus'+str(i)+'Button'
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        test_method.setIcon(icon)
        for i in range(1,3):
                name='Obus_2'+str(i)
                if(globals()[name]==True):
                        icon=QtGui.QIcon()
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
                        name_method='Huevera2Obus'+str(i)+'Button'
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        test_method.setIcon(icon)
        ##PICK
        for i in range(1,11):
                name='Pick_Obus_16'+str(i)
                if(globals()[name]==True):
                        icon=QtGui.QIcon()
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                        name_method='PickBox16_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        test_method.setIcon(icon)
        for i in range(11,21):
                name='Pick_Obus_16'+str(i)
                if(globals()[name]==True):
                        icon=QtGui.QIcon()
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                        name_method='PickBox16_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        test_method.setIcon(icon)
        for i in range(1,8):
                name='Pick_Obus_8'+str(i)
                if(globals()[name]==True):
                        icon=QtGui.QIcon()
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
                        name_method='PickBox8_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        test_method.setIcon(icon)
        for i in range(8,15):
                name='Pick_Obus_8'+str(i)
                if(globals()[name]==True):
                        icon=QtGui.QIcon()
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
                        name_method='PickBox8_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        test_method.setIcon(icon)
        for i in range(1,6):
                name='Pick_Obus_4'+str(i)
                if(globals()[name]==True):
                        icon=QtGui.QIcon()
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
                        name_method='PickBox4_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        test_method.setIcon(icon)
        for i in range(1,5):
                name='Pick_Obus_2'+str(i)
                if(globals()[name]==True):
                        icon=QtGui.QIcon()
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
                        name_method='PickBox2_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        test_method.setIcon(icon)
        
        
				
				
    def press_reset_positions_button(self):
		global Obus_21, Obus_22, Obus_41, Obus_42, Obus_43, Obus_44, Obus_81, Obus_82, Obus_83, Obus_84, Obus_85, Obus_86, Obus_87, Obus_88,Obus_161, Obus_162, Obus_163, Obus_164, Obus_165, Obus_166, Obus_167, Obus_168,Obus_169, Obus_1610, Obus_1611, Obus_1612, Obus_1613, Obus_1614, Obus_1615, Obus_1616
                #Hueveras de 2
		Obus_21=False
		Obus_22=False

		#Hueveras de 4
		Obus_41=False
		Obus_42=False
		Obus_43=False
		Obus_44=False

		#Hueveras de 8
		Obus_81=False
		Obus_82=False
		Obus_83=False
		Obus_84=False
		Obus_85=False
		Obus_86=False
		Obus_87=False
		Obus_88=False

		#Hueveras de 16
		Obus_161=False
		Obus_162=False
		Obus_163=False
		Obus_164=False
		Obus_165=False
		Obus_166=False
		Obus_167=False
		Obus_168=False
		Obus_169=False
		Obus_1610=False
		Obus_1611=False
		Obus_1612=False
		Obus_1613=False
		Obus_1614=False
		Obus_1615=False
		Obus_1616=False
		for i in range(1,9):
				icon=QtGui.QIcon();
				icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
				name_method='Huevera16Obus'+str(i)+'Button'
				test_method=getattr(self._widget, name_method)
				test_method.setIcon(icon)
                                test_method.installEventFilter(self)
		for i in range(9,17):
				icon=QtGui.QIcon();
				icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
				name_method='Huevera16Obus'+str(i)+'Button'
				test_method=getattr(self._widget, name_method)
				test_method.setIcon(icon)
                                test_method.installEventFilter(self)
                for i in range(1,5):
                                icon=QtGui.QIcon();
                                icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"), QtGui.QIcon.Disabled)
                                name_method='Huevera8Obus'+str(i)+'Button'
                                test_method=getattr(self._widget, name_method)
                                test_method.setIcon(icon)
                                test_method.installEventFilter(self)
                for i in range(5,9):
                                icon=QtGui.QIcon();
                                icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"), QtGui.QIcon.Disabled)
                                name_method='Huevera8Obus'+str(i)+'Button'
                                test_method=getattr(self._widget, name_method)
                                test_method.setIcon(icon)
                                test_method.installEventFilter(self)
                for i in range(1,5):
                                icon=QtGui.QIcon();
                                icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"), QtGui.QIcon.Disabled)
                                name_method='Huevera4Obus'+str(i)+'Button'
                                test_method=getattr(self._widget, name_method)
                                test_method.setIcon(icon)
                                test_method.installEventFilter(self)
				
                for i in range(1,3):
                                icon=QtGui.QIcon();
                                icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png"), QtGui.QIcon.Disabled)
                                name_method='Huevera2Obus'+str(i)+'Button'
                                test_method=getattr(self._widget, name_method)
                                test_method.setIcon(icon)
                                test_method.installEventFilter(self)
                
		
		
    def press_reset_positions_button_pick(self):
		global Pick_Obus_21, Pick_Obus_22, Pick_Obus_23, Pick_Obus_24, Pick_Obus_41, Pick_Obus_42, Pick_Obus_43, Pick_Obus_44
                global Pick_Obus_81, Pick_Obus_82, Pick_Obus_83, Pick_Obus_84, Pick_Obus_85, Pick_Obus_86, Pick_Obus_87, Pick_Obus_88,Pick_Obus_89, Pick_Obus_810, Pick_Obus_811, Pick_Obus_812, Pick_Obus_813, Pick_Obus_814
                global Pick_Obus_161, Pick_Obus_162, Pick_Obus_163, Pick_Obus_164, Pick_Obus_165, Pick_Obus_166, Pick_Obus_167, Pick_Obus_168,Pick_Obus_169, Pick_Obus_1610
                global Pick_Obus_1611, Pick_Obus_1612, Pick_Obus_1613, Pick_Obus_1614, Pick_Obus_1615, Pick_Obus_1616, Pick_Obus_1617, Pick_Obus_1618, Pick_Obus_1619, Pick_Obus_1620
                #Hueveras de 2
		Pick_Obus_21=False
		Pick_Obus_22=False
                Pick_Obus_23=False
		Pick_Obus_24=False

		#Hueveras de 4
		Pick_Obus_41=False
		Pick_Obus_42=False
		Pick_Obus_43=False
		Pick_Obus_44=False
                Pick_Obus_45=False

		#Hueveras de 8
		Pick_Obus_81=False
		Pick_Obus_82=False
		Pick_Obus_83=False
		Pick_Obus_84=False
		Pick_Obus_85=False
		Pick_Obus_86=False
		Pick_Obus_87=False
		Pick_Obus_88=False
                Pick_Obus_89=False
		Pick_Obus_810=False
		Pick_Obus_811=False
		Pick_Obus_812=False
		Pick_Obus_813=False
                Pick_Obus_814=False

		#Hueveras de 16
		Pick_Obus_161=False
		Pick_Obus_162=False
		Pick_Obus_163=False
		Pick_Obus_164=False
		Pick_Obus_165=False
		Pick_Obus_166=False
		Pick_Obus_167=False
		Pick_Obus_168=False
		Pick_Obus_169=False
		Pick_Obus_1610=False
		Pick_Obus_1611=False
		Pick_Obus_1612=False
		Pick_Obus_1613=False
		Pick_Obus_1614=False
		Pick_Obus_1615=False
		Pick_Obus_1616=False
                Pick_Obus_1617=False
		Pick_Obus_1618=False
		Pick_Obus_1619=False
		Pick_Obus_1620=False

                for i in range(1,11):
                        name='Pick_Obus_16'+str(i)
                        icon=QtGui.QIcon();
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                        name_method='PickBox16_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.setIcon(icon)
                        test_method.installEventFilter(self)
                for i in range(11,21):
                        name='Pick_Obus_16'+str(i)
                        icon=QtGui.QIcon();
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                        name_method='PickBox16_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.setIcon(icon)
                        test_method.installEventFilter(self)
                for i in range(1,8):
                        name='Pick_Obus_8'+str(i)
                        icon=QtGui.QIcon();
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"), QtGui.QIcon.Disabled)
                        name_method='PickBox8_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.setIcon(icon)
                        test_method.installEventFilter(self)
                for i in range(8,15):
                        name='Pick_Obus_8'+str(i)
                        icon=QtGui.QIcon();
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"), QtGui.QIcon.Disabled)
                        name_method='PickBox8_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.setIcon(icon)
                        test_method.installEventFilter(self)
                for i in range(1,6):
                        name='Pick_Obus_4'+str(i)
                        icon=QtGui.QIcon();
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"), QtGui.QIcon.Disabled)
                        name_method='PickBox4_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.setIcon(icon)
                        test_method.installEventFilter(self)
                for i in range(1,5):
                        name='Pick_Obus_2'+str(i)
                        icon=QtGui.QIcon();
                        icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png"), QtGui.QIcon.Disabled)
                        name_method='PickBox2_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.setIcon(icon)
                        test_method.installEventFilter(self)
        
    def callback_moving(self, data):
        global KUKA_AUT, first_time_moving_kuka
        #print 'CB:moving_received:',data.data
        if data.data == True:
            KUKA_AUT=True
            if first_time_moving_kuka:
                    self._widget.mode_label.setText("AUTOMATIC")
                    self.desactivate_buttons()
                    first_time_moving_kuka = False
            #selt._widget.mode_label.setStyleSheet(\ncolor: rgb(255, 0, 0))
                            
        else:
            KUKA_AUT=False
            if first_time_moving_kuka==False:
                self._widget.mode_label.setText("MANUAL")
                self.activate_buttons()
                first_time_moving_kuka = True

    def callback_moving2(self,data):
            self.do_callback_moving.emit(data)
            
    def callback_motor_status(self,data):

		global under_voltage_tool, first_time_enabled, weight_empty, weight_read
		motor1=data.motor_status[1]
		driveflags_1=numpy.array(map(int,motor1.driveflags))
		under_voltage_1=driveflags_1[12]
		if(under_voltage_1==1):
			under_voltage_tool=True
			pixmap = QtGui.QPixmap(PATH+"resource/images/pinza_roja_peq2.png")
			self._widget.under_voltage_tool.setPixmap(pixmap)
                        #print 'undervoltage'
		else:
			under_voltage_tool=False
			pixmap = QtGui.QPixmap(PATH+"resource/images/pinza_verde_peq2.png")
			self._widget.under_voltage_tool.setPixmap(pixmap)
		if(motor1.status=="OPERATION_ENABLED" and first_time_enabled):
			#if(weight_read-weight_empty<-10):
			first_time_enabled=False
                        #print 'Warninng of weight should be here'				
			ret = QMessageBox.information(self._widget, "WARNING!", 'Tool enabled', QMessageBox.Ok)
					
		if(motor1.status=="FAULT"):
			first_time_enabled=True
                #print first_time_enabled
                        
    def callback_motor_status2(self,data):
            self.do_callback_motor_status.emit(data)
			

    def callback_robot_pose(self, data):
        global pos_x_kuka, pos_y_kuka, pos_z_kuka, pos_a_kuka, pos_b_kuka, pos_c_kuka#, elapsed_time_gauges#, gauges_failure
        #print 'CB:robot_pose_received',data
        pos_x_kuka=data.x
        pos_y_kuka=data.y
        pos_z_kuka=data.z
        pos_a_kuka=data.A
        pos_b_kuka=data.B
        pos_c_kuka=data.C
        #elapsed_time_gauges=time.time()-start_time_gauges
        #print 'time between robot callback and gauges' ,elapsed_time_gauges
        #if (elapsed_time_gauges>=2):
            #gauges_failure=True
    def callback_horiz_force(self, data):
        global horiz_force_read, horiz_force_empty
        #print 'force_received:',data.data
        horiz_force_read = data.data
        self._widget.vertforce_lcdNumber.setDigitCount(4)
        self._widget.vertforce_lcdNumber.display(round((data.data-horiz_force_empty)*0.19,1))
    def callback_horiz_force2(self,data):
            self.do_callback_horiz_force.emit(data)
        
    def callback_tool_weight(self, data):
        global weight_empty, weight_read#, gauges_failure, start_time_gauges
        #start_time_gauges=time.time()
        #gauges_failure=False
        self._widget.weight_lcdNumber.setDigitCount(4)
        palette = self._widget.weight_lcdNumber.palette()
        progressBar_palette = self._widget.weightProgressBar.palette()   
        #print 'CB:tool_weight_received',data
        weight_read=data.data
        weight_no_tool=data.data#-weight_empty
        weight_reads[0]=weight_no_tool
        for i in range(1, 5):
            weight_no_tool=weight_no_tool+weight_reads[i]
        weight_no_tool=weight_no_tool/5
        for i in range(1, 5):
            weight_reads[i]=weight_reads[i-1]
        self._widget.weight_lcdNumber.setDecMode()
        #self._widget.weight_lcdNumber.setNumDigits(3)
        self._widget.weight_lcdNumber.display(round(weight_no_tool,1))
        if (-weight_no_tool)<weight_expected_min:
            palette.setColor(palette.WindowText, QtGui.QColor(10, 10, 10))
            self._widget.weightProgressBar.setStyleSheet("""QProgressBar::chunk { background: grey; }""")
            self._widget.weightProgressBar_2.setStyleSheet("""QProgressBar::chunk { background: grey; }""")
            if(weight_no_tool>5):
                    self._widget.weightProgressBar_2.setStyleSheet("""QProgressBar::chunk { background: red; }""")
        elif (-weight_no_tool)<weight_expected_max:
            palette.setColor(palette.WindowText, QtGui.QColor(20, 230, 20))
            self._widget.weightProgressBar.setStyleSheet("""QProgressBar::chunk { background: green; }""")
        else:
            palette.setColor(palette.WindowText, QtGui.QColor(255, 50, 50))
            self._widget.weightProgressBar.setStyleSheet("""QProgressBar::chunk { background: red; }""")
        self._widget.weight_lcdNumber.setPalette(palette)
        if(weight_no_tool<0):
                self._widget.weightProgressBar_2.setValue(0)
                self._widget.weightProgressBar.setValue(-int(round(weight_no_tool)))
        else: 
                self._widget.weightProgressBar_2.setValue(int(round(weight_no_tool)))
                self._widget.weightProgressBar.setValue(0)
        
    def callback_tool_weight2(self,data):
            self.do_callback_tool_weight.emit(data)

    def callback_current(self, data):
        #print 'CB:current_received',data
        global tool_current
        tool_current = data.data
        self._widget.tool_force_lcdNumber.setDigitCount(4)
        self._widget.tool_force_lcdNumber.display(round(data.data,1))
    def callback_current2(self,data):
            self.do_callback_current.emit(data)
    
    def press_move_to_rotation_table_button(self):
		ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
		global KUKA_AUT
		if ret == QMessageBox.Ok:
			try:
				placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
				ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
				KUKA_AUT=True
				while KUKA_AUT: self.sleep_loop(0.3)
				placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)
				ret=placed_abs_service(H2O1_Pose_x, H2O1_Pose_y, H2O1_Pose_z, table_pose_a, H2O1_Pose_b, H2O1_Pose_c)
				KUKA_AUT=True
				while KUKA_AUT: self.sleep_loop(0.3)
				placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
				ret = placed_abs_service(table_pose_x, table_pose_y, table_pose_z, table_pose_a, table_pose_b, table_pose_c)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
#Pick buttons obus 2
    def press_pick_obus2_1_button(self):
        global Pick_Obus_21, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_21=True
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox2_1.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus2_2_button(self):
        global Pick_Obus_22, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_22=True
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox2_2.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus2_3_button(self):
        global Pick_Obus_23, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_23=True
            origin_pick=2      
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox2_3.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus2_4_button(self):
        global Pick_Obus_24, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_24=True
            origin_pick=2      
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox2_4.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
#Pick buttons obus 4
    def press_pick_obus4_1_button(self):
        global Pick_Obus_41, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_41=True
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox4_1.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus4_2_button(self):
        global Pick_Obus_42, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_42=True
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox4_2.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus4_3_button(self):
        global Pick_Obus_43, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_43=True
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox4_3.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus4_4_button(self):
        global Pick_Obus_44, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_44=True
            origin_pick=2 
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox4_4.setIcon(icon)     
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus4_5_button(self):
        global Pick_Obus_45, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_45=True
            origin_pick=2 
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox4_5.setIcon(icon)     
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
#Pick buttons obus 8
    def press_pick_obus8_1_button(self):
        global Pick_Obus_81, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_81=True
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_1.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus8_2_button(self):
        global Pick_Obus_82, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_82=True
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_2.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus8_3_button(self):
        global Pick_Obus_83, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_83=True
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_3.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus8_4_button(self):
        global Pick_Obus_84, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_84=True
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_4.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus8_5_button(self):
        global Pick_Obus_85, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_85=True
            origin_pick=2   
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_5.setIcon(icon)   
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus8_6_button(self):
        global Pick_Obus_86, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            Pick_Obus_86=True
            origin_pick=2 
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_6.setIcon(icon)     
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus8_7_button(self):
        global Pick_Obus_87, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            Pick_Obus_87=True
            origin_pick=2 
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_7.setIcon(icon)     
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus8_8_button(self):
        global Pick_Obus_88, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_88=True
            origin_pick=3
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_8.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus8_9_button(self):
        global Pick_Obus_89, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_89=True
            origin_pick=3
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_9.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus8_10_button(self):
        global Pick_Obus_810, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_810=True
            origin_pick=3
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_10.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus8_11_button(self):
        global Pick_Obus_811, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_811=True
            origin_pick=3
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_11.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus8_12_button(self):
        global Pick_Obus_812, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            Pick_Obus_812=True
            origin_pick=4      
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_12.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus8_13_button(self):
        global Pick_Obus_813, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            Pick_Obus_813=True
            origin_pick=4   
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_13.setIcon(icon)   
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus8_14_button(self):
        global Pick_Obus_814, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=4 
            Pick_Obus_814=True     
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox8_14.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
#Pick buttons obus 16
    def press_pick_obus16_1_button(self):
        global Pick_Obus_161, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=1
            Pick_Obus_161=True
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_1.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_2_button(self):
        global Pick_Obus_162, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=1
            Pick_Obus_162=True
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_2.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_3_button(self):
        global Pick_Obus_163, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_163=True
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_3.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_4_button(self):
        global Pick_Obus_164, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=1
            Pick_Obus_164=True
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_4.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_5_button(self):
        global Pick_Obus_165, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=1
            Pick_Obus_165=True
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_5.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_6_button(self):
        global Pick_Obus_166, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=2
            Pick_Obus_166=True    
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_6.setIcon(icon) 
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e  
    def press_pick_obus16_7_button(self):
        global Pick_Obus_167, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=2
            Pick_Obus_167=True
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_7.setIcon(icon)      
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e 
    def press_pick_obus16_8_button(self):
        global Pick_Obus_168, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=2 
            Pick_Obus_168=True  
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_8.setIcon(icon)   
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e 
    def press_pick_obus16_9_button(self):
        global Pick_Obus_169, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=2      
            Pick_Obus_169=True
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_9.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e 
    def press_pick_obus16_10_button(self):
        global Pick_Obus_1610, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=2  
            Pick_Obus_1610=True  
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_10.setIcon(icon)  
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e 
    def press_pick_obus16_11_button(self):
        global Pick_Obus_1611, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=3
            Pick_Obus_1611=True
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_11.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_12_button(self):
        global Pick_Obus_1612, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=3
            Pick_Obus_1612=True
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_12.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_13_button(self):
        global Pick_Obus_1613, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_1613=True
            origin_pick=3
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_13.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_14_button(self):
        global Pick_Obus_1614, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=3
            Pick_Obus_1614=True
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_14.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_15_button(self):
        global Pick_Obus_1615, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=3
            Pick_Obus_1615=True
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_15.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_16_button(self):
        global Pick_Obus_1616, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=4    
            Pick_Obus_1616=True 
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_16.setIcon(icon) 
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_17_button(self):
        global Pick_Obus_1617, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=4    
            Pick_Obus_1617=True  
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_17.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_18_button(self):
        global Pick_Obus_1618, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=4      
            Pick_Obus_1618=True
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_18.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_19_button(self):
        global Pick_Obus_1619, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=4 
            Pick_Obus_1619=True
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_19.setIcon(icon)     
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    def press_pick_obus16_20_button(self):
        global Pick_Obus_1620, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=4  
            Pick_Obus_1620=True 
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickBox16_20.setIcon(icon)   
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    #pressing obuses
    #huevera2
    #obus1
    def press_obus2_1_button(self):
        global Obus_21
        if(Obus_21==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_21=False
        if(Obus_21==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen 
                    Obus_21=True           
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera2Obus1Button.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H2O1_Pose_x, H2O1_Pose_y, H2O1_Pose_z, H2O1_Pose_a, H2O1_Pose_b, H2O1_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon()
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera2Obus1Button.setIcon(icon)
                    #self._widget.Huevera2Obus1Button.setIcon(QtGui.QIcon(PATH+"resource/images/symb_obus_abajo41x111.png", QtGui.QIcon.Disabled))
    #obus2
    def press_obus2_2_button(self):
        global Obus_22
        if(Obus_22==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_22=False
        if(Obus_22==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_22=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera2Obus2Button.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H2O2_Pose_x, H2O2_Pose_y, H2O2_Pose_z, H2O2_Pose_a, H2O2_Pose_b, H2O2_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon()
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png"), QtGui.QIcon.Disabled)
                   # self._widget.Huevera2Obus2Button.setIcon(icon)
    #huevera4
    #obus1
    def press_obus4_1_button(self):
        global Obus_41
        if(Obus_41==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_41=False
        if(Obus_41==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Obus_41=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera4Obus1Button.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H4O1_Pose_x, H4O1_Pose_y, H4O1_Pose_z, H4O1_Pose_a, H4O1_Pose_b, H4O1_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon()
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera4Obus1Button.setIcon(icon)
    #obus2
    def press_obus4_2_button(self):
        global Obus_42
        if(Obus_42==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_42=False
        if(Obus_42==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:    
                    #cambia el color de la imagen
                    Obus_42=True            
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera4Obus2Button.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H4O2_Pose_x, H4O2_Pose_y, H4O2_Pose_z, H4O2_Pose_a, H4O2_Pose_b, H4O2_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon()
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera4Obus2Button.setIcon(icon)
    #obus3
    def press_obus4_3_button(self):
        global Obus_43
        if(Obus_43==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_43=False
        if(Obus_43==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Obus_43=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera4Obus3Button.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H4O3_Pose_x, H4O3_Pose_y, H4O3_Pose_z, H4O3_Pose_a, H4O3_Pose_b, H4O3_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon()
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera4Obus3Button.setIcon(icon)
    #obus4
    def press_obus4_4_button(self):
        global Obus_44
        if(Obus_44==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_44=False
        if(Obus_44==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_44=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera4Obus4Button.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        #placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H4O4_Pose_x, H4O4_Pose_y, H4O4_Pose_z, H4O4_Pose_a, H4O4_Pose_b, H4O4_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon()
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera4Obus4Button.setIcon(icon)
    #huevera8
    #obus1
    def press_obus8_1_button(self):
        global Obus_81
        if(Obus_81==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_81=False
        if(Obus_81==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_81=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera8Obus1Button.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H8O1_Pose_x, H8O1_Pose_y, H8O1_Pose_z, H8O1_Pose_a, H8O1_Pose_b, H8O1_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera8Obus1Button.setIcon(icon)
    #obus2
    def press_obus8_2_button(self):
        global Obus_82
        if(Obus_82==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_82=False
        if(Obus_82==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_82=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera8Obus2Button.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H8O2_Pose_x, H8O2_Pose_y, H8O2_Pose_z, H8O2_Pose_a, H8O2_Pose_b, H8O2_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera8Obus2Button.setIcon(icon)
    #obus3
    def press_obus8_3_button(self):
        global Obus_83
        if(Obus_83==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_83=False
        if(Obus_83==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_83=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera8Obus3Button.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H8O3_Pose_x, H8O3_Pose_y, H8O3_Pose_z, H8O3_Pose_a, H8O3_Pose_b, H8O3_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera8Obus3Button.setIcon(icon)
    #obus4
    def press_obus8_4_button(self):
        global Obus_84
        if(Obus_84==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_84=False
        if(Obus_84==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_84=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera8Obus4Button.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H8O4_Pose_x, H8O4_Pose_y, H8O4_Pose_z, H8O4_Pose_a, H8O4_Pose_b, H8O4_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera8Obus4Button.setIcon(icon)
    #obus5
    def press_obus8_5_button(self):
        global Obus_85
        if(Obus_85==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_85=False
        if(Obus_85==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_85=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera8Obus5Button.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H8O5_Pose_x, H8O5_Pose_y, H8O5_Pose_z, H8O5_Pose_a, H8O5_Pose_b, H8O5_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera8Obus5Button.setIcon(icon)
    #obus6
    def press_obus8_6_button(self):
        global Obus_86
        if(Obus_86==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_86=False
        if(Obus_86==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Obus_86=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera8Obus6Button.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H8O6_Pose_x, H8O6_Pose_y, H8O6_Pose_z, H8O6_Pose_a, H8O6_Pose_b, H8O6_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera8Obus6Button.setIcon(icon)
    #obus7
    def press_obus8_7_button(self):
        global Obus_87
        if(Obus_87==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_87=False
        if(Obus_87==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_87=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera8Obus7Button.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H8O7_Pose_x, H8O7_Pose_y, H8O7_Pose_z, H8O7_Pose_a, H8O7_Pose_b, H8O7_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera8Obus7Button.setIcon(icon)
    #obus8
    def press_obus8_8_button(self):
        global Obus_88
        if(Obus_88==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_88=False
        if(Obus_88==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Obus_88=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera8Obus8Button.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H8O8_Pose_x, H8O8_Pose_y, H8O8_Pose_z, H8O8_Pose_a, H8O8_Pose_b, H8O8_Pose_c)
                        #KUKA_AUT=True
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera8Obus8Button.setIcon(icon)
    #huevera16
    #obus1
    def press_obus16_1_button(self):
        global Obus_161
        if(Obus_161==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_161=False
        if(Obus_161==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_161=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus1Button.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)
                        ret2 = placed_abs_service(H16O1_Pose_x, H16O1_Pose_y, H16O1_Pose_z, H16O1_Pose_a, H16O1_Pose_b, H16O1_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus1Button.setIcon(icon)
                
    #obus2
    def press_obus16_2_button(self):
        global Obus_162
        if(Obus_162==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_162=False
        if(Obus_162==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_162=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus2Button.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O2_Pose_x, H16O2_Pose_y, H16O2_Pose_z, H16O2_Pose_a, H16O2_Pose_b, H16O2_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus2Button.setIcon(icon)            
    #obus3
    def press_obus16_3_button(self):
        global Obus_163
        if(Obus_163==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_163=False
        if(Obus_163==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_163=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus3Button.setIcon(icon)                        
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O3_Pose_x, H16O3_Pose_y, H16O3_Pose_z, H16O3_Pose_a, H16O3_Pose_b, H16O3_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus3Button.setIcon(icon)            
    #obus4
    def press_obus16_4_button(self):
        global Obus_164
        if(Obus_164==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_164=False
        if(Obus_164==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_164=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus4Button.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O4_Pose_x, H16O4_Pose_y, H16O4_Pose_z, H16O4_Pose_a, H16O4_Pose_b, H16O4_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus4Button.setIcon(icon)            
    #obus5
    def press_obus16_5_button(self):
        global Obus_165
        if(Obus_165==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_165=False
        if(Obus_165==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_165=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus5Button.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O5_Pose_x, H16O5_Pose_y, H16O5_Pose_z, H16O5_Pose_a, H16O5_Pose_b, H16O5_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus5Button.setIcon(icon)            
    #obus6
    def press_obus16_6_button(self):
        global Obus_166
        if(Obus_166==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_166=False
        if(Obus_166==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Obus_166=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus6Button.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O6_Pose_x, H16O6_Pose_y, H16O6_Pose_z, H16O6_Pose_a, H16O6_Pose_b, H16O6_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus6Button.setIcon(icon)            
    #obus7
    def press_obus16_7_button(self):
        global Obus_167
        if(Obus_167==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_167=False
        if(Obus_167==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_167=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus7Button.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O7_Pose_x, H16O7_Pose_y, H16O7_Pose_z, H16O7_Pose_a, H16O7_Pose_b, H16O7_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus7Button.setIcon(icon) 
    #obus8
    def press_obus16_8_button(self):
        global Obus_168
        if(Obus_168==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_168=False
        if(Obus_168==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Obus_168=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus8Button.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O8_Pose_x, H16O8_Pose_y, H16O8_Pose_z, H16O8_Pose_a, H16O8_Pose_b, H16O8_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus8Button.setIcon(icon) 
    #obus9
    def press_obus16_9_button(self):
        global Obus_169
        if(Obus_169==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_169=False
        if(Obus_169==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_169=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus9Button.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O9_Pose_x, H16O9_Pose_y, H16O9_Pose_z, H16O9_Pose_a, H16O9_Pose_b, H16O9_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus9Button.setIcon(icon) 
    #obus10
    def press_obus16_10_button(self):
        global Obus_1610
        if(Obus_1610==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_1610=False
        if(Obus_1610==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_1610=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus10Button.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O10_Pose_x, H16O10_Pose_y, H16O10_Pose_z, H16O10_Pose_a, H16O10_Pose_b, H16O10_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus10Button.setIcon(icon) 
    #obus11
    def press_obus16_11_button(self):
        global Obus_1611
        if(Obus_1611==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_1611=False
        if(Obus_1611==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_1611=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus11Button.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O11_Pose_x, H16O11_Pose_y, H16O11_Pose_z, H16O11_Pose_a, H16O11_Pose_b, H16O11_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus11Button.setIcon(icon) 
    #obus12
    def press_obus16_12_button(self):
        global Obus_1612
        if(Obus_1612==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_1612=False
        if(Obus_1612==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_1612=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus12Button.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O12_Pose_x, H16O12_Pose_y, H16O12_Pose_z, H16O12_Pose_a, H16O12_Pose_b, H16O12_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus12Button.setIcon(icon) 
    #obus13
    def press_obus16_13_button(self):
        global Obus_1613
        if(Obus_1613==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_1613=False
        if(Obus_1613==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_1613=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus13Button.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O13_Pose_x, H16O13_Pose_y, H16O13_Pose_z, H16O13_Pose_a, H16O13_Pose_b, H16O13_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus13Button.setIcon(icon) 
    #obus14
    def press_obus16_14_button(self):
        global Obus_1614
        if(Obus_1614==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_1614=False
        if(Obus_1614==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Obus_1614=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus14Button.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O14_Pose_x, H16O14_Pose_y, H16O14_Pose_z, H16O14_Pose_a, H16O14_Pose_b, H16O14_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus14Button.setIcon(icon) 
    #obus15
    def press_obus16_15_button(self):
        global Obus_1615
        if(Obus_1615==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_1615=False
        if(Obus_1615==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Obus_1615=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus15Button.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O15_Pose_x, H16O15_Pose_y, H16O15_Pose_z, H16O15_Pose_a, H16O15_Pose_b, H16O15_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus15Button.setIcon(icon) 
    #obus16
    def press_obus16_16_button(self):
        global Obus_1616
        if(Obus_1616==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Obus_1616=False
        if(Obus_1616==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Obus_1616=True
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.Huevera16Obus16Button.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O16_Pose_x, H16O16_Pose_y, H16O16_Pose_z, H16O16_Pose_a, H16O16_Pose_b, H16O16_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.Huevera16Obus16Button.setIcon(icon) 
    def press_tool_homming(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nBe sure there is no obus picked', QMessageBox.Ok, QMessageBox.Cancel)
        #ret = QMessageBox.critical(self._widget, "WARNING!", 'The tool is activated and there is some weight \ndetected by the gauges!', QMessageBox.Ok)
        if ret == QMessageBox.Ok:
            limit_cont_current_service=rospy.ServiceProxy(srv_limit_cont_current, set_float_value)
            limit_peak_current_service=rospy.ServiceProxy(srv_limit_peak_current, set_float_value)
            limit_cont_current_service(current_limit_0)
            limit_peak_current_service(current_limit_0)
            #Call tool homing method
            global weight_empty, weight_read, TOOL_HOMED
            try:
                gripper_move_service = rospy.ServiceProxy(srv_finger_set_pose,set_odometry)
                homing_service = rospy.ServiceProxy(srv_tool_homing, home)           
                ret = homing_service()
                TOOL_HOMED=True 
                #weight_empty=weight_read
                gripper_move_service(0.02,0,0,-0.15)
                if ret == True:
                    TOOL_HOMED=True                 
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #set current again
            if finger_type == 0:
                limit_cont_current_service(current_limit_0)
                limit_peak_current_service(current_limit_0)
            elif finger_type == 1:
                limit_cont_current_service(current_limit_1)
                limit_peak_current_service(current_limit_1)
            elif finger_type == 2:
                limit_cont_current_service(current_limit_2)
                limit_peak_current_service(current_limit_2)
            elif finger_type == 3:
                limit_cont_current_service(current_limit_cont)
                limit_peak_current_service(current_limit_3)
            elif finger_type == 4:
                limit_cont_current_service(current_limit_cont)
                limit_peak_current_service(current_limit_4)
                
    def press_finger_adjust_button(self):
        if(not TOOL_HOMED):
                QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nHoming of the tool should be done first', QMessageBox.Ok, QMessageBox.Cancel)
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
        
    def deadMan_state_changed(self):
        ret_q = QMessageBox.warning(self._widget, "WARNING!", 'Changes to the current configuration will be applied', QMessageBox.Ok, QMessageBox.Cancel)
        if(ret_q==QMessageBox.Ok):
                    if(self._widget.deadMan_check.isChecked()):
                                    try:
                                        deadman_service=rospy.ServiceProxy(srv_deadman, SetBool)
                                        ret = deadman_service(True)
                                    except rospy.ServiceException, e:
                                        print "Service call failed: %s"%e
                    elif(self._widget.deadMan_check.isChecked()==False):
                                    try:
                                        deadman_service=rospy.ServiceProxy(srv_deadman, SetBool)
                                        ret = deadman_service(False)
                                    except rospy.ServiceException, e:
                                        print "Service call failed: %s"%e
        else : 
                self._widget.deadMan_check.nextCheckState()
                
                
    def toolAngle_state_changed(self):
        ret_q = QMessageBox.warning(self._widget, "WARNING!", 'Changes to the current configuration will be applied', QMessageBox.Ok, QMessageBox.Cancel)
        if(ret_q==QMessageBox.Ok):
                        if(self._widget.toolAngle_check.isChecked()):
                                    try:
                                        angle_mode_service=rospy.ServiceProxy(srv_angle_mode, SetBool)
                                        ret = angle_mode_service(True)
                                    except rospy.ServiceException, e:
                                        print "Service call failed: %s"%e
                        elif(self._widget.toolAngle_check.isChecked()==False):
                                    try:
                                        angle_mode_service=rospy.ServiceProxy(srv_angle_mode, SetBool)
                                        ret = angle_mode_service(False)
                                    except rospy.ServiceException, e:
                                        print "Service call failed: %s"%e
        else:
                self._widget.toolAngle_check.nextCheckState()

    def toolOrientation_state_changed(self):
        ret_q = QMessageBox.warning(self._widget, "WARNING!", 'Changes to the current configuration will be applied', QMessageBox.Ok, QMessageBox.Cancel)
        if(ret_q==QMessageBox.Ok):
                if(self._widget.toolOrientation_check.isChecked()):
                            try:
                                toolOrientation_service=rospy.ServiceProxy(srv_rel_tool, SetBool)
                                ret = toolOrientation_service(True)
                            except rospy.ServiceException, e:
                                print "Service call failed: %s"%e
                elif(self._widget.toolOrientation_check.isChecked()==False):
                            try:
                                toolOrientation_service=rospy.ServiceProxy(srv_rel_tool, SetBool)
                                ret = toolOrientation_service(False)
                            except rospy.ServiceException, e:
                                print "Service call failed: %s"%e
        else:
                self._widget.toolOrientation_check.nextCheckState()
			
    
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
            

    def press_homming_button(self):     
        global KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=0
            try:
                homming_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=homming_rel_service(0, 0,pose_z_safe-pos_z_kuka , 0, 0, 0)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                home_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=home_A1_A6_service(0.0, 177)
                KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(table_pose_x, table_pose_y, table_pose_z, table_pose_a, table_pose_b, table_pose_c)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            
    def press_picktest_button(self):
        global KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, 20, 0, 0, 0)
                #KUKA_AUT=True
                #while KUKA_AUT: time.sleep(0.1)
                if ret_rel == True:
                        CURRENT_STATE=STATE_DOING_PICK_TEST
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    
    def press_tare_button(self):
        tare_service = rospy.ServiceProxy(srv_tare_gauges, SetBool)
        ret = tare_service(True)

    def press_tare_reset_button(self):
        tare_service = rospy.ServiceProxy(srv_tare_gauges, SetBool)
        ret = tare_service(False)
    #################################################CALIBRE SELECTION
    def calibre_selected(self, index):
        global finger_type, weight_expected_min, weight_expected_max, current_limit_picked
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
            self._widget.background_plate_pick.setPixmap(pixmap)
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
            for i in range(1, 21):
				name_method='PickBox16'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()
            for i in range(1, 15):
				name_method='PickBox8'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()
            for i in range(1, 6):
				name_method='PickBox4'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()
            for i in range(1, 5):
				name_method='PickBox2'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()
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
            limit_cont_current_service(current_limit_1)
            limit_peak_current_service(current_limit_1)
            current_limit_picked = current_limit_1

            pixmap = QtGui.QPixmap(PATH+"resource/images/rotated-fondo_huevera_16.png")
            self._widget.background_plate.setPixmap(pixmap)
            pixmap_pick = QtGui.QPixmap(PATH+"resource/images/BoxPick.png")
            self._widget.background_plate_pick.setPixmap(pixmap_pick)
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
            for i in range(1, 21):
				name_method='PickBox16'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.show()
            for i in range(1, 15):
				name_method='PickBox8'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()
            for i in range(1, 6):
				name_method='PickBox4'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()
            for i in range(1, 5):
				name_method='PickBox2'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()

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
            limit_cont_current_service(current_limit_2)
            limit_peak_current_service(current_limit_2)
            current_limit_picked = current_limit_2


            pixmap = QtGui.QPixmap(PATH+"resource/images/rotated-fondo_huevera_8.png")
            self._widget.background_plate.setPixmap(pixmap)
            pixmap_pick = QtGui.QPixmap(PATH+"resource/images/BoxPick.png")
            self._widget.background_plate_pick.setPixmap(pixmap_pick)
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
            for i in range(1, 21):
				name_method='PickBox16'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()
            for i in range(1, 15):
				name_method='PickBox8'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.show()
            for i in range(1, 6):
				name_method='PickBox4'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()
            for i in range(1, 5):
				name_method='PickBox2'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()  

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
            limit_cont_current_service(current_limit_cont)
            limit_peak_current_service(current_limit_3)
            current_limit_picked = current_limit_3

            pixmap = QtGui.QPixmap(PATH+"resource/images/rotated-fondo_huevera_4.png")
            self._widget.background_plate.setPixmap(pixmap)
            pixmap_pick = QtGui.QPixmap(PATH+"resource/images/BoxPick.png")
            self._widget.background_plate_pick.setPixmap(pixmap_pick)
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
            for i in range(1, 21):
				name_method='PickBox16'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()
            for i in range(1, 15):
				name_method='PickBox8'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()
            for i in range(1, 6):
				name_method='PickBox4'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.show()
            for i in range(1, 5):
				name_method='PickBox2'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide() 

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
            limit_cont_current_service(current_limit_cont)
            limit_peak_current_service(current_limit_4)
            current_limit_picked = current_limit_4
            
            pixmap = QtGui.QPixmap(PATH+"resource/images/rotated-fondo_huevera_2.png")
            self._widget.background_plate.setPixmap(pixmap)
            pixmap_pick = QtGui.QPixmap(PATH+"resource/images/BoxPick.png")
            self._widget.background_plate_pick.setPixmap(pixmap_pick)
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
            for i in range(1, 21):
				name_method='PickBox16'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()
            for i in range(1, 15):
				name_method='PickBox8'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()
            for i in range(1, 6):
				name_method='PickBox4'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.hide()
            for i in range(1, 5):
				name_method='PickBox2'+'_'+str(i)
				test_method=getattr(self._widget, name_method)
				test_method.show()
        #weight progress bar
        self._widget.weightProgressBar.setMaximum(weight_expected_max*1.3)


    def load_robot_description(self, gripper_model):
        command_string = "rosparam load ~/kuka_catkin_ws/src/kuka_experimental/kuka_robot_bringup/robot/bin/kr120toolv%d.urdf /robot_description" % gripper_model
        os.system(command_string)
        
    def press_reset_external_pc_button(self):
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nExternal PC is going to reset.\n Wait 10 sec and restart the GUI.', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #command_string = "ssh robotnik@192.168.1.10 sudo -S <<< \"R0b0tn1K\" reboot \n"
            command_string = "~/kuka_catkin_ws/src/rqt_kuka/scripts/reboot.sh"
            print command_string
            os.system(command_string)
###TEST APRIETE AUTOMATICO: si el nodo de las galgas falla se va  a liar
    def aut_press_tool(self):
        ##loop for closing tool (first translation then angle)
        gripper_move_service = rospy.ServiceProxy(srv_finger_set_pose, set_odometry)
        press_counter=0
        old_pos_x=0
        self.desactivate_buttons()
        while press_counter<5 :
                print press_counter
                print 'angle tool '
                print angle_tool
                gripper_move_service(x_tool,0,0,angle_tool-0.01)
                self.sleep_loop(0.15)
                if(tool_current>current_limit_picked):
                        press_counter=press_counter+1
                else:
                        pres_counter=0
                print abs(x_tool-old_pos_x)
                print 'current:'
                print tool_current
                
        press_counter=0
        while press_counter<5 :
                print press_counter
                gripper_move_service(x_tool+0.02,0,0,angle_tool)
                self.sleep_loop(0.15)
                if(tool_current>current_limit_picked or abs(x_tool-old_pos_x)<0.002):
                        press_counter=press_counter+1
                else:
                       pres_counter=0
                       old_pos_x=x_tool
        self.sleep_loop(0.5)
        self.activate_buttons()
        
        
    def callback_tool_state(self, data):
        global x_tool, angle_tool
        x_tool = data.position[2]
        angle_tool = data.position[3]
    
    def press_run_program_button(self):
        command_string = "rosnode kill /kuka_pad/joy; sleep 1; rosnode kill /kuka_pad/robotnik_trajectory_pad_node; sleep 1; rosnode kill /kuka_robot/kuka_cartesian_hardware_interface; sleep 1; roslaunch kuka_robot_bringup kuka_robot_bringup_standalone.launch &"
        os.system(command_string)
        
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.sub_robot_moving.unregister()
        self.sub_robot_pose.unregister()
        self.sub_tool_weight.unregister()
        self.sub_tool_current.unregister()
        self.sub_tool_status.unregister()
        pass
    def sleep_loop(self,delay):
        loop = QtCore.QEventLoop()
        timer = QtCore.QTimer()
        timer.setInterval(delay*1000)
        timer.setSingleShot(True)
        timer.timeout.connect(loop.quit)
        timer.start()
        loop.exec_()
