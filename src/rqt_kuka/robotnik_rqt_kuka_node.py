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
#PATH="/home/angel/workspaces/rqt_kuka/src/rqt_kuka/"

TOOL_HOMED=False
KUKA_AUT=False
finger_type=0
#gauges_failure=False
under_voltage_tool=False
first_time_enabled=False
#start_time_gauges=time.time()
angle_mode=True
angle_tool=0
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
Place_Obus_2_1=False
Place_Obus_2_2=False

#Hueveras de 4
Place_Obus_4_1=False
Place_Obus_4_2=False
Place_Obus_4_3=False
Place_Obus_4_4=False

#Hueveras de 8
Place_Obus_8_1=False
Place_Obus_8_2=False
Place_Obus_8_3=False
Place_Obus_8_4=False
Place_Obus_8_5=False
Place_Obus_8_6=False
Place_Obus_8_7=False
Place_Obus_8_8=False

#Hueveras de 16
Place_Obus_16_1=False
Place_Obus_16_2=False
Place_Obus_16_3=False
Place_Obus_16_4=False
Place_Obus_16_5=False
Place_Obus_16_6=False
Place_Obus_16_7=False
Place_Obus_16_8=False
Place_Obus_16_9=False
Place_Obus_16_10=False
Place_Obus_16_11=False
Place_Obus_16_12=False
Place_Obus_16_13=False
Place_Obus_16_14=False
Place_Obus_16_15=False
Place_Obus_16_16=False

#Pick Positions

#Obus already placed
#Hueveras de 2
Pick_Obus_2_1=False
Pick_Obus_2_2=False
Pick_Obus_2_3=False
Pick_Obus_2_4=False
#Hueveras de 4
Pick_Obus_4_1=False
Pick_Obus_4_2=False
Pick_Obus_4_3=False
Pick_Obus_4_4=False
Pick_Obus_4_5=False

#Hueveras de 8
Pick_Obus_8_1=False
Pick_Obus_8_2=False
Pick_Obus_8_3=False
Pick_Obus_8_4=False
Pick_Obus_8_5=False
Pick_Obus_8_6=False
Pick_Obus_8_7=False
Pick_Obus_8_8=False
Pick_Obus_8_9=False
Pick_Obus_8_10=False
Pick_Obus_8_11=False
Pick_Obus_8_12=False
Pick_Obus_8_13=False
Pick_Obus_8_14=False

#Hueveras de 16
Pick_Obus_16_1=False
Pick_Obus_16_2=False
Pick_Obus_16_3=False
Pick_Obus_16_4=False
Pick_Obus_16_5=False
Pick_Obus_16_6=False
Pick_Obus_16_7=False
Pick_Obus_16_8=False
Pick_Obus_16_9=False
Pick_Obus_16_10=False
Pick_Obus_16_11=False
Pick_Obus_16_12=False
Pick_Obus_16_13=False
Pick_Obus_16_14=False
Pick_Obus_16_15=False
Pick_Obus_16_16=False
Pick_Obus_16_17=False
Pick_Obus_16_18=False
Pick_Obus_16_19=False
Pick_Obus_16_20=False

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
        
        print '__Checking background processes__'        
        #Joysticks management with multiplexor        
        command_string = "rosrun topic_tools mux /kuka_pad/joy /kuka_pad/ps4_joy /kuka_pad/itowa_joy mux:=mux_joy __name:=joy_mux_node &"
        os.system(command_string)
        #PS4 by default
        command_string = "rosrun topic_tools mux_select mux_joy /kuka_pad/ps4_joy"
        os.system(command_string)
        
        ###launch MAIN nodes: Joysticks and main
        command_string = "rosnode kill /kuka_pad/ps4_joystick; sleep 1; rosnode kill /kuka_pad/itowa_safe_joystick; sleep 1;rosnode kill /kuka_pad/robotnik_trajectory_pad_node; sleep 1; rosnode kill /kuka_robot/kuka_cartesian_hardware_interface; sleep 1; roslaunch kuka_robot_bringup kuka_robot_bringup_standalone.launch &"        
        ###command_string = "rosnode kill /kuka_pad/ps4_joystick; sleep 1; rosnode kill /kuka_pad/itowa_safe_joystick; sleep 1;rosnode kill /kuka_pad/robotnik_trajectory_pad_node; sleep 1; rosnode kill /kuka_robot/kuka_cartesian_hardware_interface; sleep 1;"        
        os.system(command_string)
                    
        # add signals/slots
        #select obus calibre
        self._widget.calibre_comboBox.currentIndexChanged.connect(self.calibre_selected)
        self._widget.joy_comboBox.currentIndexChanged.connect(self.joy_selected)
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
        self._widget.Reset_Robot_Button.pressed.connect(self.press_reset_robot_button)
        #self._widget.Reset_Robot_Button.hide()
        self._widget.MoveToTable_Button.pressed.connect(self.press_homming_button)#self.press_move_to_rotation_table_button)
        
        
        self._widget.PickTest_Button.pressed.connect(self.press_picktest_button)
        self._widget.Gripper_Homing_Button.pressed.connect(self.press_tool_homming)
        self._widget.Led_On_Button.pressed.connect(self.press_led_on_button)
        self._widget.Led_Off_Button.pressed.connect(self.press_led_off_button)
        self._widget.Light_On_Button.pressed.connect(self.press_light_on_button)
        self._widget.Light_Off_Button.pressed.connect(self.press_light_off_button)
        self._widget.resetPositions_Button_place.pressed.connect(self.press_reset_positions_button_place)
        self._widget.resetPositions_Button_pick.pressed.connect(self.press_reset_positions_button_pick)
        self._widget.undoPositions_Button_pick.pressed.connect(self.press_undo_positions_button_pick)
        self._widget.undoPositions_Button_place.pressed.connect(self.press_undo_positions_button_place)
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
        self._widget.PickObus2_1.clicked.connect(self.press_pick_obus2_1_button)
        self._widget.PickObus2_1.hide()
        path = self.select_icon('pick',[2,1], 0)
        mask = QtGui.QPixmap(path)
        self._widget.PickObus2_1.setMask(mask.mask())
        self._widget.PickObus2_1.setMouseTracking(True)
        self._widget.PickObus2_1.installEventFilter(self)
        #Obus 2
        self._widget.PickObus2_2.clicked.connect(self.press_pick_obus2_2_button)
        self._widget.PickObus2_2.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png")
        self._widget.PickObus2_2.setMask(mask.mask())
        self._widget.PickObus2_2.setMouseTracking(True)       
        self._widget.PickObus2_2.installEventFilter(self)
        #Obus 3
        self._widget.PickObus2_3.clicked.connect(self.press_pick_obus2_3_button)
        self._widget.PickObus2_3.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png")
        self._widget.PickObus2_3.setMask(mask.mask())
        self._widget.PickObus2_3.setMouseTracking(True)       
        self._widget.PickObus2_3.installEventFilter(self)
        #Obus 4
        self._widget.PickObus2_4.clicked.connect(self.press_pick_obus2_4_button)
        self._widget.PickObus2_4.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png")
        self._widget.PickObus2_4.setMask(mask.mask())
        self._widget.PickObus2_4.setMouseTracking(True)       
        self._widget.PickObus2_4.installEventFilter(self)
        
        #Huevera 4PICK
        #Obus 1
        self._widget.PickObus4_1.clicked.connect(self.press_pick_obus4_1_button)
        self._widget.PickObus4_1.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.PickObus4_1.setMask(mask.mask())
        self._widget.PickObus4_1.setMouseTracking(True)       
        self._widget.PickObus4_1.installEventFilter(self)
        #Obus 2
        self._widget.PickObus4_2.clicked.connect(self.press_pick_obus4_2_button)
        self._widget.PickObus4_2.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.PickObus4_2.setMask(mask.mask())
        self._widget.PickObus4_2.setMouseTracking(True)       
        self._widget.PickObus4_2.installEventFilter(self)
        #Obus 3
        self._widget.PickObus4_3.clicked.connect(self.press_pick_obus4_3_button)
        self._widget.PickObus4_3.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.PickObus4_3.setMask(mask.mask())
        self._widget.PickObus4_3.setMouseTracking(True)       
        self._widget.PickObus4_3.installEventFilter(self)
        #Obus 4
        self._widget.PickObus4_4.clicked.connect(self.press_pick_obus4_4_button)
        self._widget.PickObus4_4.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101.png")
        self._widget.PickObus4_4.setMask(mask.mask())
        self._widget.PickObus4_4.setMouseTracking(True)       
        self._widget.PickObus4_4.installEventFilter(self)
        #Obus 5
        self._widget.PickObus4_5.clicked.connect(self.press_pick_obus4_5_button)
        self._widget.PickObus4_5.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101.png")
        self._widget.PickObus4_5.setMask(mask.mask())
        self._widget.PickObus4_5.setMouseTracking(True)       
        self._widget.PickObus4_5.installEventFilter(self)
        
        #Huevera 8PICK
        #obus1
        self._widget.PickObus8_1.clicked.connect(self.press_pick_obus8_1_button)
        self._widget.PickObus8_1.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickObus8_1.setMask(mask.mask())
        self._widget.PickObus8_1.setMouseTracking(True)       
        self._widget.PickObus8_1.installEventFilter(self)
        #obus2
        self._widget.PickObus8_2.clicked.connect(self.press_pick_obus8_2_button)
        self._widget.PickObus8_2.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickObus8_2.setMask(mask.mask())
        self._widget.PickObus8_2.setMouseTracking(True)       
        self._widget.PickObus8_2.installEventFilter(self)
        #obus3
        self._widget.PickObus8_3.clicked.connect(self.press_pick_obus8_3_button)
        self._widget.PickObus8_3.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickObus8_3.setMask(mask.mask())
        self._widget.PickObus8_3.setMouseTracking(True)       
        self._widget.PickObus8_3.installEventFilter(self)
        #obus4
        self._widget.PickObus8_4.clicked.connect(self.press_pick_obus8_4_button)
        self._widget.PickObus8_4.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickObus8_4.setMask(mask.mask())
        self._widget.PickObus8_4.setMouseTracking(True)       
        self._widget.PickObus8_4.installEventFilter(self)
        #obus5
        self._widget.PickObus8_5.clicked.connect(self.press_pick_obus8_5_button)
        self._widget.PickObus8_5.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickObus8_5.setMask(mask.mask())
        self._widget.PickObus8_5.setMouseTracking(True)       
        self._widget.PickObus8_5.installEventFilter(self)
        #obus6
        self._widget.PickObus8_6.clicked.connect(self.press_pick_obus8_6_button)
        self._widget.PickObus8_6.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickObus8_6.setMask(mask.mask())
        self._widget.PickObus8_6.setMouseTracking(True)       
        self._widget.PickObus8_6.installEventFilter(self)
        #obus7
        self._widget.PickObus8_7.clicked.connect(self.press_pick_obus8_7_button)
        self._widget.PickObus8_7.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PickObus8_7.setMask(mask.mask())
        self._widget.PickObus8_7.setMouseTracking(True)       
        self._widget.PickObus8_7.installEventFilter(self)
        #obus8
        self._widget.PickObus8_8.clicked.connect(self.press_pick_obus8_8_button)
        self._widget.PickObus8_8.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickObus8_8.setMask(mask.mask())
        self._widget.PickObus8_8.setMouseTracking(True)       
        self._widget.PickObus8_8.installEventFilter(self)
        #obus9
        self._widget.PickObus8_9.clicked.connect(self.press_pick_obus8_9_button)
        self._widget.PickObus8_9.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickObus8_9.setMask(mask.mask())
        self._widget.PickObus8_9.setMouseTracking(True)       
        self._widget.PickObus8_9.installEventFilter(self)
        #obus9
        self._widget.PickObus8_10.clicked.connect(self.press_pick_obus8_10_button)
        self._widget.PickObus8_10.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickObus8_10.setMask(mask.mask())
        self._widget.PickObus8_10.setMouseTracking(True)       
        self._widget.PickObus8_10.installEventFilter(self)
        #obus11
        self._widget.PickObus8_11.clicked.connect(self.press_pick_obus8_11_button)
        self._widget.PickObus8_11.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickObus8_11.setMask(mask.mask())
        self._widget.PickObus8_11.setMouseTracking(True)       
        self._widget.PickObus8_11.installEventFilter(self)
        #obus12
        self._widget.PickObus8_12.clicked.connect(self.press_pick_obus8_12_button)
        self._widget.PickObus8_12.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickObus8_12.setMask(mask.mask())
        self._widget.PickObus8_12.setMouseTracking(True)       
        self._widget.PickObus8_12.installEventFilter(self)
        #obus13
        self._widget.PickObus8_13.clicked.connect(self.press_pick_obus8_13_button)
        self._widget.PickObus8_13.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickObus8_13.setMask(mask.mask())
        self._widget.PickObus8_13.setMouseTracking(True)       
        self._widget.PickObus8_13.installEventFilter(self)
        #obus14
        self._widget.PickObus8_14.clicked.connect(self.press_pick_obus8_14_button)
        self._widget.PickObus8_14.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PickObus8_14.setMask(mask.mask())
        self._widget.PickObus8_14.setMouseTracking(True)       
        self._widget.PickObus8_14.installEventFilter(self)
        
        #Huevera 16PICK
        #obus1
        self._widget.PickObus16_1.clicked.connect(self.press_pick_obus16_1_button)
        self._widget.PickObus16_1.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickObus16_1.setMask(mask.mask())
        self._widget.PickObus16_1.setMouseTracking(True)       
        self._widget.PickObus16_1.installEventFilter(self)
        #obus2
        self._widget.PickObus16_2.clicked.connect(self.press_pick_obus16_2_button)
        self._widget.PickObus16_2.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickObus16_2.setMask(mask.mask())
        self._widget.PickObus16_2.setMouseTracking(True)       
        self._widget.PickObus16_2.installEventFilter(self)
        #obus3
        self._widget.PickObus16_3.clicked.connect(self.press_pick_obus16_3_button)
        self._widget.PickObus16_3.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickObus16_3.setMask(mask.mask())
        self._widget.PickObus16_3.setMouseTracking(True)       
        self._widget.PickObus16_3.installEventFilter(self)
        #obus4
        self._widget.PickObus16_4.clicked.connect(self.press_pick_obus16_4_button)
        self._widget.PickObus16_4.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickObus16_4.setMask(mask.mask())
        self._widget.PickObus16_4.setMouseTracking(True)       
        self._widget.PickObus16_4.installEventFilter(self)
        #obus5
        self._widget.PickObus16_5.clicked.connect(self.press_pick_obus16_5_button)
        self._widget.PickObus16_5.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickObus16_5.setMask(mask.mask())
        self._widget.PickObus16_5.setMouseTracking(True)       
        self._widget.PickObus16_5.installEventFilter(self)
        #obus6
        self._widget.PickObus16_6.clicked.connect(self.press_pick_obus16_6_button)
        self._widget.PickObus16_6.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickObus16_6.setMask(mask.mask())
        self._widget.PickObus16_6.setMouseTracking(True)       
        self._widget.PickObus16_6.installEventFilter(self)
        #obus7
        self._widget.PickObus16_7.clicked.connect(self.press_pick_obus16_7_button)
        self._widget.PickObus16_7.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickObus16_7.setMask(mask.mask())
        self._widget.PickObus16_7.setMouseTracking(True)       
        self._widget.PickObus16_7.installEventFilter(self)
        #obus8
        self._widget.PickObus16_8.clicked.connect(self.press_pick_obus16_8_button)
        self._widget.PickObus16_8.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickObus16_8.setMask(mask.mask())
        self._widget.PickObus16_8.setMouseTracking(True)       
        self._widget.PickObus16_8.installEventFilter(self)
        #obus9
        self._widget.PickObus16_9.clicked.connect(self.press_pick_obus16_9_button)
        self._widget.PickObus16_9.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickObus16_9.setMask(mask.mask())
        self._widget.PickObus16_9.setMouseTracking(True)       
        self._widget.PickObus16_9.installEventFilter(self)
        #obus10
        self._widget.PickObus16_10.clicked.connect(self.press_pick_obus16_10_button)
        self._widget.PickObus16_10.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PickObus16_10.setMask(mask.mask())
        self._widget.PickObus16_10.setMouseTracking(True)       
        self._widget.PickObus16_10.installEventFilter(self)
        #obus11
        self._widget.PickObus16_11.clicked.connect(self.press_pick_obus16_11_button)
        self._widget.PickObus16_11.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickObus16_11.setMask(mask.mask())
        self._widget.PickObus16_11.setMouseTracking(True)       
        self._widget.PickObus16_11.installEventFilter(self)
        #obus12
        self._widget.PickObus16_12.clicked.connect(self.press_pick_obus16_12_button)
        self._widget.PickObus16_12.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickObus16_12.setMask(mask.mask())
        self._widget.PickObus16_12.setMouseTracking(True)       
        self._widget.PickObus16_12.installEventFilter(self)
        #obus13
        self._widget.PickObus16_13.clicked.connect(self.press_pick_obus16_13_button)
        self._widget.PickObus16_13.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickObus16_13.setMask(mask.mask())
        self._widget.PickObus16_13.setMouseTracking(True)       
        self._widget.PickObus16_13.installEventFilter(self)
        #obus14
        self._widget.PickObus16_14.clicked.connect(self.press_pick_obus16_14_button)
        self._widget.PickObus16_14.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickObus16_14.setMask(mask.mask())
        self._widget.PickObus16_14.setMouseTracking(True)       
        self._widget.PickObus16_14.installEventFilter(self)
        #obus15
        self._widget.PickObus16_15.clicked.connect(self.press_pick_obus16_15_button)
        self._widget.PickObus16_15.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickObus16_15.setMask(mask.mask())
        self._widget.PickObus16_15.setMouseTracking(True)       
        self._widget.PickObus16_15.installEventFilter(self)
        #obus16
        self._widget.PickObus16_16.clicked.connect(self.press_pick_obus16_16_button)
        self._widget.PickObus16_16.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickObus16_16.setMask(mask.mask())
        self._widget.PickObus16_16.setMouseTracking(True)       
        self._widget.PickObus16_16.installEventFilter(self)
        #obus17
        self._widget.PickObus16_17.clicked.connect(self.press_pick_obus16_17_button)
        self._widget.PickObus16_17.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickObus16_17.setMask(mask.mask())
        self._widget.PickObus16_17.setMouseTracking(True)       
        self._widget.PickObus16_17.installEventFilter(self)
        #obus18
        self._widget.PickObus16_18.clicked.connect(self.press_pick_obus16_18_button)
        self._widget.PickObus16_18.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickObus16_18.setMask(mask.mask())
        self._widget.PickObus16_18.setMouseTracking(True)       
        self._widget.PickObus16_18.installEventFilter(self)
        #obus19
        self._widget.PickObus16_19.clicked.connect(self.press_pick_obus16_19_button)
        self._widget.PickObus16_19.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickObus16_19.setMask(mask.mask())
        self._widget.PickObus16_19.setMouseTracking(True)       
        self._widget.PickObus16_19.installEventFilter(self)
        #obus20
        self._widget.PickObus16_20.clicked.connect(self.press_pick_obus16_20_button)
        self._widget.PickObus16_20.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PickObus16_20.setMask(mask.mask())
        self._widget.PickObus16_20.setMouseTracking(True)       
        self._widget.PickObus16_20.installEventFilter(self)
        ##obuses buttons PLACE
        #Huevera_2
        h2o1posex=70
        h2o1posey=560
        #obus1     
        self._widget.PlaceObus2_1.setGeometry(h2o1posex,h2o1posey,111,41)
        self._widget.PlaceObus2_1.clicked.connect(self.press_obus2_1_button)
        self._widget.PlaceObus2_1.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo41x111.png"))
        self._widget.PlaceObus2_1.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png")
        self._widget.PlaceObus2_1.setMask(mask.mask())
        self._widget.PlaceObus2_1.setMouseTracking(True)       
        self._widget.PlaceObus2_1.installEventFilter(self)
        #obus2
        self._widget.PlaceObus2_2.setGeometry(h2o1posex,h2o1posey-70,111,41)
        self._widget.PlaceObus2_2.clicked.connect(self.press_obus2_2_button)
        self._widget.PlaceObus2_2.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo41x111.png"))
        self._widget.PlaceObus2_2.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png")
        self._widget.PlaceObus2_2.setMask(mask.mask())
        self._widget.PlaceObus2_2.setMouseTracking(True)       
        self._widget.PlaceObus2_2.installEventFilter(self)        
        
        #Huevera_4
        h4o1posex=60
        h4o1posey=600
        #obus1
        self._widget.PlaceObus4_1.setGeometry(h4o1posex,h4o1posey,102,37)
        self._widget.PlaceObus4_1.clicked.connect(self.press_obus4_1_button)
        self._widget.PlaceObus4_1.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"))
        self._widget.PlaceObus4_1.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.PlaceObus4_1.setMask(mask.mask())
        self._widget.PlaceObus4_1.setMouseTracking(True)       
        self._widget.PlaceObus4_1.installEventFilter(self)
        #obus2
        self._widget.PlaceObus4_2.setGeometry(h4o1posex,h4o1posey-50,102,37)
        self._widget.PlaceObus4_2.clicked.connect(self.press_obus4_2_button)
        self._widget.PlaceObus4_2.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"))
        self._widget.PlaceObus4_2.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.PlaceObus4_2.setMask(mask.mask())
        self._widget.PlaceObus4_2.setMouseTracking(True)       
        self._widget.PlaceObus4_2.installEventFilter(self)
        #obus3
        self._widget.PlaceObus4_3.setGeometry(h4o1posex,h4o1posey-102,102,37)
        self._widget.PlaceObus4_3.clicked.connect(self.press_obus4_3_button)
        self._widget.PlaceObus4_3.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"))
        self._widget.PlaceObus4_3.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.PlaceObus4_3.setMask(mask.mask())
        self._widget.PlaceObus4_3.setMouseTracking(True)       
        self._widget.PlaceObus4_3.installEventFilter(self)
        #obus4
        self._widget.PlaceObus4_4.setGeometry(h4o1posex,h4o1posey-152,102,37)
        self._widget.PlaceObus4_4.clicked.connect(self.press_obus4_4_button)
        self._widget.PlaceObus4_4.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"))
        self._widget.PlaceObus4_4.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo31x101.png")
        self._widget.PlaceObus4_4.setMask(mask.mask())
        self._widget.PlaceObus4_4.setMouseTracking(True)       
        self._widget.PlaceObus4_4.installEventFilter(self)

        #Huevera_8
        h8o1posex=60
        h8o1posey=610
        #obus1
        self._widget.PlaceObus8_1.setGeometry(h8o1posex,h8o1posey,71,26)
        self._widget.PlaceObus8_1.clicked.connect(self.press_obus8_1_button)
        self._widget.PlaceObus8_1.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"))
        self._widget.PlaceObus8_1.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PlaceObus8_1.setMask(mask.mask())
        self._widget.PlaceObus8_1.setMouseTracking(True)       
        self._widget.PlaceObus8_1.installEventFilter(self)
        #obus2
        self._widget.PlaceObus8_2.setGeometry(h8o1posex,h8o1posey-38,71,26)
        self._widget.PlaceObus8_2.clicked.connect(self.press_obus8_2_button)
        self._widget.PlaceObus8_2.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"))
        self._widget.PlaceObus8_2.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PlaceObus8_2.setMask(mask.mask())
        self._widget.PlaceObus8_2.setMouseTracking(True)       
        self._widget.PlaceObus8_2.installEventFilter(self)
        #obus3
        self._widget.PlaceObus8_3.setGeometry(h8o1posex,h8o1posey-86,71,26)
        self._widget.PlaceObus8_3.clicked.connect(self.press_obus8_3_button)
        self._widget.PlaceObus8_3.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"))
        self._widget.PlaceObus8_3.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PlaceObus8_3.setMask(mask.mask())
        self._widget.PlaceObus8_3.setMouseTracking(True)       
        self._widget.PlaceObus8_3.installEventFilter(self)
        #obus4
        self._widget.PlaceObus8_4.setGeometry(h8o1posex,h8o1posey-132,71,26)
        self._widget.PlaceObus8_4.clicked.connect(self.press_obus8_4_button)
        self._widget.PlaceObus8_4.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"))
        self._widget.PlaceObus8_4.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png")
        self._widget.PlaceObus8_4.setMask(mask.mask())
        self._widget.PlaceObus8_4.setMouseTracking(True)       
        self._widget.PlaceObus8_4.installEventFilter(self)
        #obus5
        self._widget.PlaceObus8_5.setGeometry(h8o1posex+51,h8o1posey-18,71,26)
        self._widget.PlaceObus8_5.clicked.connect(self.press_obus8_5_button)
        self._widget.PlaceObus8_5.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"))
        self._widget.PlaceObus8_5.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PlaceObus8_5.setMask(mask.mask())
        self._widget.PlaceObus8_5.setMouseTracking(True)       
        self._widget.PlaceObus8_5.installEventFilter(self)
        #obus6
        self._widget.PlaceObus8_6.setGeometry(h8o1posex+51,h8o1posey-63,71,26)
        self._widget.PlaceObus8_6.clicked.connect(self.press_obus8_6_button)
        self._widget.PlaceObus8_6.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"))
        self._widget.PlaceObus8_6.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PlaceObus8_6.setMask(mask.mask())
        self._widget.PlaceObus8_6.setMouseTracking(True)       
        self._widget.PlaceObus8_6.installEventFilter(self)
        #obus7
        self._widget.PlaceObus8_7.setGeometry(h8o1posex+51,h8o1posey-109,71,26)
        self._widget.PlaceObus8_7.clicked.connect(self.press_obus8_7_button)
        self._widget.PlaceObus8_7.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"))
        self._widget.PlaceObus8_7.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PlaceObus8_7.setMask(mask.mask())
        self._widget.PlaceObus8_7.setMouseTracking(True)       
        self._widget.PlaceObus8_7.installEventFilter(self)
        #obus8
        self._widget.PlaceObus8_8.setGeometry(h8o1posex+51,h8o1posey-150,71,26)
        self._widget.PlaceObus8_8.clicked.connect(self.press_obus8_8_button)
        self._widget.PlaceObus8_8.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"))
        self._widget.PlaceObus8_8.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png")
        self._widget.PlaceObus8_8.setMask(mask.mask())
        self._widget.PlaceObus8_8.setMouseTracking(True)       
        self._widget.PlaceObus8_8.installEventFilter(self)
        
        #huevera16
        h16o1posex=60
        h16o1posey=610
        #obus1
        self._widget.PlaceObus16_1.setGeometry(h16o1posex,h16o1posey,51,19)
        self._widget.PlaceObus16_1.clicked.connect(self.press_obus16_1_button)
        self._widget.PlaceObus16_1.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.PlaceObus16_1.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PlaceObus16_1.setMask(mask.mask())
        self._widget.PlaceObus16_1.setMouseTracking(True)       
        self._widget.PlaceObus16_1.installEventFilter(self)
        #obus2
        self._widget.PlaceObus16_2.setGeometry(h16o1posex,h16o1posey-22,51,19)
        self._widget.PlaceObus16_2.clicked.connect(self.press_obus16_2_button)
        self._widget.PlaceObus16_2.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.PlaceObus16_2.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PlaceObus16_2.setMask(mask.mask())
        self._widget.PlaceObus16_2.setMouseTracking(True)       
        self._widget.PlaceObus16_2.installEventFilter(self)
        #obus3
        self._widget.PlaceObus16_3.setGeometry(h16o1posex,h16o1posey-42,51,19)
        self._widget.PlaceObus16_3.clicked.connect(self.press_obus16_3_button)
        self._widget.PlaceObus16_3.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.PlaceObus16_3.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PlaceObus16_3.setMask(mask.mask())
        self._widget.PlaceObus16_3.setMouseTracking(True)       
        self._widget.PlaceObus16_3.installEventFilter(self)
        #obus4
        self._widget.PlaceObus16_4.setGeometry(h16o1posex,h16o1posey-63,51,19)
        self._widget.PlaceObus16_4.clicked.connect(self.press_obus16_4_button)
        self._widget.PlaceObus16_4.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.PlaceObus16_4.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PlaceObus16_4.setMask(mask.mask())
        self._widget.PlaceObus16_4.setMouseTracking(True)       
        self._widget.PlaceObus16_4.installEventFilter(self)
        #obus5
        self._widget.PlaceObus16_5.setGeometry(h16o1posex,h16o1posey-86,51,19)
        self._widget.PlaceObus16_5.clicked.connect(self.press_obus16_5_button)
        self._widget.PlaceObus16_5.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.PlaceObus16_5.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PlaceObus16_5.setMask(mask.mask())
        self._widget.PlaceObus16_5.setMouseTracking(True)       
        self._widget.PlaceObus16_5.installEventFilter(self)
        #obus6
        self._widget.PlaceObus16_6.setGeometry(h16o1posex,h16o1posey-109,51,19)
        self._widget.PlaceObus16_6.clicked.connect(self.press_obus16_6_button)
        self._widget.PlaceObus16_6.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.PlaceObus16_6.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PlaceObus16_6.setMask(mask.mask())
        self._widget.PlaceObus16_6.setMouseTracking(True)       
        self._widget.PlaceObus16_6.installEventFilter(self)
        #obus7
        self._widget.PlaceObus16_7.setGeometry(h16o1posex,h16o1posey-132,51,19)
        self._widget.PlaceObus16_7.clicked.connect(self.press_obus16_7_button)
        self._widget.PlaceObus16_7.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.PlaceObus16_7.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PlaceObus16_7.setMask(mask.mask())
        self._widget.PlaceObus16_7.setMouseTracking(True)       
        self._widget.PlaceObus16_7.installEventFilter(self)
        #obus8
        self._widget.PlaceObus16_8.setGeometry(h16o1posex,h16o1posey-152,51,19)
        self._widget.PlaceObus16_8.clicked.connect(self.press_obus16_8_button)
        self._widget.PlaceObus16_8.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
        self._widget.PlaceObus16_8.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png")
        self._widget.PlaceObus16_8.setMask(mask.mask())
        self._widget.PlaceObus16_8.setMouseTracking(True)       
        self._widget.PlaceObus16_8.installEventFilter(self)
        #obus9
        self._widget.PlaceObus16_9.setGeometry(h16o1posex+74,h16o1posey,51,19)
        self._widget.PlaceObus16_9.clicked.connect(self.press_obus16_9_button)
        self._widget.PlaceObus16_9.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.PlaceObus16_9.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PlaceObus16_9.setMask(mask.mask())
        self._widget.PlaceObus16_9.setMouseTracking(True)       
        self._widget.PlaceObus16_9.installEventFilter(self)
        #obus10
        self._widget.PlaceObus16_10.setGeometry(h16o1posex+74,h16o1posey-22,51,19)
        self._widget.PlaceObus16_10.clicked.connect(self.press_obus16_10_button)
        self._widget.PlaceObus16_10.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.PlaceObus16_10.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PlaceObus16_10.setMask(mask.mask())
        self._widget.PlaceObus16_10.setMouseTracking(True)       
        self._widget.PlaceObus16_10.installEventFilter(self)
        #obus11
        self._widget.PlaceObus16_11.setGeometry(h16o1posex+74,h16o1posey-42,51,19)
        self._widget.PlaceObus16_11.clicked.connect(self.press_obus16_11_button)
        self._widget.PlaceObus16_11.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.PlaceObus16_11.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PlaceObus16_11.setMask(mask.mask())
        self._widget.PlaceObus16_11.setMouseTracking(True)       
        self._widget.PlaceObus16_11.installEventFilter(self)
        #obus12
        self._widget.PlaceObus16_12.setGeometry(h16o1posex+74,h16o1posey-63,51,19)
        self._widget.PlaceObus16_12.clicked.connect(self.press_obus16_12_button)
        self._widget.PlaceObus16_12.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.PlaceObus16_12.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PlaceObus16_12.setMask(mask.mask())
        self._widget.PlaceObus16_12.setMouseTracking(True)       
        self._widget.PlaceObus16_12.installEventFilter(self)
        #obus13
        self._widget.PlaceObus16_13.setGeometry(h16o1posex+74,h16o1posey-86,51,19)
        self._widget.PlaceObus16_13.clicked.connect(self.press_obus16_13_button)
        self._widget.PlaceObus16_13.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.PlaceObus16_13.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PlaceObus16_13.setMask(mask.mask())
        self._widget.PlaceObus16_13.setMouseTracking(True)       
        self._widget.PlaceObus16_13.installEventFilter(self)
        #obus14
        self._widget.PlaceObus16_14.setGeometry(h16o1posex+74,h16o1posey-109,51,19)
        self._widget.PlaceObus16_14.clicked.connect(self.press_obus16_14_button)
        self._widget.PlaceObus16_14.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.PlaceObus16_14.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PlaceObus16_14.setMask(mask.mask())
        self._widget.PlaceObus16_14.setMouseTracking(True)       
        self._widget.PlaceObus16_14.installEventFilter(self)
        #obus15
        self._widget.PlaceObus16_15.setGeometry(h16o1posex+74,h16o1posey-132,51,19)
        self._widget.PlaceObus16_15.clicked.connect(self.press_obus16_15_button)
        self._widget.PlaceObus16_15.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.PlaceObus16_15.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PlaceObus16_15.setMask(mask.mask())
        self._widget.PlaceObus16_15.setMouseTracking(True)       
        self._widget.PlaceObus16_15.installEventFilter(self)
        #obus16
        self._widget.PlaceObus16_16.setGeometry(h16o1posex+74,h16o1posey-152,51,19)
        self._widget.PlaceObus16_16.clicked.connect(self.press_obus16_16_button)
        self._widget.PlaceObus16_16.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
        self._widget.PlaceObus16_16.hide()
        mask = QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png")
        self._widget.PlaceObus16_16.setMask(mask.mask())
        self._widget.PlaceObus16_16.setMouseTracking(True)       
        self._widget.PlaceObus16_16.installEventFilter(self)
        
                
        
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
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Deadman service not available', QMessageBox.Ok)
                
        #Default: angle activated
        try:
                angle_mode_service=rospy.ServiceProxy(srv_angle_mode, SetBool)
                ret = angle_mode_service(True)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Angle Mode service not available', QMessageBox.Ok)
        #Default: tool orientation reference deactivated
        try:
                toolOrientation_service=rospy.ServiceProxy(srv_rel_tool, SetBool)
                ret = toolOrientation_service(False)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Tool Orientation service not available', QMessageBox.Ok)

        
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
        #Variable para almacenar el ultimo obus seleccionado
        self.last_obus_selected_pick = -1
        self.last_obus_selected_place = -1


    #filtro para detectar el raton y ponerlo verde si está el cursor encima o dejarlo blanco si no
    def eventFilter(self, object, event):               
        if not KUKA_AUT :
            #huevera 16
            if finger_type == 1 :
                if event.type() == QtCore.QEvent.HoverEnter:                                
                    #obuses hacia abajo (los de arriba [1-8])                    
                    if ((object == self._widget.PlaceObus16_1 and not Place_Obus_16_1)
                    or (object == self._widget.PlaceObus16_2 and not Place_Obus_16_2)
                    or (object == self._widget.PlaceObus16_3 and not Place_Obus_16_3)
                    or (object == self._widget.PlaceObus16_4 and not Place_Obus_16_4)
                    or (object == self._widget.PlaceObus16_5 and not Place_Obus_16_5)
                    or (object == self._widget.PlaceObus16_6 and not Place_Obus_16_6)
                    or (object == self._widget.PlaceObus16_7 and not Place_Obus_16_7)
                    or (object == self._widget.PlaceObus16_8 and not Place_Obus_16_8)
                    or (object ==self._widget.PickObus16_1 and not Pick_Obus_16_1)
                    or (object ==self._widget.PickObus16_2 and not Pick_Obus_16_2)
                    or (object ==self._widget.PickObus16_3 and not Pick_Obus_16_3)
                    or (object ==self._widget.PickObus16_4 and not Pick_Obus_16_4)
                    or (object ==self._widget.PickObus16_5 and not Pick_Obus_16_5)
                    or (object ==self._widget.PickObus16_6 and not Pick_Obus_16_6)
                    or (object ==self._widget.PickObus16_7 and not Pick_Obus_16_7)
                    or (object ==self._widget.PickObus16_8 and not Pick_Obus_16_8)
                    or (object ==self._widget.PickObus16_9 and not Pick_Obus_16_9)
                    or (object ==self._widget.PickObus16_10 and not Pick_Obus_16_10)
                    ) :
                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51P.png"))

                    #obuses hacia arriba
                    elif (
                        (object == self._widget.PlaceObus16_9 and not Place_Obus_16_9)
                        or (object == self._widget.PlaceObus16_10 and not Place_Obus_16_10)
                        or (object == self._widget.PlaceObus16_11 and not Place_Obus_16_11)
                        or (object == self._widget.PlaceObus16_12 and not Place_Obus_16_12)
                        or (object == self._widget.PlaceObus16_13 and not Place_Obus_16_13)
                        or (object == self._widget.PlaceObus16_14 and not Place_Obus_16_14)
                        or (object == self._widget.PlaceObus16_15 and not Place_Obus_16_15)
                        or (object == self._widget.PlaceObus16_16 and not Place_Obus_16_16)
                        or (object == self._widget.PickObus16_11 and not Pick_Obus_16_11)
                        or (object == self._widget.PickObus16_12 and not Pick_Obus_16_12)
                        or (object == self._widget.PickObus16_13 and not Pick_Obus_16_13)
                        or (object == self._widget.PickObus16_14 and not Pick_Obus_16_14)
                        or (object == self._widget.PickObus16_15 and not Pick_Obus_16_15)
                        or (object == self._widget.PickObus16_16 and not Pick_Obus_16_16)
                        or (object == self._widget.PickObus16_17 and not Pick_Obus_16_17)
                        or (object == self._widget.PickObus16_18 and not Pick_Obus_16_18)
                        or (object == self._widget.PickObus16_19 and not Pick_Obus_16_19)
                        or (object == self._widget.PickObus16_20 and not Pick_Obus_16_20)
                    ):
                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51P.png"))
                if event.type() == QtCore.QEvent.HoverLeave:
                    #obuses hacia abajo (los de arriba [1-8])
                    if ((object == self._widget.PlaceObus16_1 and not Place_Obus_16_1)
                    or (object == self._widget.PlaceObus16_2 and not Place_Obus_16_2)
                    or (object == self._widget.PlaceObus16_3 and not Place_Obus_16_3)
                    or (object == self._widget.PlaceObus16_4 and not Place_Obus_16_4)
                    or (object == self._widget.PlaceObus16_5 and not Place_Obus_16_5)
                    or (object == self._widget.PlaceObus16_6 and not Place_Obus_16_6)
                    or (object == self._widget.PlaceObus16_7 and not Place_Obus_16_7)
                    or (object == self._widget.PlaceObus16_8 and not Place_Obus_16_8)
                    or (object ==self._widget.PickObus16_1 and not Pick_Obus_16_1)
                    or (object ==self._widget.PickObus16_2 and not Pick_Obus_16_2)
                    or (object ==self._widget.PickObus16_3 and not Pick_Obus_16_3)
                    or (object ==self._widget.PickObus16_4 and not Pick_Obus_16_4)
                    or (object ==self._widget.PickObus16_5 and not Pick_Obus_16_5)
                    or (object ==self._widget.PickObus16_6 and not Pick_Obus_16_6)
                    or (object ==self._widget.PickObus16_7 and not Pick_Obus_16_7)
                    or (object ==self._widget.PickObus16_8 and not Pick_Obus_16_8)
                    or (object ==self._widget.PickObus16_9 and not Pick_Obus_16_9)
                    or (object ==self._widget.PickObus16_10 and not Pick_Obus_16_10)
                    ) :
                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"))
                    #obuses hacia arriba
                    elif (
                        (object == self._widget.PlaceObus16_9 and not Place_Obus_16_9)
                        or (object == self._widget.PlaceObus16_10 and not Place_Obus_16_10)
                        or (object == self._widget.PlaceObus16_11 and not Place_Obus_16_11)
                        or (object == self._widget.PlaceObus16_12 and not Place_Obus_16_12)
                        or (object == self._widget.PlaceObus16_13 and not Place_Obus_16_13)
                        or (object == self._widget.PlaceObus16_14 and not Place_Obus_16_14)
                        or (object == self._widget.PlaceObus16_15 and not Place_Obus_16_15)
                        or (object == self._widget.PlaceObus16_16 and not Place_Obus_16_16)
                        or (object == self._widget.PickObus16_11 and not Pick_Obus_16_11)
                        or (object == self._widget.PickObus16_12 and not Pick_Obus_16_12)
                        or (object == self._widget.PickObus16_13 and not Pick_Obus_16_13)
                        or (object == self._widget.PickObus16_14 and not Pick_Obus_16_14)
                        or (object == self._widget.PickObus16_15 and not Pick_Obus_16_15)
                        or (object == self._widget.PickObus16_16 and not Pick_Obus_16_16)
                        or (object == self._widget.PickObus16_17 and not Pick_Obus_16_17)
                        or (object == self._widget.PickObus16_18 and not Pick_Obus_16_18)
                        or (object == self._widget.PickObus16_19 and not Pick_Obus_16_19)
                        or (object == self._widget.PickObus16_20 and not Pick_Obus_16_20)
                    ):
                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"))
            #huevera 8
            if finger_type == 2 :
                if event.type() == QtCore.QEvent.HoverEnter:
                    #obuses hacia abajo (los de arriba [1-4])
                    if ((object == self._widget.PlaceObus8_1 and not Place_Obus_8_1)
                    or (object == self._widget.PlaceObus8_2 and not Place_Obus_8_2)
                    or (object == self._widget.PlaceObus8_3 and not Place_Obus_8_3)
                    or (object == self._widget.PlaceObus8_4 and not Place_Obus_8_4)
                    or (object == self._widget.PickObus8_1 and not Pick_Obus_8_1)
                    or (object == self._widget.PickObus8_2 and not Pick_Obus_8_2)
                    or (object == self._widget.PickObus8_3 and not Pick_Obus_8_3)
                    or (object == self._widget.PickObus8_4 and not Pick_Obus_8_4)
                    or (object == self._widget.PickObus8_5 and not Pick_Obus_8_5)
                    or (object == self._widget.PickObus8_6 and not Pick_Obus_8_6)
                    or (object == self._widget.PickObus8_7 and not Pick_Obus_8_7)
                    ):
                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba26x71P.png"))
                    #obuses hacia arriba
                    elif (
                        (object == self._widget.PlaceObus8_5 and not Place_Obus_8_5)
                        or (object == self._widget.PlaceObus8_6 and not Place_Obus_8_6)
                        or (object == self._widget.PlaceObus8_7 and not Place_Obus_8_7)
                        or (object == self._widget.PlaceObus8_8 and not Place_Obus_8_8)
                        or (object == self._widget.PickObus8_8 and not Pick_Obus_8_8)
                        or (object == self._widget.PickObus8_9 and not Pick_Obus_8_9)
                        or (object == self._widget.PickObus8_10 and not Pick_Obus_8_10)
                        or (object == self._widget.PickObus8_11 and not Pick_Obus_8_11)
                        or (object == self._widget.PickObus8_12 and not Pick_Obus_8_12)
                        or (object == self._widget.PickObus8_13 and not Pick_Obus_8_13)
                        or (object == self._widget.PickObus8_14 and not Pick_Obus_8_14)
                    ):
                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo26x71P.png"))
                if event.type() == QtCore.QEvent.HoverLeave:
                    #obuses hacia abajo (los de arriba [1-4])
                    if ((object == self._widget.PlaceObus8_1 and not Place_Obus_8_1)
                    or (object == self._widget.PlaceObus8_2 and not Place_Obus_8_2)
                    or (object == self._widget.PlaceObus8_3 and not Place_Obus_8_3)
                    or (object == self._widget.PlaceObus8_4 and not Place_Obus_8_4)
                    or (object == self._widget.PickObus8_1 and not Pick_Obus_8_1)
                    or (object == self._widget.PickObus8_2 and not Pick_Obus_8_2)
                    or (object == self._widget.PickObus8_3 and not Pick_Obus_8_3)
                    or (object == self._widget.PickObus8_4 and not Pick_Obus_8_4)
                    or (object == self._widget.PickObus8_5 and not Pick_Obus_8_5)
                    or (object == self._widget.PickObus8_6 and not Pick_Obus_8_6)
                    or (object == self._widget.PickObus8_7 and not Pick_Obus_8_7)
                    ):
                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"))
                    #obuses hacia arriba
                    elif (
                        (object == self._widget.PlaceObus8_5 and not Place_Obus_8_5)
                        or (object == self._widget.PlaceObus8_6 and not Place_Obus_8_6)
                        or (object == self._widget.PlaceObus8_7 and not Place_Obus_8_7)
                        or (object == self._widget.PlaceObus8_8 and not Place_Obus_8_8)
                        or (object == self._widget.PickObus8_8 and not Pick_Obus_8_8)
                        or (object == self._widget.PickObus8_9 and not Pick_Obus_8_9)
                        or (object == self._widget.PickObus8_10 and not Pick_Obus_8_10)
                        or (object == self._widget.PickObus8_11 and not Pick_Obus_8_11)
                        or (object == self._widget.PickObus8_12 and not Pick_Obus_8_12)
                        or (object == self._widget.PickObus8_13 and not Pick_Obus_8_13)
                        or (object == self._widget.PickObus8_14 and not Pick_Obus_8_14)
                    ):
                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"))        
            #huevera 4
            if finger_type == 3 :
                if event.type() == QtCore.QEvent.HoverEnter:
                    if (
                        (object == self._widget.PlaceObus4_1 and not Place_Obus_4_1)
                        or (object == self._widget.PlaceObus4_2 and not Place_Obus_4_2)
                        or (object == self._widget.PlaceObus4_3 and not Place_Obus_4_3)
                        or (object == self._widget.PlaceObus4_4 and not Place_Obus_4_4)
                        or (object == self._widget.PickObus4_1 and not Pick_Obus_4_1)
                        or (object == self._widget.PickObus4_2 and not Pick_Obus_4_2)
                        or (object == self._widget.PickObus4_3 and not Pick_Obus_4_3)
                        or (object == self._widget.PickObus4_4 and not Pick_Obus_4_4)
                        or (object == self._widget.PickObus4_5 and not Pick_Obus_4_5)
                    ):
                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo37x101P.png"))
                if event.type() == QtCore.QEvent.HoverLeave:
                    if (
                        (object == self._widget.PlaceObus4_1 and not Place_Obus_4_1)
                        or (object == self._widget.PlaceObus4_2 and not Place_Obus_4_2)
                        or (object == self._widget.PlaceObus4_3 and not Place_Obus_4_3)
                        or (object == self._widget.PlaceObus4_4 and not Place_Obus_4_4)
                        or (object == self._widget.PickObus4_1 and not Pick_Obus_4_1)
                        or (object == self._widget.PickObus4_2 and not Pick_Obus_4_2)
                        or (object == self._widget.PickObus4_3 and not Pick_Obus_4_3)
                        or (object == self._widget.PickObus4_4 and not Pick_Obus_4_4)
                        or (object == self._widget.PickObus4_5 and not Pick_Obus_4_5)
                    ):
                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"))
            #huevera 2
            if finger_type == 4 :
                if event.type() == QtCore.QEvent.HoverEnter:
                    if (
                        (object == self._widget.PlaceObus2_1 and not Place_Obus_2_1)
                        or (object == self._widget.PlaceObus2_2 and not Place_Obus_2_2)
                        or (object == self._widget.PickObus2_1 and not Pick_Obus_2_1)
                        or (object == self._widget.PickObus2_2 and not Pick_Obus_2_2)
                        or (object == self._widget.PickObus2_3 and not Pick_Obus_2_3)
                        or (object == self._widget.PickObus2_4 and not Pick_Obus_2_4)
                    ):            
                        object.setIcon(QtGui.QIcon(PATH+"resource/images/rotated-symb_obus_abajo41x111P.png"))
                if event.type() == QtCore.QEvent.HoverLeave:
                    if (
                        (object == self._widget.PlaceObus2_1 and not Place_Obus_2_1)
                        or (object == self._widget.PlaceObus2_2 and not Place_Obus_2_2)
                        or (object == self._widget.PickObus2_1 and not Pick_Obus_2_1)
                        or (object == self._widget.PickObus2_2 and not Pick_Obus_2_2)
                        or (object == self._widget.PickObus2_3 and not Pick_Obus_2_3)
                        or (object == self._widget.PickObus2_4 and not Pick_Obus_2_4)
                    ):
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
        self._widget.resetPositions_Button_place.setEnabled(False)
        self._widget.resetPositions_Button_pick.setEnabled(False)
        self._widget.undoPositions_Button_place.setEnabled(False)
        self._widget.undoPositions_Button_pick.setEnabled(False)
        self._widget.calibre_comboBox.setEnabled(False)
        self._widget.Finger_Adjust_Button.setEnabled(False)
        self._widget.PlaceObus2_1.setEnabled(False)
        self._widget.PlaceObus2_2.setEnabled(False)
        self._widget.PlaceObus4_1.setEnabled(False)
        self._widget.PlaceObus4_2.setEnabled(False)
        self._widget.PlaceObus4_3.setEnabled(False)
        self._widget.PlaceObus4_4.setEnabled(False)
        self._widget.PlaceObus8_1.setEnabled(False)
        self._widget.PlaceObus8_2.setEnabled(False)
        self._widget.PlaceObus8_3.setEnabled(False)
        self._widget.PlaceObus8_4.setEnabled(False)
        self._widget.PlaceObus8_5.setEnabled(False)
        self._widget.PlaceObus8_6.setEnabled(False)
        self._widget.PlaceObus8_7.setEnabled(False)
        self._widget.PlaceObus8_8.setEnabled(False)
        self._widget.PlaceObus16_1.setEnabled(False)
        self._widget.PlaceObus16_2.setEnabled(False)
        self._widget.PlaceObus16_3.setEnabled(False)
        self._widget.PlaceObus16_4.setEnabled(False)
        self._widget.PlaceObus16_5.setEnabled(False)
        self._widget.PlaceObus16_6.setEnabled(False)
        self._widget.PlaceObus16_7.setEnabled(False)
        self._widget.PlaceObus16_8.setEnabled(False)
        self._widget.PlaceObus16_9.setEnabled(False)
        self._widget.PlaceObus16_10.setEnabled(False)
        self._widget.PlaceObus16_11.setEnabled(False)
        self._widget.PlaceObus16_12.setEnabled(False)
        self._widget.PlaceObus16_13.setEnabled(False)
        self._widget.PlaceObus16_14.setEnabled(False)
        self._widget.PlaceObus16_15.setEnabled(False)
        self._widget.PlaceObus16_16.setEnabled(False)
        for i in range(1, 21):
            name_method='PickObus16'+'_'+str(i)
            test_method=getattr(self._widget, name_method)
            test_method.setEnabled(False)
        for i in range(1, 15):
            name_method='PickObus8'+'_'+str(i)
            test_method=getattr(self._widget, name_method)
            test_method.setEnabled(False)
        for i in range(1, 6):
            name_method='PickObus4'+'_'+str(i)
            test_method=getattr(self._widget, name_method)
            test_method.setEnabled(False)
        for i in range(1, 5):
            name_method='PickObus2'+'_'+str(i)
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
        self._widget.resetPositions_Button_place.setEnabled(True)
        self._widget.resetPositions_Button_pick.setEnabled(True)
        self._widget.undoPositions_Button_place.setEnabled(True)
        self._widget.undoPositions_Button_pick.setEnabled(True)
        self._widget.calibre_comboBox.setEnabled(True)
        self._widget.joy_comboBox.setEnabled(True)
        for i in range(1, 21):
            name_method='PickObus16'+'_'+str(i)
            test_method=getattr(self._widget, name_method)
            test_method.setEnabled(True)
        for i in range(1, 15):
            name_method='PickObus8'+'_'+str(i)
            test_method=getattr(self._widget, name_method)
            test_method.setEnabled(True)
        for i in range(1, 6):
            name_method='PickObus4'+'_'+str(i)
            test_method=getattr(self._widget, name_method)
            test_method.setEnabled(True)
        for i in range(1, 5):
            name_method='PickObus2'+'_'+str(i)
            test_method=getattr(self._widget, name_method)
            test_method.setEnabled(True)
        
        if(origin_pick==0):	
                self._widget.PlaceObus16_1.setEnabled(True)
                self._widget.PlaceObus16_2.setEnabled(True)
                self._widget.PlaceObus16_3.setEnabled(True)
                self._widget.PlaceObus16_4.setEnabled(True)
                self._widget.PlaceObus16_5.setEnabled(True)
                self._widget.PlaceObus16_6.setEnabled(True)
                self._widget.PlaceObus16_7.setEnabled(True)
                self._widget.PlaceObus16_8.setEnabled(True)
                self._widget.PlaceObus16_9.setEnabled(True)
                self._widget.PlaceObus16_10.setEnabled(True)
                self._widget.PlaceObus16_11.setEnabled(True)
                self._widget.PlaceObus16_12.setEnabled(True)
                self._widget.PlaceObus16_13.setEnabled(True)
                self._widget.PlaceObus16_14.setEnabled(True)
                self._widget.PlaceObus16_15.setEnabled(True)
                self._widget.PlaceObus16_16.setEnabled(True)
                self._widget.PlaceObus2_1.setEnabled(True)
                self._widget.PlaceObus2_2.setEnabled(True)
                self._widget.PlaceObus4_1.setEnabled(True)
                self._widget.PlaceObus4_2.setEnabled(True)
                self._widget.PlaceObus4_3.setEnabled(True)
                self._widget.PlaceObus4_4.setEnabled(True)
                self._widget.PlaceObus8_1.setEnabled(True)
                self._widget.PlaceObus8_2.setEnabled(True)
                self._widget.PlaceObus8_3.setEnabled(True)
                self._widget.PlaceObus8_4.setEnabled(True)
                self._widget.PlaceObus8_5.setEnabled(True)
                self._widget.PlaceObus8_6.setEnabled(True)
                self._widget.PlaceObus8_7.setEnabled(True)
                self._widget.PlaceObus8_8.setEnabled(True)
        elif((finger_type==3 or finger_type==4) and (origin_pick==2 or origin_pick==4)):
                self._widget.PlaceObus4_3.setEnabled(True)
                self._widget.PlaceObus4_4.setEnabled(True)
                self._widget.PlaceObus2_2.setEnabled(True)
        elif((finger_type==3 or finger_type==4) and (origin_pick==1 or origin_pick==3)):
                self._widget.PlaceObus4_1.setEnabled(True)
                self._widget.PlaceObus4_2.setEnabled(True)
                self._widget.PlaceObus2_1.setEnabled(True)
                
        elif(origin_pick==2 or origin_pick==3):
            self._widget.PlaceObus16_9.setEnabled(True)
            self._widget.PlaceObus16_10.setEnabled(True)
            self._widget.PlaceObus16_11.setEnabled(True)
            self._widget.PlaceObus16_12.setEnabled(True)
            self._widget.PlaceObus16_5.setEnabled(True)
            self._widget.PlaceObus16_6.setEnabled(True)
            self._widget.PlaceObus16_7.setEnabled(True)
            self._widget.PlaceObus16_8.setEnabled(True)
            self._widget.PlaceObus8_3.setEnabled(True)
            self._widget.PlaceObus8_4.setEnabled(True)
            self._widget.PlaceObus8_5.setEnabled(True)
            self._widget.PlaceObus8_6.setEnabled(True)
        elif(origin_pick==1 or origin_pick==4):
            self._widget.PlaceObus16_13.setEnabled(True)
            self._widget.PlaceObus16_14.setEnabled(True)
            self._widget.PlaceObus16_15.setEnabled(True)
            self._widget.PlaceObus16_16.setEnabled(True)
            self._widget.PlaceObus16_1.setEnabled(True)
            self._widget.PlaceObus16_2.setEnabled(True)
            self._widget.PlaceObus16_3.setEnabled(True)
            self._widget.PlaceObus16_4.setEnabled(True)
            self._widget.PlaceObus8_1.setEnabled(True)
            self._widget.PlaceObus8_2.setEnabled(True)
            self._widget.PlaceObus8_7.setEnabled(True)
            self._widget.PlaceObus8_8.setEnabled(True)
        ##PLACE
        for i in range(1,17):
                name='Place_Obus_16_'+str(i)
                if(globals()[name]==True):                        
                        name_method='PlaceObus16_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)    
        for i in range(1,9):
                name='Place_Obus_8_'+str(i)
                if(globals()[name]==True):                      
                        name_method='PlaceObus8_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
        for i in range(1,5):
                name='Place_Obus_4_'+str(i)
                if(globals()[name]==True):                    
                        name_method='PlaceObus4_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
        for i in range(1,3):
                name='Place_Obus_2_'+str(i)
                if(globals()[name]==True):
                        #icon=QtGui.QIcon()
                        #icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
                        name_method='PlaceObus2_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        #test_method.setIcon(icon)
        ##PICK
        for i in range(1,21):
                name='Pick_Obus_16_'+str(i)
                if(globals()[name]==True):
                        #icon=QtGui.QIcon()
                        #icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                        name_method='PickObus16_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        #test_method.setIcon(icon)
        for i in range(1,15):
                name='Pick_Obus_8_'+str(i)
                if(globals()[name]==True):
                        #icon=QtGui.QIcon()
                        #icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
                        name_method='PickObus8_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        #test_method.setIcon(icon)
        for i in range(1,6):
                name='Pick_Obus_4_'+str(i)
                if(globals()[name]==True):
                        #icon=QtGui.QIcon()
                        #icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
                        name_method='PickObus4_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        #test_method.setIcon(icon)
        for i in range(1,5):
                name='Pick_Obus_2_'+str(i)
                if(globals()[name]==True):
                        #icon=QtGui.QIcon()
                        #icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
                        name_method='PickObus2_'+str(i)
                        test_method=getattr(self._widget, name_method)
                        test_method.removeEventFilter(self)
                        #test_method.setIcon(icon)
        
    def press_reset_positions_button_place(self):
        global Place_Obus_2_1, Place_Obus_2_2, Place_Obus_4_1, Place_Obus_4_2, Place_Obus_4_3, Place_Obus_4_4, Place_Obus_8_1, Place_Obus_8_2, Place_Obus_8_3, Place_Obus_8_4, Place_Obus_8_5, Place_Obus_8_6, Place_Obus_8_7, Place_Obus_8_8,Place_Obus_16_1, Place_Obus_16_2, Place_Obus_16_3, Place_Obus_16_4, Place_Obus_16_5, Place_Obus_16_6, Place_Obus_16_7, Place_Obus_16_8,Place_Obus_16_9, Place_Obus_16_10, Place_Obus_16_11, Place_Obus_16_12, Place_Obus_16_13, Place_Obus_16_14, Place_Obus_16_15, Place_Obus_16_16
        #Hueveras de 2
        Place_Obus_2_1=False
        Place_Obus_2_2=False
        #Hueveras de 4
        Place_Obus_4_1=False
        Place_Obus_4_2=False
        Place_Obus_4_3=False
        Place_Obus_4_4=False
        #Hueveras de 8
        Place_Obus_8_1=False
        Place_Obus_8_2=False
        Place_Obus_8_3=False
        Place_Obus_8_4=False
        Place_Obus_8_5=False
        Place_Obus_8_6=False
        Place_Obus_8_7=False
        Place_Obus_8_8=False
        #Hueveras de 16
        Place_Obus_16_1=False
        Place_Obus_16_2=False
        Place_Obus_16_3=False
        Place_Obus_16_4=False
        Place_Obus_16_5=False
        Place_Obus_16_6=False
        Place_Obus_16_7=False
        Place_Obus_16_8=False
        Place_Obus_16_9=False
        Place_Obus_16_10=False
        Place_Obus_16_11=False
        Place_Obus_16_12=False
        Place_Obus_16_13=False
        Place_Obus_16_14=False
        Place_Obus_16_15=False
        Place_Obus_16_16=False
        for i in range(1,9):
            icon=QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
            name_method='PlaceObus16_'+str(i)
            test_method=getattr(self._widget, name_method)
            test_method.setIcon(icon)
            test_method.installEventFilter(self)
        for i in range(9,17):
            icon=QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
            name_method='PlaceObus16_'+str(i)
            test_method=getattr(self._widget, name_method)
            test_method.setIcon(icon)
            test_method.installEventFilter(self)
        for i in range(1,5):
            icon=QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"), QtGui.QIcon.Disabled)
            name_method='PlaceObus8_'+str(i)
            test_method=getattr(self._widget, name_method)
            test_method.setIcon(icon)
            test_method.installEventFilter(self)
        for i in range(5,9):
            icon=QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"), QtGui.QIcon.Disabled)
            name_method='PlaceObus8_'+str(i)
            test_method=getattr(self._widget, name_method)
            test_method.setIcon(icon)
            test_method.installEventFilter(self)
        for i in range(1,5):
            icon=QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"), QtGui.QIcon.Disabled)
            name_method='PlaceObus4_'+str(i)
            test_method=getattr(self._widget, name_method)
            test_method.setIcon(icon)
            test_method.installEventFilter(self)
        for i in range(1,3):
            icon=QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png"), QtGui.QIcon.Disabled)
            name_method='PlaceObus4_'+str(i)
            test_method=getattr(self._widget, name_method)
            test_method.setIcon(icon)
            test_method.installEventFilter(self)

        self.last_obus_selected_place = -1
                
    def press_reset_positions_button_pick(self):
        global Pick_Obus_2_1, Pick_Obus_2_2, Pick_Obus_2_3, Pick_Obus_2_4, Pick_Obus_4_1, Pick_Obus_4_2, Pick_Obus_4_3, Pick_Obus_4_4
        global Pick_Obus_8_1, Pick_Obus_8_2, Pick_Obus_8_3, Pick_Obus_8_4, Pick_Obus_8_5, Pick_Obus_8_6, Pick_Obus_8_7, Pick_Obus_8_8,Pick_Obus_8_9, Pick_Obus_8_10, Pick_Obus_8_11, Pick_Obus_8_12, Pick_Obus_8_13, Pick_Obus_8_14
        global Pick_Obus_16_1, Pick_Obus_16_2, Pick_Obus_16_3, Pick_Obus_16_4, Pick_Obus_16_5, Pick_Obus_16_6, Pick_Obus_16_7, Pick_Obus_16_8,Pick_Obus_16_9, Pick_Obus_16_10
        global Pick_Obus_16_11, Pick_Obus_16_12, Pick_Obus_16_13, Pick_Obus_16_14, Pick_Obus_16_15, Pick_Obus_16_16, Pick_Obus_16_17, Pick_Obus_16_18, Pick_Obus_16_19, Pick_Obus_16_20
        #Hueveras de 2
        Pick_Obus_2_1=False
        Pick_Obus_2_2=False
        Pick_Obus_2_3=False
        Pick_Obus_2_4=False
        #Hueveras de 4
        Pick_Obus_4_1=False
        Pick_Obus_4_2=False
        Pick_Obus_4_3=False
        Pick_Obus_4_4=False
        #Hueveras de 8
        Pick_Obus_8_1=False
        Pick_Obus_8_2=False
        Pick_Obus_8_3=False
        Pick_Obus_8_4=False
        Pick_Obus_8_5=False
        Pick_Obus_8_6=False
        Pick_Obus_8_7=False
        Pick_Obus_8_8=False
        Pick_Obus_8_9=False
        Pick_Obus_8_10=False
        Pick_Obus_8_11=False
        Pick_Obus_8_12=False
        Pick_Obus_8_13=False
        Pick_Obus_8_14=False
        #Hueveras de 16
        Pick_Obus_16_1=False
        Pick_Obus_16_2=False
        Pick_Obus_16_3=False
        Pick_Obus_16_4=False
        Pick_Obus_16_5=False
        Pick_Obus_16_6=False
        Pick_Obus_16_7=False
        Pick_Obus_16_8=False
        Pick_Obus_16_9=False
        Pick_Obus_16_10=False
        Pick_Obus_16_11=False
        Pick_Obus_16_12=False
        Pick_Obus_16_13=False
        Pick_Obus_16_14=False
        Pick_Obus_16_15=False
        Pick_Obus_16_16=False
        Pick_Obus_16_17=False
        Pick_Obus_16_18=False
        Pick_Obus_16_19=False
        Pick_Obus_16_20=False
        for i in range(1,11):
                name='Pick_Obus_16_'+str(i)
                icon=QtGui.QIcon();
                icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                name_method='PickObus16_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.setIcon(icon)
                test_method.installEventFilter(self)
        for i in range(11,21):
                name='Pick_Obus_16_'+str(i)
                icon=QtGui.QIcon();
                icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                name_method='PickObus16_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.setIcon(icon)
                test_method.installEventFilter(self)
        for i in range(1,8):
                name='Pick_Obus_8_'+str(i)
                icon=QtGui.QIcon();
                icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71.png"), QtGui.QIcon.Disabled)
                name_method='PickObus8_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.setIcon(icon)
                test_method.installEventFilter(self)
        for i in range(8,15):
                name='Pick_Obus_8_'+str(i)
                icon=QtGui.QIcon();
                icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71.png"), QtGui.QIcon.Disabled)
                name_method='PickObus8_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.setIcon(icon)
                test_method.installEventFilter(self)
        for i in range(1,6):
                name='Pick_Obus_4_'+str(i)
                icon=QtGui.QIcon();
                icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"), QtGui.QIcon.Disabled)
                name_method='PickObus4_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.setIcon(icon)
                test_method.installEventFilter(self)
        for i in range(1,5):
                name='Pick_Obus_2_'+str(i)
                icon=QtGui.QIcon();
                icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png"), QtGui.QIcon.Disabled)
                name_method='PickObus2_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.setIcon(icon)
                test_method.installEventFilter(self)

        self.last_obus_selected_pick = -1

    
    def select_icon(self,operation, obus_id,state):
        #operation es "pick" o "place"
        #obus_id es un vector donde la posicion [0] contiene el calibre y posicion [1] la posicion
        #state=0 normal - blanco
        #state=1 resaltado - verde
        #state=2 seleccionado - rojo
        calibre = int(obus_id[0])
        num = int(obus_id[1])
        print calibre
        print num
        if calibre == 2:
            return PATH+"resource/images/rotated-symb_obus_abajo41x111.png"
        elif calibre == 4:
            return PATH+"resource/images/rotated-symb_obus_abajo37x101.png"
        elif calibre == 8:
            if operation == 'pick':
                if num < 8:
                    if state == 0:
                        return PATH+"resource/images/rotated-symb_obus_arriba26x71.png"
                    if state == 1:
                        return PATH+"resource/images/rotated-symb_obus_arriba26x71P.png"
                    if state == 2:
                        return PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"
                if num >= 8:
                    if state == 0:
                        return PATH+"resource/images/rotated-symb_obus_abajo26x71.png"
                    if state == 1:
                        return PATH+"resource/images/rotated-symb_obus_abajo26x71P.png"
                    if state == 2:
                        return PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"
            elif operation == 'place':
                if num < 5:
                    if state == 0:
                        return PATH+"resource/images/rotated-symb_obus_arriba26x71.png"
                    if state == 1:
                        return PATH+"resource/images/rotated-symb_obus_arriba26x71P.png"
                    if state == 2:
                        return PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"
                if num >= 5:
                    if state == 0:
                        return PATH+"resource/images/rotated-symb_obus_abajo26x71.png"
                    if state == 1:
                        return PATH+"resource/images/rotated-symb_obus_abajo26x71P.png"
                    if state == 2:
                        return PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"
        elif calibre == 16:
            if operation == 'pick':
                if num < 11:
                    if state == 0:
                        return PATH+"resource/images/rotated-symb_obus_arriba19x51.png"
                    if state == 1:
                        return PATH+"resource/images/rotated-symb_obus_arriba19x51P.png"
                    if state == 2:
                        return PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"
                elif num >= 11:
                    if state == 0:
                        return PATH+"resource/images/rotated-symb_obus_abajo19x51.png"
                    if state == 1:
                        return PATH+"resource/images/rotated-symb_obus_abajo19x51P.png"
                    if state == 2:
                        return PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"
            elif operation == 'place':
                if num < 9:
                    if state == 0:
                        return PATH+"resource/images/rotated-symb_obus_arriba19x51.png"
                    if state == 1:
                        return PATH+"resource/images/rotated-symb_obus_arriba19x51P.png"
                    if state == 2:
                        return PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"
                elif num >= 9:
                    if state == 0:
                        return PATH+"resource/images/rotated-symb_obus_abajo19x51.png"
                    if state == 1:
                        return PATH+"resource/images/rotated-symb_obus_abajo19x51P.png"
                    if state == 2:
                        return PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"



    def press_undo_positions_button_pick(self):
        print 'undo_pick'
        if self.last_obus_selected_pick == -1: return
        #last pick_obus_selected format 'YY_XX'
        #Pick_Obus_YY_XX = False
        #Update global variable state
        srt ="Pick_Obus_%s" %self.last_obus_selected_pick
        globals()[srt] = False
        obus_id = self.last_obus_selected_pick.split("_")
        path = self.select_icon('pick', obus_id, 0)
        #change icon
        icon=QtGui.QIcon();        
        icon = QtGui.QIcon(path)
        name_method="PickObus%s" %self.last_obus_selected_pick
        test_method=getattr(self._widget, name_method)
        test_method.setIcon(icon)
        test_method.installEventFilter(self)

        self.last_obus_selected_pick = -1

    def press_undo_positions_button_place(self):
        print 'undo_place'
        if self.last_obus_selected_place == -1: return
        #last place_obus_selected format 'YY_XX'
        #Place_Obus_YY_XX = False
        #Update global variable state
        srt ="Place_Obus_%s" %self.last_obus_selected_place
        globals()[srt] = False
        obus_id = self.last_obus_selected_place.split("_")
        path = self.select_icon('place', obus_id, 0)
        #change icon
        icon=QtGui.QIcon();
        icon = QtGui.QIcon(path)
        name_method="PlaceObus%s" %self.last_obus_selected_place
        test_method=getattr(self._widget, name_method)
        test_method.setIcon(icon)
        test_method.installEventFilter(self)

        self.last_obus_selected_place = -1

    def callback_moving(self, data):
        global KUKA_AUT, first_time_moving_kuka
        #print 'CB:moving_received:',data.data
        if data.data == True :
            if not KUKA_AUT:
                KUKA_AUT=True
            if first_time_moving_kuka:
                    self._widget.mode_label.setText("AUTOMATIC")
                    self.desactivate_buttons()
                    first_time_moving_kuka = False
                            
        else:
            if KUKA_AUT:    
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
        if ret == QMessageBox.Ok:
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)
                ret=placed_abs_service(H2O1_Pose_x, H2O1_Pose_y, H2O1_Pose_z, table_pose_a, H2O1_Pose_b, H2O1_Pose_c)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(table_pose_x, table_pose_y, table_pose_z, table_pose_a, table_pose_b, table_pose_c)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available.', QMessageBox.Ok)

#Pick buttons obus 2
    def press_pick_obus2_1_button(self):
        global Pick_Obus_2_1, pos_z_kuka, pos_a_kuka, origin_pick # KUKA_AUT,
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_2_1=True
            self.last_obus_selected_pick='2_1'
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus2_1.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x21', QMessageBox.Ok)

    def press_pick_obus2_2_button(self):
        global Pick_Obus_2_2, pos_z_kuka, pos_a_kuka, origin_pick # KUKA_AUT,
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_2_2=True
            self.last_obus_selected_pick='2_2'
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus2_2.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x22', QMessageBox.Ok)

    def press_pick_obus2_3_button(self):
        global Pick_Obus_2_3, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_2_3=True
            self.last_obus_selected_pick='2_3'
            origin_pick=2      
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus2_3.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x23', QMessageBox.Ok)

    def press_pick_obus2_4_button(self):
        global Pick_Obus_2_4, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_2_4=True
            self.last_obus_selected_pick='2_4'
            origin_pick=2      
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus2_4.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x24', QMessageBox.Ok)
#Pick buttons obus 4
    def press_pick_obus4_1_button(self):
        global Pick_Obus_4_1, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_4_1=True
            self.last_obus_selected_pick='4_1'
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus4_1.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x24', QMessageBox.Ok)
    def press_pick_obus4_2_button(self):
        global Pick_Obus_4_2, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_4_2=True
            self.last_obus_selected_pick='4_2'
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus4_2.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x42', QMessageBox.Ok)

    def press_pick_obus4_3_button(self):
        global Pick_Obus_4_3, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_4_3=True
            self.last_obus_selected_pick='4_3'
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus4_3.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x43', QMessageBox.Ok)

    def press_pick_obus4_4_button(self):
        global Pick_Obus_4_4, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_4_4=True
            self.last_obus_selected_pick='4_4'
            origin_pick=2 
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus4_4.setIcon(icon)     
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x44', QMessageBox.Ok)

    def press_pick_obus4_5_button(self):
        global Pick_Obus_4_5, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_4_5=True
            self.last_obus_selected_pick='4_5'
            origin_pick=2 
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus4_5.setIcon(icon)     
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x45', QMessageBox.Ok)

#Pick buttons obus 8
    def press_pick_obus8_1_button(self):
        global Pick_Obus_8_1, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_8_1=True
            self.last_obus_selected_pick='8_1'
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_1.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x81', QMessageBox.Ok)

    def press_pick_obus8_2_button(self):
        global Pick_Obus_8_2, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_8_2=True
            self.last_obus_selected_pick='8_2'
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_2.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x82', QMessageBox.Ok)

    def press_pick_obus8_3_button(self):
        global Pick_Obus_8_3, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_8_3=True
            self.last_obus_selected_pick='8_3'
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_3.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x83', QMessageBox.Ok)

    def press_pick_obus8_4_button(self):
        global Pick_Obus_8_4, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_8_4=True
            self.last_obus_selected_pick='8_4'
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_4.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x84', QMessageBox.Ok)

    def press_pick_obus8_5_button(self):
        global Pick_Obus_8_5, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_8_5=True
            self.last_obus_selected_pick='8_5'
            origin_pick=2   
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_5.setIcon(icon)   
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x85', QMessageBox.Ok)

    def press_pick_obus8_6_button(self):
        global Pick_Obus_8_6, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            Pick_Obus_8_6=True
            self.last_obus_selected_pick='8_6'
            origin_pick=2 
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_6.setIcon(icon)     
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x86', QMessageBox.Ok)

    def press_pick_obus8_7_button(self):
        global Pick_Obus_8_7, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            Pick_Obus_8_7=True
            self.last_obus_selected_pick='8_7'
            origin_pick=2 
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_7.setIcon(icon)     
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x87', QMessageBox.Ok)

    def press_pick_obus8_8_button(self):
        global Pick_Obus_8_8, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_8_8=True
            self.last_obus_selected_pick='8_8'
            origin_pick=3
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_8.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x88', QMessageBox.Ok)
                
    def press_pick_obus8_9_button(self):
        global Pick_Obus_8_9, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_8_9=True
            self.last_obus_selected_pick='8_9'
            origin_pick=3
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_9.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x89', QMessageBox.Ok)

    def press_pick_obus8_10_button(self):
        global Pick_Obus_8_10, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_8_10=True
            self.last_obus_selected_pick='8_10'
            origin_pick=3
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_10.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x810', QMessageBox.Ok)
                
    def press_pick_obus8_11_button(self):
        global Pick_Obus_8_11, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_8_11=True
            self.last_obus_selected_pick='8_11'
            origin_pick=3
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_11.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x811', QMessageBox.Ok)

    def press_pick_obus8_12_button(self):
        global Pick_Obus_8_12, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            Pick_Obus_8_12=True
            self.last_obus_selected_pick='8_12'
            origin_pick=4      
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_12.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x812', QMessageBox.Ok)

    def press_pick_obus8_13_button(self):
        global Pick_Obus_8_13, pos_z_kuka, pos_a_kuka, origin_pick#, KUKA_AUT
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            Pick_Obus_8_13=True
            self.last_obus_selected_pick='8_13'
            origin_pick=4   
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_13.setIcon(icon)   
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x813', QMessageBox.Ok)

    def press_pick_obus8_14_button(self):
        global Pick_Obus_8_14, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=4 
            Pick_Obus_8_14=True
            self.last_obus_selected_pick='8_14'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus8_14.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x814', QMessageBox.Ok)

#Pick buttons obus 16
    def press_pick_obus16_1_button(self):
        global Pick_Obus_16_1, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=1
            Pick_Obus_16_1=True
            self.last_obus_selected_pick='16_1'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_1.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x161', QMessageBox.Ok)

    def press_pick_obus16_2_button(self):
        global Pick_Obus_16_2, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=1
            Pick_Obus_16_2=True
            self.last_obus_selected_pick='16_2'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_2.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x162', QMessageBox.Ok)

    def press_pick_obus16_3_button(self):
        global Pick_Obus_16_3, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_16_3=True
            self.last_obus_selected_pick='16_3'
            origin_pick=1
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_3.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x163', QMessageBox.Ok)

    def press_pick_obus16_4_button(self):
        global Pick_Obus_16_4, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=1
            Pick_Obus_16_4=True
            self.last_obus_selected_pick='16_4'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_4.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x164', QMessageBox.Ok)

    def press_pick_obus16_5_button(self):
        global Pick_Obus_16_5, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=1
            Pick_Obus_16_5=True
            self.last_obus_selected_pick='16_5'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_5.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x165', QMessageBox.Ok)

    def press_pick_obus16_6_button(self):
        global Pick_Obus_16_6, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=2
            Pick_Obus_16_6=True
            self.last_obus_selected_pick='16_6'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_6.setIcon(icon) 
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x166', QMessageBox.Ok)

    def press_pick_obus16_7_button(self):
        global Pick_Obus_16_7, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=2
            Pick_Obus_16_7=True
            self.last_obus_selected_pick='16_7'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_7.setIcon(icon)      
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x167', QMessageBox.Ok)

    def press_pick_obus16_8_button(self):
        global Pick_Obus_16_8, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=2 
            Pick_Obus_16_8=True
            self.last_obus_selected_pick='16_8'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_8.setIcon(icon)   
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x168', QMessageBox.Ok)

    def press_pick_obus16_9_button(self):
        global Pick_Obus_16_9, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=2      
            Pick_Obus_16_9=True
            self.last_obus_selected_pick='16_9'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_9.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x169', QMessageBox.Ok)

    def press_pick_obus16_10_button(self):
        global Pick_Obus_16_10, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=2  
            Pick_Obus_16_10=True
            self.last_obus_selected_pick='16_10'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_10.setIcon(icon)  
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x1610', QMessageBox.Ok)

    def press_pick_obus16_11_button(self):
        global Pick_Obus_16_11, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=3
            Pick_Obus_16_11=True
            self.last_obus_selected_pick='16_11'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_11.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x1611', QMessageBox.Ok)

    def press_pick_obus16_12_button(self):
        global Pick_Obus_16_12, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=3
            Pick_Obus_16_12=True
            self.last_obus_selected_pick='16_12'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_12.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x1612', QMessageBox.Ok)

    def press_pick_obus16_13_button(self):
        global Pick_Obus_16_13, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            Pick_Obus_16_13=True
            self.last_obus_selected_pick='16_13'
            origin_pick=3
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_13.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x1613', QMessageBox.Ok)

    def press_pick_obus16_14_button(self):
        global Pick_Obus_16_14, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=3
            Pick_Obus_16_14=True
            self.last_obus_selected_pick='16_14'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_14.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x1614', QMessageBox.Ok)

    def press_pick_obus16_15_button(self):
        global Pick_Obus_16_15, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            origin_pick=3
            Pick_Obus_16_15=True
            self.last_obus_selected_pick='16_15'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_15.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick1_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick1_A1_A6_service(pick_A1, pick_left_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x1615', QMessageBox.Ok)

    def press_pick_obus16_16_button(self):
        global Pick_Obus_16_16, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=4    
            Pick_Obus_16_16=True
            self.last_obus_selected_pick='16_16'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_16.setIcon(icon) 
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x1616', QMessageBox.Ok)

    def press_pick_obus16_17_button(self):
        global Pick_Obus_16_17, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=4    
            Pick_Obus_16_17=True
            self.last_obus_selected_pick='16_17'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_17.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x1617', QMessageBox.Ok)

    def press_pick_obus16_18_button(self):
        global Pick_Obus_16_18, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=4      
            Pick_Obus_16_18=True
            self.last_obus_selected_pick='16_18'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_18.setIcon(icon)
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x1618', QMessageBox.Ok)

    def press_pick_obus16_19_button(self):
        global Pick_Obus_16_19, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=4 
            Pick_Obus_16_19=True
            self.last_obus_selected_pick='16_19'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_19.setIcon(icon)     
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x1619', QMessageBox.Ok)

    def press_pick_obus16_20_button(self):
        global Pick_Obus_16_20, KUKA_AUT, pos_z_kuka, pos_a_kuka, origin_pick
        ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot is going to move autonomously', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok: 
            origin_pick=4  
            Pick_Obus_16_20=True
            self.last_obus_selected_pick='16_20'
            icon = QtGui.QIcon();
            icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
            self._widget.PickObus16_20.setIcon(icon)   
            #Call service to move robot up and then to pre place pose, should be slow
            try:
                picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                pick_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=pick_A1_A6_service(pick_A1, pick_right_A6)
                if ret == True:
                    CURRENT_STATE=STATE_MOVING_TO_PLACE
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self._widget, "WARNING!", 'Movement Service not available. Code=0x1620', QMessageBox.Ok)

    #pressing obuses para place
    #huevera2
    #obus1
    def press_obus2_1_button(self):
        global Place_Obus_2_1
        if(Place_Obus_2_1==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_2_1=False
        if(Place_Obus_2_1==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen 
                    Place_Obus_2_1=True
                    self.last_obus_selected_place='2_1'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus2_1.setIcon(icon)
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
                        ret = placed_abs_service(H2O1_Pose_x, H2O1_Pose_y, H2O1_Pose_z, H2O1_Pose_a, H2O1_Pose_b, H2O1_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon()
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus2_1.setIcon(icon)
                    
    #obus2
    def press_obus2_2_button(self):
        global Place_Obus_2_2
        if(Place_Obus_2_2==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_2_2=False
        if(Place_Obus_2_2==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_2_2=True
                    self.last_obus_selected_place='2_2'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus2_2.setIcon(icon)
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
                        ret = placed_abs_service(H2O2_Pose_x, H2O2_Pose_y, H2O2_Pose_z, H2O2_Pose_a, H2O2_Pose_b, H2O2_Pose_c)
                        KUKA_AUT=True
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon()
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo41x111.png"), QtGui.QIcon.Disabled)
                   # self._widget.PlaceObus2_2.setIcon(icon)
    #huevera4
    #obus1
    def press_obus4_1_button(self):
        global Place_Obus_4_1
        if(Place_Obus_4_1==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_4_1=False
        if(Place_Obus_4_1==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Place_Obus_4_1=True
                    self.last_obus_selected_place='4_1'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus4_1.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        #ret = placed_abs_service(H4O1_Pose_x, H4O1_Pose_y, H4O1_Pose_z, H4O1_Pose_a, H4O1_Pose_b, H4O1_Pose_c)
                        #KUKA_AUT=True
                        #self.sleep_loop(2)
                        #while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon()
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus4_1.setIcon(icon)
    #obus2
    def press_obus4_2_button(self):
        global Place_Obus_4_2
        if(Place_Obus_4_2==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_4_2=False
        if(Place_Obus_4_2==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:    
                    #cambia el color de la imagen
                    Place_Obus_4_2=True
                    self.last_obus_selected_place='4_2'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus4_2.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
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
                    #self._widget.PlaceObus4_2.setIcon(icon)
    #obus3
    def press_obus4_3_button(self):
        global Place_Obus_4_3
        if(Place_Obus_4_3==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_4_3=False
        if(Place_Obus_4_3==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Place_Obus_4_3=True
                    self.last_obus_selected_place='4_3'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus4_3.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
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
                    #self._widget.PlaceObus4_3.setIcon(icon)
    #obus4
    def press_obus4_4_button(self):
        global Place_Obus_4_4
        if(Place_Obus_4_4==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_4_4=False
        if(Place_Obus_4_4==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_4_4=True
                    self.last_obus_selected_place='4_4'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo37x101PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus4_4.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
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
                    #self._widget.PlaceObus4_4.setIcon(icon)
    #huevera8
    #obus1
    def press_obus8_1_button(self):
        global Place_Obus_8_1
        if(Place_Obus_8_1==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_8_1=False
        if(Place_Obus_8_1==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_8_1=True
                    self.last_obus_selected_place='8_1'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus8_1.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
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
                    #self._widget.PlaceObus8_1.setIcon(icon)
    #obus2
    def press_obus8_2_button(self):
        global Place_Obus_8_2
        if(Place_Obus_8_2==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_8_2=False
        if(Place_Obus_8_2==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_8_2=True
                    self.last_obus_selected_place='8_2'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus8_2.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
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
                    #self._widget.PlaceObus8_2.setIcon(icon)
    #obus3
    def press_obus8_3_button(self):
        global Place_Obus_8_3
        if(Place_Obus_8_3==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_8_3=False
        if(Place_Obus_8_3==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_8_3=True
                    self.last_obus_selected_place='8_3'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus8_3.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
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
                    #self._widget.PlaceObus8_3.setIcon(icon)
    #obus4
    def press_obus8_4_button(self):
        global Place_Obus_8_4
        if(Place_Obus_8_4==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_8_4=False
        if(Place_Obus_8_4==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_8_4=True
                    self.last_obus_selected_place='8_4'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus8_4.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
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
                    #self._widget.PlaceObus8_4.setIcon(icon)
    #obus5
    def press_obus8_5_button(self):
        global Place_Obus_8_5
        if(Place_Obus_8_5==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_8_5=False
        if(Place_Obus_8_5==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_8_5=True
                    self.last_obus_selected_place='8_5'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus8_5.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
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
                    #self._widget.PlaceObus8_5.setIcon(icon)
    #obus6
    def press_obus8_6_button(self):
        global Place_Obus_8_6
        if(Place_Obus_8_6==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_8_6=False
        if(Place_Obus_8_6==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Place_Obus_8_6=True
                    self.last_obus_selected_place='8_6'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus8_6.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
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
                    #self._widget.PlaceObus8_6.setIcon(icon)
    #obus7
    def press_obus8_7_button(self):
        global Place_Obus_8_7
        if(Place_Obus_8_7==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_8_7=False
        if(Place_Obus_8_7==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_8_7=True
                    self.last_obus_selected_place='8_7'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus8_7.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
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
                    #self._widget.PlaceObus8_7.setIcon(icon)
    #obus8
    def press_obus8_8_button(self):
        global Place_Obus_8_8
        if(Place_Obus_8_8==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_8_8=False
        if(Place_Obus_8_8==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Place_Obus_8_8=True
                    self.last_obus_selected_place='8_8'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo26x71PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus8_8.setIcon(icon)
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
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
                    #self._widget.PlaceObus8_8.setIcon(icon)
    #huevera16
    #obus1
    def press_obus16_1_button(self):
        global Place_Obus_16_1
        if(Place_Obus_16_1==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_1=False
        if(Place_Obus_16_1==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_16_1=True
                    self.last_obus_selected_place='16_1'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_1.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)
                        ret2 = placed_abs_service(H16O1_Pose_x, H16O1_Pose_y, H16O1_Pose_z, H16O1_Pose_a, H16O1_Pose_b, H16O1_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_1.setIcon(icon)
                
    #obus2
    def press_obus16_2_button(self):
        global Place_Obus_16_2
        if(Place_Obus_16_2==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_2=False
        if(Place_Obus_16_2==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_16_2=True
                    self.last_obus_selected_place='16_2'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_2.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O2_Pose_x, H16O2_Pose_y, H16O2_Pose_z, H16O2_Pose_a, H16O2_Pose_b, H16O2_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_2.setIcon(icon)            
    #obus3
    def press_obus16_3_button(self):
        global Place_Obus_16_3
        if(Place_Obus_16_3==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_3=False
        if(Place_Obus_16_3==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_16_3=True
                    self.last_obus_selected_place='16_3'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_3.setIcon(icon)                        
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O3_Pose_x, H16O3_Pose_y, H16O3_Pose_z, H16O3_Pose_a, H16O3_Pose_b, H16O3_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_3.setIcon(icon)            
    #obus4
    def press_obus16_4_button(self):
        global Place_Obus_16_4
        if(Place_Obus_16_4==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_4=False
        if(Place_Obus_16_4==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_16_4=True
                    self.last_obus_selected_place='16_4'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_4.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O4_Pose_x, H16O4_Pose_y, H16O4_Pose_z, H16O4_Pose_a, H16O4_Pose_b, H16O4_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_4.setIcon(icon)            
    #obus5
    def press_obus16_5_button(self):
        global Place_Obus_16_5
        if(Place_Obus_16_5==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_5=False
        if(Place_Obus_16_5==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_16_5=True
                    self.last_obus_selected_place='16_5'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_5.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O5_Pose_x, H16O5_Pose_y, H16O5_Pose_z, H16O5_Pose_a, H16O5_Pose_b, H16O5_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_5.setIcon(icon)            
    #obus6
    def press_obus16_6_button(self):
        global Place_Obus_16_6
        if(Place_Obus_16_6==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_6=False
        if(Place_Obus_16_6==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Place_Obus_16_6=True
                    self.last_obus_selected_place='16_6'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_6.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O6_Pose_x, H16O6_Pose_y, H16O6_Pose_z, H16O6_Pose_a, H16O6_Pose_b, H16O6_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_6.setIcon(icon)            
    #obus7
    def press_obus16_7_button(self):
        global Place_Obus_16_7
        if(Place_Obus_16_7==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_7=False
        if(Place_Obus_16_7==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_16_7=True
                    self.last_obus_selected_place='16_7'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_7.setIcon(icon)            
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O7_Pose_x, H16O7_Pose_y, H16O7_Pose_z, H16O7_Pose_a, H16O7_Pose_b, H16O7_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_7.setIcon(icon) 
    #obus8
    def press_obus16_8_button(self):
        global Place_Obus_16_8
        if(Place_Obus_16_8==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_8=False
        if(Place_Obus_16_8==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Place_Obus_16_8=True
                    self.last_obus_selected_place='16_8'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_8.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O8_Pose_x, H16O8_Pose_y, H16O8_Pose_z, H16O8_Pose_a, H16O8_Pose_b, H16O8_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_8.setIcon(icon) 
    #obus9
    def press_obus16_9_button(self):
        global Place_Obus_16_9
        if(Place_Obus_16_9==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_9=False
        if(Place_Obus_16_9==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_16_9=True
                    self.last_obus_selected_place='16_9'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_9.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O9_Pose_x, H16O9_Pose_y, H16O9_Pose_z, H16O9_Pose_a, H16O9_Pose_b, H16O9_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_9.setIcon(icon) 
    #obus10
    def press_obus16_10_button(self):
        global Place_Obus_16_10
        if(Place_Obus_16_10==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_10=False
        if(Place_Obus_16_10==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_16_10=True
                    self.last_obus_selected_place='16_10'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_10.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O10_Pose_x, H16O10_Pose_y, H16O10_Pose_z, H16O10_Pose_a, H16O10_Pose_b, H16O10_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_10.setIcon(icon) 
    #obus11
    def press_obus16_11_button(self):
        global Place_Obus_16_11
        if(Place_Obus_16_11==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_11=False
        if(Place_Obus_16_11==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_16_11=True
                    self.last_obus_selected_place='16_11'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_11.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O11_Pose_x, H16O11_Pose_y, H16O11_Pose_z, H16O11_Pose_a, H16O11_Pose_b, H16O11_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_11.setIcon(icon) 
    #obus12
    def press_obus16_12_button(self):
        global Place_Obus_16_12
        if(Place_Obus_16_12==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_12=False
        if(Place_Obus_16_12==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_16_12=True
                    self.last_obus_selected_place='16_12'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_12.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_left_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O12_Pose_x, H16O12_Pose_y, H16O12_Pose_z, H16O12_Pose_a, H16O12_Pose_b, H16O12_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_12.setIcon(icon) 
    #obus13
    def press_obus16_13_button(self):
        global Place_Obus_16_13
        if(Place_Obus_16_13==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_13=False
        if(Place_Obus_16_13==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_16_13=True
                    self.last_obus_selected_place='16_13'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_13.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O13_Pose_x, H16O13_Pose_y, H16O13_Pose_z, H16O13_Pose_a, H16O13_Pose_b, H16O13_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_13.setIcon(icon) 
    #obus14
    def press_obus16_14_button(self):
        global Place_Obus_16_14
        if(Place_Obus_16_14==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_14=False
        if(Place_Obus_16_14==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Place_Obus_16_14=True
                    self.last_obus_selected_place='16_14'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_14.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O14_Pose_x, H16O14_Pose_y, H16O14_Pose_z, H16O14_Pose_a, H16O14_Pose_b, H16O14_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_14.setIcon(icon) 
    #obus15
    def press_obus16_15_button(self):
        global Place_Obus_16_15
        if(Place_Obus_16_15==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_15=False
        if(Place_Obus_16_15==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:
                    #cambia el color de la imagen
                    Place_Obus_16_15=True
                    self.last_obus_selected_place='16_15'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_15.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O15_Pose_x, H16O15_Pose_y, H16O15_Pose_z, H16O15_Pose_a, H16O15_Pose_b, H16O15_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_arriba19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_15.setIcon(icon) 
    #obus16
    def press_obus16_16_button(self):
        global Place_Obus_16_16
        if(Place_Obus_16_16==True):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Apparently that position is already picked.\n Do you still want to go?', QMessageBox.Ok, QMessageBox.Cancel)
                if(ret==QMessageBox.Ok):
                        Place_Obus_16_16=False
        if(Place_Obus_16_16==False):
                ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
                if ret == QMessageBox.Ok:           
                    #cambia el color de la imagen
                    Place_Obus_16_16=True
                    self.last_obus_selected_place='16_16'
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51PP.png"), QtGui.QIcon.Disabled)
                    self._widget.PlaceObus16_16.setIcon(icon) 
                    #llamara al servicio de mover
                    global KUKA_AUT
                    #Call service to move robot up and then to the pre-pick pose, should be fast
                    try:
                        placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                        ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        place_axes_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                        ret = place_axes_service(place_A1, place_right_A6)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                        placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                        ret = placed_abs_service(H16O16_Pose_x, H16O16_Pose_y, H16O16_Pose_z, H16O16_Pose_a, H16O16_Pose_b, H16O16_Pose_c)
                        #KUKA_AUT=True
                        self.sleep_loop(2)
                        while KUKA_AUT: self.sleep_loop(0.3)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    #y volvera a poner el color original
                    icon = QtGui.QIcon();
                    icon.addPixmap(QtGui.QPixmap(PATH+"resource/images/rotated-symb_obus_abajo19x51.png"), QtGui.QIcon.Disabled)
                    #self._widget.PlaceObus16_16.setIcon(icon) 
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
                #gripper_move_service(0.02,0,0,-0.15)
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
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                home_A1_A6_service=rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
                ret=home_A1_A6_service(0.0, 177)
                #KUKA_AUT=True
                self.sleep_loop(2)
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

    #################################################JOY SELECTION
    def joy_selected(self, index):
        if index == 0:
            print 'PS4 selected'
            command_string = "rosrun topic_tools mux_select mux_joy /kuka_pad/ps4_joy"
            os.system(command_string)
        elif index == 1:            
            print 'ITOWA selected'
            command_string = "rosrun topic_tools mux_select mux_joy /kuka_pad/itowa_joy"
            os.system(command_string)

    
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
            self._widget.PlaceObus2_1.hide()
            self._widget.PlaceObus2_2.hide()            
            self._widget.PlaceObus4_1.hide()
            self._widget.PlaceObus4_2.hide()
            self._widget.PlaceObus4_3.hide()
            self._widget.PlaceObus4_4.hide()
            self._widget.PlaceObus8_1.hide()
            self._widget.PlaceObus8_2.hide()
            self._widget.PlaceObus8_3.hide()
            self._widget.PlaceObus8_4.hide()
            self._widget.PlaceObus8_5.hide()
            self._widget.PlaceObus8_6.hide()
            self._widget.PlaceObus8_7.hide()
            self._widget.PlaceObus8_8.hide()            
            self._widget.PlaceObus16_1.hide()
            self._widget.PlaceObus16_2.hide()
            self._widget.PlaceObus16_3.hide()
            self._widget.PlaceObus16_4.hide()
            self._widget.PlaceObus16_5.hide()
            self._widget.PlaceObus16_6.hide()
            self._widget.PlaceObus16_7.hide()
            self._widget.PlaceObus16_8.hide()  
            self._widget.PlaceObus16_9.hide()
            self._widget.PlaceObus16_10.hide()
            self._widget.PlaceObus16_11.hide()
            self._widget.PlaceObus16_12.hide()
            self._widget.PlaceObus16_13.hide()
            self._widget.PlaceObus16_14.hide()
            self._widget.PlaceObus16_15.hide()
            self._widget.PlaceObus16_16.hide()   
            for i in range(1, 21):
                name_method='PickObus16'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.hide()
            for i in range(1, 15):
                name_method='PickObus8'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.hide()
            for i in range(1, 6):
                name_method='PickObus4'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.hide()
            for i in range(1, 5):
                name_method='PickObus2'+'_'+str(i)
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
            try:
                limit_cont_current_service(current_limit_1)
                limit_peak_current_service(current_limit_1)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))
                
            current_limit_picked = current_limit_1

            pixmap = QtGui.QPixmap(PATH+"resource/images/rotated-fondo_huevera_16.png")
            self._widget.background_plate.setPixmap(pixmap)
            pixmap_pick = QtGui.QPixmap(PATH+"resource/images/BoxPick_3.png")
            self._widget.background_plate_pick.setPixmap(pixmap_pick)
            self._widget.PlaceObus16_1.show()
            self._widget.PlaceObus16_2.show()
            self._widget.PlaceObus16_3.show()
            self._widget.PlaceObus16_4.show()
            self._widget.PlaceObus16_5.show()
            self._widget.PlaceObus16_6.show()
            self._widget.PlaceObus16_7.show()
            self._widget.PlaceObus16_8.show()  
            self._widget.PlaceObus16_9.show()
            self._widget.PlaceObus16_10.show()
            self._widget.PlaceObus16_11.show()
            self._widget.PlaceObus16_12.show()
            self._widget.PlaceObus16_13.show()
            self._widget.PlaceObus16_14.show()
            self._widget.PlaceObus16_15.show()
            self._widget.PlaceObus16_16.show()   
            self._widget.PlaceObus2_1.hide()
            self._widget.PlaceObus2_2.hide()
            self._widget.PlaceObus4_1.hide()
            self._widget.PlaceObus4_2.hide()
            self._widget.PlaceObus4_3.hide()
            self._widget.PlaceObus4_4.hide()            
            self._widget.PlaceObus8_1.hide()
            self._widget.PlaceObus8_2.hide()
            self._widget.PlaceObus8_3.hide()
            self._widget.PlaceObus8_4.hide()
            self._widget.PlaceObus8_5.hide()
            self._widget.PlaceObus8_6.hide()
            self._widget.PlaceObus8_7.hide()
            self._widget.PlaceObus8_8.hide()
            for i in range(1, 21):
                name_method='PickObus16'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.show()
            for i in range(1, 15):
                name_method='PickObus8'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.hide()
            for i in range(1, 6):
                name_method='PickObus4'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.hide()
            for i in range(1, 5):
                name_method='PickObus2'+'_'+str(i)
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
            try:
                limit_cont_current_service(current_limit_2)
                limit_peak_current_service(current_limit_2)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))

            current_limit_picked = current_limit_2


            pixmap = QtGui.QPixmap(PATH+"resource/images/rotated-fondo_huevera_8.png")
            self._widget.background_plate.setPixmap(pixmap)
            pixmap_pick = QtGui.QPixmap(PATH+"resource/images/BoxPick_3.png")
            self._widget.background_plate_pick.setPixmap(pixmap_pick)
            self._widget.PlaceObus2_1.hide()
            self._widget.PlaceObus2_2.hide()
            self._widget.PlaceObus4_1.hide()
            self._widget.PlaceObus4_2.hide()
            self._widget.PlaceObus4_3.hide()
            self._widget.PlaceObus4_4.hide()                        
            self._widget.PlaceObus8_1.show()
            self._widget.PlaceObus8_2.show()
            self._widget.PlaceObus8_3.show()
            self._widget.PlaceObus8_4.show()
            self._widget.PlaceObus8_5.show()
            self._widget.PlaceObus8_6.show()
            self._widget.PlaceObus8_7.show()
            self._widget.PlaceObus8_8.show()
            self._widget.PlaceObus16_1.hide()
            self._widget.PlaceObus16_2.hide()
            self._widget.PlaceObus16_3.hide()
            self._widget.PlaceObus16_4.hide()
            self._widget.PlaceObus16_5.hide()
            self._widget.PlaceObus16_6.hide()
            self._widget.PlaceObus16_7.hide()
            self._widget.PlaceObus16_8.hide()  
            self._widget.PlaceObus16_9.hide()
            self._widget.PlaceObus16_10.hide()
            self._widget.PlaceObus16_11.hide()
            self._widget.PlaceObus16_12.hide()
            self._widget.PlaceObus16_13.hide()
            self._widget.PlaceObus16_14.hide()
            self._widget.PlaceObus16_15.hide()
            self._widget.PlaceObus16_16.hide() 
            for i in range(1, 21):
                name_method='PickObus16'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.hide()
            for i in range(1, 15):
                name_method='PickObus8'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.show()
            for i in range(1, 6):
                name_method='PickObus4'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.hide()
            for i in range(1, 5):
                name_method='PickObus2'+'_'+str(i)
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
            try:
                limit_cont_current_service(current_limit_cont)
                limit_peak_current_service(current_limit_3)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))

            current_limit_picked = current_limit_3

            pixmap = QtGui.QPixmap(PATH+"resource/images/rotated-fondo_huevera_4.png")
            self._widget.background_plate.setPixmap(pixmap)
            pixmap_pick = QtGui.QPixmap(PATH+"resource/images/BoxPick_3.png")
            self._widget.background_plate_pick.setPixmap(pixmap_pick)
            self._widget.PlaceObus4_1.show()
            self._widget.PlaceObus4_2.show()
            self._widget.PlaceObus4_3.show()
            self._widget.PlaceObus4_4.show()
            self._widget.PlaceObus2_1.hide()
            self._widget.PlaceObus2_2.hide()
            self._widget.PlaceObus8_1.hide()
            self._widget.PlaceObus8_2.hide()
            self._widget.PlaceObus8_3.hide()
            self._widget.PlaceObus8_4.hide()
            self._widget.PlaceObus8_5.hide()
            self._widget.PlaceObus8_6.hide()
            self._widget.PlaceObus8_7.hide()
            self._widget.PlaceObus8_8.hide()              
            self._widget.PlaceObus16_1.hide()
            self._widget.PlaceObus16_2.hide()
            self._widget.PlaceObus16_3.hide()
            self._widget.PlaceObus16_4.hide()
            self._widget.PlaceObus16_5.hide()
            self._widget.PlaceObus16_6.hide()
            self._widget.PlaceObus16_7.hide()
            self._widget.PlaceObus16_8.hide()  
            self._widget.PlaceObus16_9.hide()
            self._widget.PlaceObus16_10.hide()
            self._widget.PlaceObus16_11.hide()
            self._widget.PlaceObus16_12.hide()
            self._widget.PlaceObus16_13.hide()
            self._widget.PlaceObus16_14.hide()
            self._widget.PlaceObus16_15.hide()
            self._widget.PlaceObus16_16.hide()
            for i in range(1, 21):
                name_method='PickObus16'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.hide()
            for i in range(1, 15):
                name_method='PickObus8'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.hide()
            for i in range(1, 6):
                name_method='PickObus4'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.show()
            for i in range(1, 5):
                name_method='PickObus2'+'_'+str(i)
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
            try:
                limit_cont_current_service(current_limit_cont)
                limit_peak_current_service(current_limit_4)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))

            current_limit_picked = current_limit_4
            
            pixmap = QtGui.QPixmap(PATH+"resource/images/rotated-fondo_huevera_2.png")
            self._widget.background_plate.setPixmap(pixmap)
            pixmap_pick = QtGui.QPixmap(PATH+"resource/images/BoxPick_3.png")
            self._widget.background_plate_pick.setPixmap(pixmap_pick)
            self._widget.PlaceObus2_1.show()
            self._widget.PlaceObus2_2.show()
            self._widget.PlaceObus4_1.hide()
            self._widget.PlaceObus4_2.hide()
            self._widget.PlaceObus4_3.hide()
            self._widget.PlaceObus4_4.hide()
            self._widget.PlaceObus8_1.hide()
            self._widget.PlaceObus8_2.hide()
            self._widget.PlaceObus8_3.hide()
            self._widget.PlaceObus8_4.hide()
            self._widget.PlaceObus8_5.hide()
            self._widget.PlaceObus8_6.hide()
            self._widget.PlaceObus8_7.hide()
            self._widget.PlaceObus8_8.hide()              
            self._widget.PlaceObus16_1.hide()
            self._widget.PlaceObus16_2.hide()
            self._widget.PlaceObus16_3.hide()
            self._widget.PlaceObus16_4.hide()
            self._widget.PlaceObus16_5.hide()
            self._widget.PlaceObus16_6.hide()
            self._widget.PlaceObus16_7.hide()
            self._widget.PlaceObus16_8.hide()  
            self._widget.PlaceObus16_9.hide()
            self._widget.PlaceObus16_10.hide()
            self._widget.PlaceObus16_11.hide()
            self._widget.PlaceObus16_12.hide()
            self._widget.PlaceObus16_13.hide()
            self._widget.PlaceObus16_14.hide()
            self._widget.PlaceObus16_15.hide()
            self._widget.PlaceObus16_16.hide()
            for i in range(1, 21):
                name_method='PickObus16'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.hide()
            for i in range(1, 15):
                name_method='PickObus8'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.hide()
            for i in range(1, 6):
                name_method='PickObus4'+'_'+str(i)
                test_method=getattr(self._widget, name_method)
                test_method.hide()
            for i in range(1, 5):
                name_method='PickObus2'+'_'+str(i)
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
        global angle_tool
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
                if(tool_current>current_limit_cont):
                        press_counter=press_counter+1
                else:
                        press_counter=0
                print abs(x_tool-old_pos_x)
                print 'current:'
                print tool_current
                print 'counter:'
                print press_counter
                
        press_counter=0
        while press_counter<5 :
                print press_counter
                gripper_move_service(x_tool+0.02,0,0,angle_tool)
                self.sleep_loop(0.15)
                if(tool_current>current_limit_cont or abs(x_tool-old_pos_x)<0.002):
                        press_counter=press_counter+1
                else:
                       press_counter=0
                       old_pos_x=x_tool
                print 'current:'
                print tool_current
                print 'counter:'
                print press_counter
        self.sleep_loop(0.5)
        self.activate_buttons()
        
        
    def callback_tool_state(self, data):
        global x_tool, angle_tool
        x_tool = data.position[2]
        angle_tool = data.position[3]
    
    def press_reset_robot_button(self):
        #command_string = "rosnode kill /kuka_pad/ps4_joystick; sleep 1; rosnode kill /kuka_pad/itowa_safe_joystick; sleep 1; rosnode kill /kuka_pad/robotnik_trajectory_pad_node; sleep 1; rosnode kill /kuka_robot/kuka_cartesian_hardware_interface; sleep 1; roslaunch kuka_robot_bringup kuka_robot_bringup_standalone.launch &"
        command_string = "rosnode kill /kuka_robot/kuka_cartesian_hardware_interface; sleep 1; ROS_NAMESPACE=kuka_robot roslaunch kuka_rsi_cartesian_hw_interface test_hardware_interface.launch &"
        os.system(command_string)
        
    def shutdown_plugin(self):
        print "Program finishing..."
        # TODO unregister all publishers here
        print 'closing all...'
        self.sub_robot_moving.unregister()
        self.sub_robot_pose.unregister()
        self.sub_tool_weight.unregister()
        self.sub_tool_current.unregister()
        self.sub_tool_force.unregister()
        self.sub_tool_status.unregister()
        self.sub_tool_force.unregister()
        self.sub_tool_status.unregister()
        self.sub_tool_state.unregister()
        #Stop nodes
        command_string = "rosnode kill /kuka_pad/itowa_safe_joystick; rosnode kill /kuka_pad/ps4_joystick; rosnode kill /kuka_pad/robotnik_trajectory_pad_node; rosnode kill /kuka_robot/kuka_cartesian_hardware_interface"        
        os.system(command_string)
        pass
    def sleep_loop(self,delay):
        loop = QtCore.QEventLoop()
        timer = QtCore.QTimer()
        timer.setInterval(delay*1000)
        timer.setSingleShot(True)
        timer.timeout.connect(loop.quit)
        timer.start()
        loop.exec_()
