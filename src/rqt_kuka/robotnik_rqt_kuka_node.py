#!/usr/bin/python

import os
import inspect

import rospy
import rospkg
import time 
import xacro
import subprocess
import sys
import QtCore
import QtGui

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialog, QFileDialog, QMessageBox, QTableWidgetItem
from std_msgs.msg import Bool, Float64, Float32
from sensor_msgs.msg import JointState
from robotnik_msgs.srv import home, set_odometry, set_CartesianEuler_pose, set_digital_output
from robotnik_msgs.msg import Cartesian_Euler_pose
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

TOOL_HOMED=False
KUKA_AUT=False
finger_type=0
#service names:
srv_name_move_abs_fast='/kuka_robot/setKukaAbsFast'
srv_name_move_abs_slow='/kuka_robot/setKukaAbs'
srv_name_move_rel_fast='/kuka_robot/setKukaRelFast'
srv_name_move_rel_slow='/kuka_robot/setKukaRel'
srv_tool_homing='/kuka_tool/robotnik_base_hw/home'
srv_finger_set_pose='/kuka_tool_finger_node/set_odometry' #robotnik_msgs.set.odometry
srv_digital_io='/kuka_tool/robotnik_base_hw/set_digital_output'

#topic names:
topic_cart_pose_kuka='/kuka_robot/cartesian_pos_kuka'
topic_kuka_moving='/kuka_robot/kuka_moving'
topic_tool_weight='/phidget_load/load_mean'
topic_current='/kuka_tool/robotnik_base_hw/current0'

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
        
        # add signals/slots
        #select obus calibre
        self._widget.calibre_comboBox.currentIndexChanged.connect(self.calibre_selected)
        #self._widget.calibre_comboBox.highlighted.connect(self.arm_activated)

        #Buttons
        self._widget.Home_Button.pressed.connect(self.press_homming_button)
        self._widget.Pick_Left_Button.pressed.connect(self.press_pick_left_button)
        self._widget.Pick_Right_Button.pressed.connect(self.press_pick_right_button)
        self._widget.Place_Left_Button.pressed.connect(self.press_place_left_button)
        self._widget.Place_Right_Button.pressed.connect(self.press_place_right_button)
        self._widget.Finger_Adjust_Button.pressed.connect(self.press_finger_adjust_button)
        self._widget.Tare_Button.pressed.connect(self.press_tare_button)
        self._widget.Tare_Reset_Button.pressed.connect(self.press_tare_reset_button)
        
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
        #displays
        #self._widget.weight_lcdNumber.pressed.connect(self.press_load_yaml)
        #self._widget.tool_force_lcdNumber.pressed.connect(self.press_save_yaml)
        
        
        #subscriber to robot state
        rospy.Subscriber(topic_kuka_moving, Bool, self.callback_moving)

        #subscriber to robot pose
        rospy.Subscriber(topic_cart_pose_kuka, Cartesian_Euler_pose, self.callback_robot_pose)     

        #subscriber to tool weight detected
        rospy.Subscriber(topic_tool_weight, Float64, self.callback_tool_weight)     

        #subscriber to tool current
        rospy.Subscriber(topic_current, Float32, self.callback_current) 
        
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            #self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
            self._widget.setWindowTitle("Robotnik Kuka Interface")
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        self._yaml_file = ""
        self._params = dict()
        self._name = "RqtKuka"
        
        self._keys_not_steps = ['arm_ip', 'arm_port', 'joint_names', 'group_name', 'action_ns']

    def desactivate_buttons(self):
        self._widget.Home_Button.setEnabled(False)
        self._widget.PickTest_Button.setEnabled(False)
        self._widget.Gripper_Homing_Button.setEnabled(False)
        self._widget.Pick_Right_Button.setEnabled(False)
        self._widget.Pick_Left_Button.setEnabled(False)
        self._widget.Place_Right_Button.setEnabled(False)
        self._widget.Place_Left_Button.setEnabled(False)

    def activate_buttons(self):
        self._widget.PickTest_Button.setEnabled(True)
        self._widget.Gripper_Homing_Button.setEnabled(True)
        self._widget.Home_Button.setEnabled(True)
        self._widget.Pick_Right_Button.setEnabled(True)
        self._widget.Pick_Left_Button.setEnabled(True)
        self._widget.Place_Right_Button.setEnabled(True)
        self._widget.Place_Left_Button.setEnabled(True)
        
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

    def callback_robot_pose(self, data):
		global pos_x_kuka, pos_y_kuka, pos_z_kuka, pos_a_kuka
		#print 'CB:robot_pose_received',data
		pos_x_kuka=data.x
		pos_y_kuka=data.y
		pos_z_kuka=data.z
		pos_a_kuka=data.A
		pos_b_kuka=data.B
		pos_c_kuka=data.C

    def callback_tool_weight(self, data):
		global weight_empty, weight_read
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
        
    def press_tool_homming(self):
		ret = QMessageBox.warning(self._widget, "WARNING!", 'Are you sure? \nBe sure there is no obus picked', QMessageBox.Ok, QMessageBox.Cancel)
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
			#	CURRENT_STATE=3
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
		#Call service to move robot up and then to pre place pose, should be slow
		try:
			picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
			ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
			KUKA_AUT=True
			while KUKA_AUT: self.sleep_loop(0.1)
			picked_abs_service = rospy.ServiceProxy(srv_name_move_abs_fast, set_CartesianEuler_pose)
			#if((pos_a_kuka<=180 and pos_a_kuka>=90)):
			#if(pos_a_kuka>=Prepick_angle_limit and pos_a_kuka<=Prepick_Pose_a_right):
			#	print "pass at 76 (-14-90)"
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
		global KUKA_AUT, pos_z_kuka
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
		global weight_empty
		weight_empty=weight_read

    def press_tare_reset_button(self):
		global weight_empty
		weight_empty = 0
    
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
            #desactivate all buttons
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
            self._widget.weight_limited.setText("1");
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
            self._widget.weight_limited.setText("2");
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
            self._widget.weight_limited.setText("4");
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
            
    def press_capture_button(self):
		print 'capture button'
		#msg = rospy.wait_for_message('',)

    def load_robot_description(self, gripper_model):
		command_string = "rosparam load ~/kuka_catkin_ws/src/kuka_experimental/kuka_robot_bringup/robot/bin/kr120toolv%d.urdf /robot_description" % gripper_model
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
