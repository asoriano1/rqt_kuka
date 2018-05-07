#!/usr/bin/python

import os
import inspect

import rospy
import rospkg
import time 
import xacro
import subprocess
import sys

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialog, QFileDialog, QMessageBox, QTableWidgetItem
from std_msgs.msg import Bool, Float64, Float32
from sensor_msgs.msg import JointState
from robotnik_msgs.srv import home, set_odometry, set_CartesianEuler_pose
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
#service names:
srv_name_move_abs_fast='/kuka_robot/setKukaAbsFast'
srv_name_move_abs_slow='/kuka_robot/setKukaAbs'
srv_name_move_rel_fast='/kuka_robot/setKukaRelFast'
srv_name_move_rel_slow='/kuka_robot/setKukaRel'
srv_tool_homing='/kuka_tool/robotnik_base_hw/home'
srv_finger_set_pose='/kuka_tool_finger_node/set_odometry' #robotnik_msgs.set.odometry

#topic names:
topic_cart_pose_kuka='/kuka_robot/cartesian_pos_kuka'
topic_kuka_moving='/kuka_robot/kuka_moving'
topic_tool_weight='/kuka_gauges/phidget_load/load_mean'
topic_current='/kuka_tool/robotnik_base_hw/current'

#Prepick Pose # tf.transformations.quaternion_from_euler(0, 0, th)
#Prepick_Pose=Pose(Point(100, 100, 100), Quaternion(0, 0, 0, 1))
#rosservice call /kuka_robot/setKukaAbs "{x: 1707.69, y: 235.42, z: 1435.39, A: -59.39, B: 0, C: -174}" 
#CAJA NEGRA
Preplace_Pose_x=1707.69
Preplace_Pose_y=235.42
Preplace_Pose_z=1435.39
Preplace_Pose_a=-50
Preplace_Pose_b=0#-0.21
Preplace_Pose_c=-174#178.41


#Preplace Pose
#Preplace_Pose=Pose(Point(400, 400, 100), Quaternion(0, 0, 0, 1))
#rosservice call /kuka_robot/setKukaAbs "{x: 255.69, y: 1704.42, z: 1475.39, A: 14.39, B: 0, C: -174}" 
#CAJA GRIS
Prepick_Pose_x=255.49
Prepick_Pose_y=1704.49
Prepick_Pose_z=1475.38
Prepick_Pose_a=-14.24#15.2
Prepick_Pose_b=0.0#-0.12
Prepick_Pose_c=174#178.73

#Homming Pose
Homming_Pose_x=255.49
Homming_Pose_y=1704.49
Homming_Pose_z=1475.38
Homming_Pose_a=-14.24#15.2
Homming_Pose_b=0.0#-0.12
Homming_Pose_c=174#178.73

#RollerBench Pose
RollerBench_Pose=Pose(Point(200, 200, 100), Quaternion(0, 0, 0, 1))
pos_z_kuka=0.0
weight_read=0.0
weight_empty=0.0

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
        self._widget.PrePick_Button.pressed.connect(self.press_prepick_button)
        self._widget.PrePlace_Button.pressed.connect(self.press_preplaced_button)
        
        self._widget.PickTest_Button.pressed.connect(self.press_picktest_button)
        self._widget.Gripper_Homing_Button.pressed.connect(self.press_tool_homming)
        self._widget.Gripper_Straighten_Button.pressed.connect(self.press_tool_straighten)

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
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        self._yaml_file = ""
        self._params = dict()
        self._name = "RqtKuka"
        
        self._keys_not_steps = ['arm_ip', 'arm_port', 'joint_names', 'group_name', 'action_ns']

    def desactivate_buttons(self):
        self._widget.PrePick_Button.setEnabled(False)
        self._widget.PickTest_Button.setEnabled(False)
        self._widget.PrePlace_Button.setEnabled(False)

    def activate_buttons(self):
        self._widget.PrePick_Button.setEnabled(True)
        self._widget.PickTest_Button.setEnabled(True)
        self._widget.PrePlace_Button.setEnabled(True)

    def callback_moving(self, data):
		global KUKA_AUT
		#print 'CB:moving_received:',data.data
		if data.data == True:
							KUKA_AUT=True
							self._widget.mode_label.setText("AUTOMATIC")
							#selt._widget.mode_label.setStyleSheet(\ncolor: rgb(255, 0, 0))
		else:
			KUKA_AUT=False
			self._widget.mode_label.setText("MANUAL")

    def callback_robot_pose(self, data):
		global pos_z_kuka
		print 'CB:robot_pose_received',data
		pos_z_kuka=data.z

    def callback_tool_weight(self, data):
		global weight_empty, weight_read
		print 'CB:tool_weight_received',data
		weight_read=data.data
		weight_no_tool=data.data-weight_empty
		self._widget.weight_lcdNumber.display(weight_no_tool)

    def callback_current(self, data):
        #print 'CB:current_received',data
        self._widget.tool_force_lcdNumber.display(data.data)

    def press_tool_homming(self):
        #Call tool homing method
        global weight_empty, weight_read
        try:
            gripper_move_service = rospy.ServiceProxy(srv_finger_set_pose,set_odometry)
            homing_service = rospy.ServiceProxy(srv_tool_homing, home)           
            ret = homing_service()
            weight_empty=weight_read
            gripper_move_service(0.05,0,0,-0.3)
            if ret == True:
                TOOL_HOMED=True
                self._widget.info_label.setText("Service tool homing call done")
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            self._widget.info_label.setText("Service tool homing call failed")

    def press_preplaced_button(self):
		global KUKA_AUT, pos_z_kuka
        #Call service to move robot up and then to the pre-pick pose, should be fast
		try:
			placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
			ret_rel=placed_rel_service(0, 0, Preplace_Pose_z-pos_z_kuka, 0, 0, 0)
			KUKA_AUT=True
			while KUKA_AUT: time.sleep(0.05)
			placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)
			ret = placed_abs_service(Preplace_Pose_x, Preplace_Pose_y, Preplace_Pose_z, Preplace_Pose_a,Preplace_Pose_b,Preplace_Pose_c)
			#ret=placed_rel_service(0, 0, -100, 0, 0, 0)
			if ret == True:
				CURRENT_STATE=3
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

    def press_prepick_button(self):
		global KUKA_AUT, pos_z_kuka
		#Call service to move robot up and then to pre place pose, should be slow
		try:
			picked_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
			ret_rel=picked_rel_service(0, 0,Prepick_Pose_z-pos_z_kuka , 0, 0, 0)
			KUKA_AUT=True
			while KUKA_AUT: time.sleep(0.05)
			picked_abs_service = rospy.ServiceProxy(srv_name_move_abs_fast, set_CartesianEuler_pose)
			ret = picked_abs_service(Prepick_Pose_x, Prepick_Pose_y, Prepick_Pose_z, Prepick_Pose_a,Prepick_Pose_b,Prepick_Pose_c)
			#ret=placed_rel_service(0, 0, -100, 0, 0, 0)
			if ret == True:
				CURRENT_STATE=STATE_MOVING_TO_PLACE
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			
    def press_tool_homming(self):
		global KUKA_AUT, pos_z_kuka
		#Call service to move robot up and then to pre place pose, should be slow
		try:
			homming_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
			ret_rel=homming_rel_service(0, 0,Homming_Pose_z-pos_z_kuka , 0, 0, 0)
			KUKA_AUT=True
			while KUKA_AUT: time.sleep(0.05)
			homming_abs_service = rospy.ServiceProxy(srv_name_move_abs_fast, set_CartesianEuler_pose)
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
			KUKA_AUT=True
			while KUKA_AUT: time.sleep(0.05)
			if ret_rel == True:
					CURRENT_STATE=STATE_DOING_PICK_TEST
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

    def press_tool_straighten(self):
		global KUKA_AUT
		print "Service called"
			    
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
    
    def calibre_selected(self, index):
        print 'Selected:', index
        #self.set_current_arm() NO DEBERIA ENTRAR AQUI SIN EL HOMING
        gripper_trasl_service = rospy.ServiceProxy(srv_finger_set_pose,set_odometry)
        if index == 0:
            print 'No gripper selected'
            self.desactivate_buttons()
            #desactivate all buttons
        else:
            #TODO: check if the gripper is empty. If there is some load not allow to move autonomously
            self.activate_buttons()            
        if index == 1:            
            print 'Set gripper to 100mm'
            #TODO: Set gripper to 100mm
            tras_from_homing=0.2-0.1;
            #ret=gripper_trasl_service(tras_from_homing,0,0,0)
            #if ret == False:
				#print 'Set gripper to 100mm: OUT OF RANGE'
            #if rospy.get_param('robot_description')
            try:
                rospy.delete_param('robot_description')
            except KeyError:
                print "value not set"
            self.load_robot_description(index)
        elif index == 2:
            print 'Set gripper to 140mm'
            #TODO: Set gripper to 140mm
            tras_from_homing=0.2-0.14;
            #ret=gripper_trasl_service(tras_from_homing,0,0,0)
            #if ret == False:
			#	print 'Set gripper to 140mm: OUT OF RANGE'
            try:
                rospy.delete_param('robot_description')
            except KeyError:
                print "value not set"
            self.load_robot_description(index+1)
        elif index == 3:
            print 'Set gripper to 160mm'
            #TODO: Set gripper to 160mm
            tras_from_homing=0.2-0.16;
            #ret=gripper_trasl_service(tras_from_homing,0,0,0)
            #if ret == False:
			#	print 'Set gripper to 140mm: OUT OF RANGE'
            try:
                rospy.delete_param('robot_description')
            except KeyError:
                print "value not set"
            self.load_robot_description(index+1)
        elif index == 4:
            print 'Set gripper to 270mm'
            #TODO: Set gripper to 270mm
            #ret=gripper_trasl_service(0.03,0,0,0)
            try:
                rospy.delete_param('robot_description')
            except KeyError:
                print "value not set"
            self.load_robot_description(index+1)

    def load_robot_description(self, gripper_model):
		command_string = "rosparam load ~/workspaces/kuka_catkin_ws/src/kuka_experimental/kuka_robot_bringup/robot/bin/kr120toolv%d.urdf /robot_description" % gripper_model
		os.system(command_string)
        
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
    
    def get_function_name(self):
        return inspect.currentframe().f_back.f_code.co_name
    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
