#!/usr/bin/python

import os
import inspect

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialog, QFileDialog, QMessageBox, QTableWidgetItem
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from robotnik_msgs.srv import home, set_odometry
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

#service names:
srv_name_move_abs_fast='/setKukaAbsFast'
srv_name_move_abs_slow='/setKukaAbs'
srv_name_move_rel_fast='/setKukaRelFast'
srv_name_move_rel_slow='/setKukaRel'
srv_tool_homing='/kuka_tool/robotnik_base_hw/home'
srv_finger_set_pose='/kuka_tool_finger_node/set_odometry' #robotnik_msgs.set.odometry

#topic names:
topic_cart_pose_kuka='/kuka_robot/cartesian_pos_kuka'
topic_kuka_moving='/kuka_robot/kuka_moving'
topic_tool_weight='/kuka_gauges/phidget_load/load_mean'
topic_current='current'

#Prepick Pose # tf.transformations.quaternion_from_euler(0, 0, th)
#Prepick_Pose=Pose(Point(100, 100, 100), Quaternion(0, 0, 0, 1))
Prepick_Pose_x=100
Prepick_Pose_y=100
Prepick_Pose_z=100
Prepick_Pose_theta=0

#Preplace Pose
#Preplace_Pose=Pose(Point(400, 400, 100), Quaternion(0, 0, 0, 1))
Preplace_Pose_x=400
Preplace_Pose_y=400
Preplace_Pose_z=100
Preplace_Pose_theta=0

#RollerBench Pose
RollerBench_Pose=Pose(Point(200, 200, 100), Quaternion(0, 0, 0, 1))

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
        #self._widget.PickTest_Button.pressed.connect(self.press_select_yaml)
        #self._widget.Picked_Button.pressed.connect(self.press_select_yaml)
        self._widget.Placed_Button.pressed.connect(self.press_preplace_button)
        self._widget.Gripper_Homing_Button.pressed.connect(self.press_tool_homming)
        self._widget.Apply_Button.pressed.connect(self.press_apply_button)

        #displays
        #self._widget.weight_lcdNumber.pressed.connect(self.press_load_yaml)
        #self._widget.tool_force_lcdNumber.pressed.connect(self.press_save_yaml)
        
        #subscriber to robot state
        rospy.Subscriber(topic_kuka_moving, Bool, self.callback_moving)

        #subscriber to robot pose
        rospy.Subscriber(topic_cart_pose_kuka, JointState, self.callback_robot_pose)     

        #subscriber to tool weight detected
        rospy.Subscriber(topic_tool_weight, JointState, self.callback_tool_weight)     

        #subscriber to tool current
        rospy.Subscriber(topic_current, JointState, self.callback_current) 
        
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
        self._widget.Picked_Button.setEnabled(False)
        self._widget.Placed_Button.setEnabled(False)

    def activate_buttons(self):
        self._widget.PrePick_Button.setEnabled(True)
        self._widget.PickTest_Button.setEnabled(True)
        self._widget.Picked_Button.setEnabled(True)
        self._widget.Placed_Button.setEnabled(True)

    def callback_moving(self, data):
        print 'CB:moving_received:',data.data
        if data.data == True:
            self._widget.mode_label.setText("AUTOMATIC")
            #selt._widget.mode_label.setStyleSheet(\ncolor: rgb(255, 0, 0))
        else:
            self._widget.mode_label.setText("MANUAL")

    def callback_robot_pose(self, data):
        print 'CB:robot_pose_received',data
        self._widget.robot_pose_x.setText(str(data.position[0]))
        self._widget.robot_pose_y.setText(str(data.position[1]))
        self._widget.robot_pose_z.setText(str(data.position[2]))
        self._widget.robot_pose_a.setText(str(data.position[3]))
        self._widget.robot_pose_b.setText(str(data.position[4]))
        self._widget.robot_pose_c.setText(str(data.position[5]))

    def callback_tool_weight(self, data):
        print 'CB:tool_weight_received',data
        self._widget.weight_lcdNumber.display(8)

    def callback_current(self, data):
        print 'CB:current_received',data
        self._widget.tool_force_lcdNumber.display(8)

    def press_tool_homming(self):
        #Call tool homing method
        try:
            homing_service = rospy.ServiceProxy(srv_tool_homing, home)            
            ret = homing_service()
            if ret == True:
                TOOL_HOMED=True
                self._widget.info_label.setText("Service tool homing call done")
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            self._widget.info_label.setText("Service tool homing call failed")

    def press_prepick_button(self):
        #Call service to move robot to prepick pose
        try:
            prepick_service = rospy.ServiceProxy(srv_name_move_abs_fast, set_odometry)            
            ret = prepick_service(Prepick_Pose_x, Prepick_Pose_y, Prepick_Pose_z, Prepick_Pose_theta)
            if ret == True:
                CURRENT_STATE=1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e 

    def press_preplace_button(self):
        #Call service to move robot to prepick pose
        try:
            prepick_service = rospy.ServiceProxy(srv_name_move_abs_fast, set_odometry)            
            ret = prepick_service(Prepick_Pose_x, Prepick_Pose_y, Prepick_Pose_z, Prepick_Pose_theta)
            if ret == True:
                CURRENT_STATE=3
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
    
    def calibre_selected(self, index):
        print 'Selected:', index
        #self.set_current_arm()
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
        elif index == 2:
            print 'Set gripper to 140mm'
            #TODO: Set gripper to 140mm
        elif index == 3:
            print 'Set gripper to 160mm'
            #TODO: Set gripper to 160mm
        elif index == 4:
            print 'Set gripper to 270mm'
            #TODO: Set gripper to 270mm

        
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
