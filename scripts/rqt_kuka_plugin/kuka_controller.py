# kuka_controller.py

import rospy
from robotnik_msgs.srv import set_CartesianEuler_pose, set_digital_output, home
from std_srvs.srv import SetBool
from geometry_msgs.msg import Pose, Point, Quaternion

class KukaController:
    def __init__(self):
        # Esperamos a que los servicios estén disponibles
        rospy.wait_for_service('/kuka_robot/setKukaAbs')
        rospy.wait_for_service('/kuka_tool/robotnik_base_hw/set_digital_output')
        rospy.wait_for_service('/kuka_tool/robotnik_base_hw/home')

        # Conectamos con los servicios
        self.move_abs_srv = rospy.ServiceProxy('/kuka_robot/setKukaAbs', set_CartesianEuler_pose)
        self.set_digital_output_srv = rospy.ServiceProxy('/kuka_tool/robotnik_base_hw/set_digital_output', set_digital_output)
        self.tool_home_srv = rospy.ServiceProxy('/kuka_tool/robotnik_base_hw/home', home)

    def move_to_pose(self, pose):
        try:
            req = set_CartesianEuler_pose._request_class()
            req.pose.x = pose.x
            req.pose.y = pose.y
            req.pose.z = pose.z
            req.pose.A = pose.a
            req.pose.B = pose.b
            req.pose.C = pose.c
            return self.move_abs_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"[KukaController] Error al mover el robot: {e}")
            return None

    def set_digital_output(self, pin, value):
        try:
            req = set_digital_output._request_class()
            req.pin = pin
            req.value = value
            return self.set_digital_output_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"[KukaController] Error al cambiar salida digital: {e}")
            return None

    def home_tool(self):
        try:
            return self.tool_home_srv()
        except rospy.ServiceException as e:
            rospy.logerr(f"[KukaController] Error al hacer homing del tool: {e}")
            return None

