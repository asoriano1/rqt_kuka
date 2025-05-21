# rqt_kuka_widget.py

from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPixmap
from python_qt_binding import loadUi
import os
from config import UI_PATH
from resources import BACKGROUND_IMAGE

class RqtKukaWidget(QWidget):
    def __init__(self, controller, state_manager, parent=None):
        super(RqtKukaWidget, self).__init__(parent)
        loadUi(UI_PATH, self)
        self.setObjectName('RqtKukaUi')

        self.controller = controller
        self.state_manager = state_manager

        self.init_ui()

    def init_ui(self):
        self.background_plate.setPixmap(QPixmap(BACKGROUND_IMAGE))

        if hasattr(self, 'Gripper_Homing_Button'):
            self.Gripper_Homing_Button.pressed.connect(self.press_gripper_homing)

        if hasattr(self, 'resetPositions_Button_pick'):
            self.resetPositions_Button_pick.pressed.connect(self.reset_pick_positions)

    def press_gripper_homing(self):
        result = self.controller.home_tool()
        if result:
            print("Tool homed correctamente")
        else:
            print("Fallo en homing del tool")

    def reset_pick_positions(self):
        self.state_manager.reset_all()
        print("Estados de pick/place reiniciados.")

