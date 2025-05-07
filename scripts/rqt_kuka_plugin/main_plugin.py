# main_plugin.py

import os
from qt_gui.plugin import Plugin
from PyQt5.QtWidgets import QWidget
from python_qt_binding import loadUi

from config import UI_PATH
from resources import BACKGROUND_IMAGE
from poses import poses
from state_manager import StateManager

class RqtKukaPlugin(Plugin):
    def __init__(self, context):
        super(RqtKukaPlugin, self).__init__(context)
        self.setObjectName('RqtKukaPlugin')

        self.state_manager = StateManager()

        # Setup UI
        self._widget = QWidget()
        loadUi(UI_PATH, self._widget)
        self._widget.setObjectName('RqtKukaUi')

        # Background image example (if relevant)
        self._widget.background_plate.setPixmap(BACKGROUND_IMAGE)

        # Agregar widget al contexto
        if context.serial_number() > 1:
            self._widget.setWindowTitle(f"RqtKuka ({context.serial_number()})")
        context.add_widget(self._widget)

        # Aquí puedes conectar botones, inicializar ROS, etc.
        self.setup_buttons()

    def setup_buttons(self):
        # Ejemplo: conectar un botón con acción
        if hasattr(self._widget, 'resetPositions_Button_pick'):
            self._widget.resetPositions_Button_pick.pressed.connect(self.reset_pick_positions)

    def reset_pick_positions(self):
        self.state_manager.reset_all()
        print("Todos los estados de pick y place han sido reiniciados.")

