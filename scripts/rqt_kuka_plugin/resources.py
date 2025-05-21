# resources.py
# -*- coding: utf-8 -*-

from config import IMG_PATH

# Imágenes para cada tipo de obus (0: original, 1: resaltado, 2: seleccionado)
imgObus16izq = [IMG_PATH + "obus_izq_19x51_{}.png".format(i) for i in range(3)]
imgObus16der = [IMG_PATH + "obus_der_19x51_{}.png".format(i) for i in range(3)]
imgObus8izq  = [IMG_PATH + "obus_izq_26x71_{}.png".format(i) for i in range(3)]
imgObus8der  = [IMG_PATH + "obus_der_26x71_{}.png".format(i) for i in range(3)]
imgObus4     = [IMG_PATH + "obus_der_37x101_{}.png".format(i) for i in range(3)]
imgObus2     = [IMG_PATH + "obus_der_41x111_{}.png".format(i) for i in range(3)]

BACKGROUND_IMAGE = IMG_PATH + "fondo_huevera_0.png"

