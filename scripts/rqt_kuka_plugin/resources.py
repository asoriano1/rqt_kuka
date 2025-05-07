# resources.py
from config import IMG_PATH

# Imagenes para cada tipo de obus (0: original, 1: resaltado, 2: seleccionado)
imgObus16izq = [IMG_PATH + f"obus_izq_19x51_{i}.png" for i in range(3)]
imgObus16der = [IMG_PATH + f"obus_der_19x51_{i}.png" for i in range(3)]
imgObus8izq  = [IMG_PATH + f"obus_izq_26x71_{i}.png" for i in range(3)]
imgObus8der  = [IMG_PATH + f"obus_der_26x71_{i}.png" for i in range(3)]
imgObus4     = [IMG_PATH + f"obus_der_37x101_{i}.png" for i in range(3)]
imgObus2     = [IMG_PATH + f"obus_der_41x111_{i}.png" for i in range(3)]

BACKGROUND_IMAGE = IMG_PATH + "fondo_huevera_0.png"

