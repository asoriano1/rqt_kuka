# state_flags.py
# -*- coding: utf-8 -*-


# Inicializa los diccionarios con estados por huevera y obús
def create_status_dict(hueveras):
    return {
        h: {i + 1: False for i in range(n)}
        for h, n in hueveras.items()
    }

# Cantidad de obuses por huevera
HUEVERAS_PICK = {
    2: 4,    # posiblemente sobredimensionado
    4: 5,    # idem
    8: 14,
    16: 20   # parece tener más de los 16 reales
}

HUEVERAS_PLACE = {
    2: 2,
    4: 4,
    8: 8,
    16: 16
}

# Estados de pick y place
pick_status = create_status_dict(HUEVERAS_PICK)
place_status = create_status_dict(HUEVERAS_PLACE)

