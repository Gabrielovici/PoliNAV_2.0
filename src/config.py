import numpy as np

# --- PARAMETRI NAVIGARE & CONTROL ---
DIST_MEMORARE = 1.2
VITEZA_BASE = 1.3  # Am pus viteza un pic mai mare, ca inainte
TOLERANTA_TINTA = 0.30

# --- PARAMETRI EVITARE OBSTACOLE ---
DIST_OCOLIRE = 0.35

# --- SETARI HARTA ---
MAP_RESOLUTION = 0.5
MAP_ORIGIN_X = -12.20
MAP_ORIGIN_Y = -12.30

MEMORY_THRESHOLDS = {
    'scaun': 1.2,
    'fotoliu': 0.4,
    'persoana': 1.0,
    'planta': 0.45,
    'tonomat':3.0,
    'masa':1.3,
    'default': 15.0
}

VISUAL_MIN_HEIGHT = {
    'scaun': 0.20,
    'fotoliu': 0.55,
    'planta': 0.70,
    'tonomat': 1.00,
    'masa':1.00,
    'default': 0.25
}


def genereaza_harta_L():
    width_m = 20.0
    height_m = 15.0

    cols = int(width_m / MAP_RESOLUTION)
    rows = int(height_m / MAP_RESOLUTION)

    grid = [[1 for _ in range(cols)] for _ in range(rows)]

    padding = 0.25 #mai ingorsam

    for r in range(rows):
        for c in range(cols):
            x = (c * MAP_RESOLUTION) + (MAP_RESOLUTION / 2)
            y = (r * MAP_RESOLUTION) + (MAP_RESOLUTION / 2)

            is_walkable = False

            # Zona Baza
            if (padding < x < 20.0 - padding) and (padding < y < 5.0 - padding):
                is_walkable = True

            # Turn Stanga
            if (padding < x < 5.0 - padding) and (4.5 <= y < 10.0 - padding):
                is_walkable = True

            # Turn Mijloc
            if (10.0 + padding < x < 15.0 - padding) and (4.5 <= y < 15.0 - padding):
                is_walkable = True

            if is_walkable:
                grid[r][c] = 0

    return grid