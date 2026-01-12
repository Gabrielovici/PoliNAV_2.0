import numpy as np

# --- PARAMETRI NAVIGARE & CONTROL ---
DIST_MEMORARE = 2.0         # Memoreaza doar daca e sub 2m
VITEZA_BASE = 1.2
TOLERANTA_TINTA = 0.3

# --- PARAMETRI EVITARE OBSTACOLE ---
DIST_OCOLIRE = 0.35

# --- SETARI HARTA ---
MAP_RESOLUTION = 0.5
MAP_ORIGIN_X = -12.20
MAP_ORIGIN_Y = -12.30

# --- RAZE DE IDENTITATE (Cat de departate pot fi 2 detectii ca sa fie acelasi obiect) ---
MEMORY_THRESHOLDS = {
    'scaun': 0.5,
    'fotoliu': 1.0,    # Fotoliile sunt mari, raza mare
    'persoana': 1.5,
    'planta': 0.8,
    'default': 2.0
}

# --- FILTRU VISUAL (NOU!) ---
# Cat % din inaltimea ecranului trebuie sa ocupe obiectul
# ca sa consideram ca e "langa noi" (sub 1.5m), chiar daca senzorul e orb.
VISUAL_MIN_HEIGHT = {
    'scaun': 0.20,     # Scaunul e mediu -> 20%
    'fotoliu': 0.35,   # Fotoliul e mare -> trebuie sa fie MARE pe ecran (35%)
    'planta': 0.45,    # Planta e inalta -> 25%
    'tonomat':0.60,
    'default': 0.10    # Cutiile (care nu au nume in lista) sunt mici -> 10%
}

def genereaza_harta_L():
    # ... (restul functiei ramane identic, nu s-a schimbat)
    width_m = 20.0
    height_m = 15.0
    cols = int(width_m / MAP_RESOLUTION)
    rows = int(height_m / MAP_RESOLUTION)
    grid = [[1 for _ in range(cols)] for _ in range(rows)]
    padding = 0.7

    for r in range(rows):
        for c in range(cols):
            x = (c * MAP_RESOLUTION) + (MAP_RESOLUTION / 2)
            y = (r * MAP_RESOLUTION) + (MAP_RESOLUTION / 2)
            is_walkable = False
            if (padding < x < 20.0 - padding) and (padding < y < 5.0 - padding): is_walkable = True
            if (padding < x < 5.0 - padding) and (4.5 <= y < 10.0 - padding): is_walkable = True
            if (10.0 + padding < x < 15.0 - padding) and (4.5 <= y < 15.0 - padding): is_walkable = True
            if is_walkable: grid[r][c] = 0
    return grid