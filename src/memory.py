import json
import math
import os
import src.config as config


MEMORY_FILE = os.path.join("data", "harta_robot.json")


def incarca_harta():
    if not os.path.exists(MEMORY_FILE):
        return []
    try:
        with open(MEMORY_FILE, "r") as f:
            data = json.load(f)
            if isinstance(data, list):
                return data
            return []
    except:
        return []


def salveaza_harta(lista_obiecte):
    try:
        with open(MEMORY_FILE, "w") as f:
            json.dump(lista_obiecte, f, indent=4)
    except Exception as e:
        print(f"[MEMORIE EROARE] Nu pot salva: {e}")


def get_next_id(lista_obiecte):
    if not lista_obiecte:
        return 1
    max_id = max([o['id'] for o in lista_obiecte])
    return max_id + 1


def proceseaza_obiect(lista_obiecte, tip, x, y):
    """
    Logica inteligenta de evitare a dublurilor.
    """

    # 1. Luam raza specifica din config (ex: Tonomat = 2.0m)
    raza_identitate = config.MEMORY_THRESHOLDS.get(tip, config.MEMORY_THRESHOLDS['default'])

    cel_mai_apropiat = None
    dist_minima = 999.0

    # 2. Cautam daca avem DEJA acest obiect in harta_robot.json
    for obj in lista_obiecte:
        if obj['tip'] == tip:
            d = math.sqrt((obj['x'] - x) ** 2 + (obj['y'] - y) ** 2)
            if d < dist_minima:
                dist_minima = d
                cel_mai_apropiat = obj

    # 3. VERIFICARE DUBLURI
    if cel_mai_apropiat is not None and dist_minima < raza_identitate:
        # --- CAZUL: E ACELASI OBIECT! ---
        # Nu cream unul nou. Doar ii actualizam pozitia (facem media).
        # Asta ajuta daca prima data l-a vazut putin gresit.

        cel_mai_apropiat['x'] = (cel_mai_apropiat['x'] + x) / 2.0
        cel_mai_apropiat['y'] = (cel_mai_apropiat['y'] + y) / 2.0

        msg = f"Actualizat {tip} #{cel_mai_apropiat['id']} (Dist: {dist_minima:.2f}m)"
        return lista_obiecte, msg
    else:
        # --- CAZUL: OBIECT NOU-NOUT ---
        new_id = get_next_id(lista_obiecte)
        nou = {
            "id": new_id,
            "tip": tip,
            "x": round(x, 2),
            "y": round(y, 2),
            # Poti adauga si alte date daca vrei
        }
        lista_obiecte.append(nou)
        msg = f"NOU! {tip} detectat -> ID:{new_id} la ({x:.1f}, {y:.1f})"
        return lista_obiecte, msg