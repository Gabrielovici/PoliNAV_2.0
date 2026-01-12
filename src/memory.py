import json
import os
import math
import src.config as config

# Calea catre fisierul json
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FISIER_MEMORIE = os.path.join(BASE_DIR, 'data', 'harta_robot.json')


def incarca_harta():
    """
    Incarca lista de obiecte. Daca fisierul e vechi (dictionar) sau gol, returneaza lista goala.
    """
    if os.path.exists(FISIER_MEMORIE):
        try:
            with open(FISIER_MEMORIE, 'r') as f:
                data = json.load(f)
                # Verificam daca e formatul nou (Lista) sau cel vechi (Dictionar)
                if isinstance(data, list):
                    print(f"[MEMORIE] Am incarcat {len(data)} obiecte.")
                    return data
                else:
                    print("[MEMORIE] Format vechi detectat. Resetez memoria.")
                    return []
        except Exception as e:
            print(f"[MEMORIE] Fisier corupt sau gol: {e}")
            return []
    return []


def salveaza_harta(lista_obiecte):
    try:
        # Ne asiguram ca folderul 'data' exista
        os.makedirs(os.path.dirname(FISIER_MEMORIE), exist_ok=True)
        with open(FISIER_MEMORIE, 'w') as f:
            json.dump(lista_obiecte, f, indent=4)
    except Exception as e:
        print(f"[MEMORIE EROARE] Nu pot salva harta: {e}")


def proceseaza_obiect(lista_obiecte, clasa, x_nou, y_nou):
    """
    Aceasta functie contine toata matematica de decizie.
    Primeste: Lista curenta, Tipul obiectului vazut, Coordonatele lui calculate.
    Returneaza: (Lista actualizata, Mesaj pentru consola)
    """
    # 1. Aflam raza de identitate din config (0.3m pt scaun, 2.0m default)
    # Folosim .get() ca sa nu crape daca clasa nu e in lista
    prag_distanta = config.MEMORY_THRESHOLDS.get(clasa, config.MEMORY_THRESHOLDS['default'])

    index_gasit = -1
    dist_minima = 9999.0

    # 2. Cautam in memorie daca avem deja un obiect de ACELASI TIP in apropiere
    for i, obj in enumerate(lista_obiecte):
        if obj['tip'] == clasa:
            # Distanta Euclidiana
            dist = math.sqrt((obj['x'] - x_nou) ** 2 + (obj['y'] - y_nou) ** 2)

            # Daca e in raza de actiune (ex: < 30cm)
            if dist < prag_distanta:
                # Retinem cel mai apropiat (in caz ca sunt mai multe suprapuse)
                if dist < dist_minima:
                    dist_minima = dist
                    index_gasit = i

    # 3. Luam Decizia
    if index_gasit != -1:
        # CAZ A: OBIECT DEJA CUNOSCUT (Actualizare)
        # Facem media pozitiilor pentru a rafina precizia (reduce tremuratul)
        vechi_x = lista_obiecte[index_gasit]['x']
        vechi_y = lista_obiecte[index_gasit]['y']

        lista_obiecte[index_gasit]['x'] = (vechi_x + x_nou) / 2.0
        lista_obiecte[index_gasit]['y'] = (vechi_y + y_nou) / 2.0

        # Returnam lista neschimbata ca lungime, doar coordonate updatate
        return lista_obiecte, f"Actualizat {clasa} #{lista_obiecte[index_gasit]['id']}"

    else:
        # CAZ B: OBIECT NOU
        # Ii dam un ID unic
        nou_id = len(lista_obiecte) + 1

        obiect_nou = {
            "id": nou_id,
            "tip": clasa,
            "x": round(x_nou, 3),  # Rotunjim la 3 zecimale
            "y": round(y_nou, 3)
        }

        lista_obiecte.append(obiect_nou)
        return lista_obiecte, f"NOU! {clasa} detectat -> ID:{nou_id} la ({x_nou:.1f}, {y_nou:.1f})"