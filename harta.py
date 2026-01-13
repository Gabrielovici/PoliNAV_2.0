import matplotlib.pyplot as plt
import numpy as np

# --- COPIEM SETARILE TALE DIN CONFIG ---
MAP_RESOLUTION = 0.5  # 1 celula = 0.5 metri


def genereaza_harta_L():
    """
    Genereaza matricea hartii (Grid Map) cu zone de siguranta (Padding).
    Latime: 20m, Inaltime: 15m.
    """
    width_m = 20.0
    height_m = 15.0

    cols = int(width_m / MAP_RESOLUTION)
    rows = int(height_m / MAP_RESOLUTION)

    # 1. Initializam totul ca ZID (1)
    grid = [[1 for _ in range(cols)] for _ in range(rows)]

    # 2. "Sapam" zonele libere, lasand o margine de siguranta (Padding)
    padding = 0.7

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


# --- VIZUALIZARE CU MATPLOTLIB ---
def arata_harta():
    grid = genereaza_harta_L()
    matrix = np.array(grid)

    print(f"Dimensiune Harta: {matrix.shape}")
    print("Legenda: ALB = Liber (0), NEGRU = Zid/Padding (1)")

    plt.figure(figsize=(10, 8))

    # Desenam matricea
    # origin='lower' pune randul 0 jos (ca in sistemul de coordonate X-Y)
    # cmap='gray' face 0=Negru, 1=Alb. Folosim 'gray_r' (reverse) sau logic manual.
    # Aici: 0 (Walkable) vrem sa fie ALB, 1 (Zid) vrem sa fie NEGRU.
    # Default imshow: Low values=Black, High=White.
    # Deci 0->Negru, 1->Alb. Noi vrem invers.

    plt.imshow(matrix, cmap='Greys', origin='lower', extent=[0, 20, 0, 15])

    plt.title("Harta de Navigare (Planner Grid)")
    plt.xlabel("X [metri]")
    plt.ylabel("Y [metri]")

    # Adaugam grid pentru a vedea celulele
    plt.grid(which='both', color='red', linestyle='-', linewidth=0.5, alpha=0.3)
    plt.minorticks_on()

    plt.show()


if __name__ == "__main__":
    arata_harta()