import os
import time
import cv2
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# --- CONFIGURARE ---
OUTPUT_DIR = "dataset/raw"  # Unde salvam pozele
VITEZA_DEPLASARE = 1.5  # Viteza roti
VITEZA_ROTIRE = 0.8  # Viteza rotire


def init_simulator():
    """Conectare la CoppeliaSim si preluare handles"""
    print("Conectare la simulator...")
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.setStepping(False)

    try:
        handles = {
            'left': sim.getObject('/PioneerP3DX/leftMotor'),
            'right': sim.getObject('/PioneerP3DX/rightMotor'),
            'camera': sim.getObject('/PioneerP3DX/visionSensor'),
            'robot': sim.getObject('/PioneerP3DX')
        }
        return sim, handles
    except Exception as e:
        print(f"[EROARE] Nu gasesc robotul: {e}")
        exit()


def proceseaza_imagine(img_raw, res):
    """Converteste imaginea din format Coppelia in format OpenCV (BGR)"""
    img_np = np.frombuffer(img_raw, dtype=np.uint8)
    img_np = img_np.reshape((res[1], res[0], 3))

    # In Coppelia imaginea e inversata pe verticala si e RGB
    img_np = cv2.flip(img_np, 0)
    img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
    return img_bgr


def main():
    # 1. Creare folder dataset daca nu exista
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"Creat folder: {OUTPUT_DIR}")

    sim, h = init_simulator()
    sim.startSimulation()

    print("\n--- CONTROL MANUAL & DATASET ---")
    print(" [W] - Fata")
    print(" [S] - Spate")
    print(" [A] - Rotire Stanga")
    print(" [D] - Rotire Dreapta")
    print(" [SPACE] - Stop")
    print(" [C] - CAPTURA FOTO (Salveaza in dataset/raw)")
    print(" [Q] - Iesire")
    print("----------------------------------")

    count = len(os.listdir(OUTPUT_DIR))
    print(f"Poze existente in folder: {count}")

    # Variabile pentru viteza curenta (pentru miscare lina)
    vl, vr = 0.0, 0.0

    while True:
        # A. Preluare Imagine
        img_raw, res = sim.getVisionSensorImg(h['camera'])

        if len(img_raw) > 0:
            frame = proceseaza_imagine(img_raw, res)

            # Afisam instructiuni pe ecran
            info = f"Imagini: {count} | Comanda: {vl:.1f}/{vr:.1f}"
            cv2.putText(frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2)

            cv2.imshow("Dataset Collector", frame)

        # B. Control Tastatura
        key = cv2.waitKey(1) & 0xFF

        # STOP (Space)
        if key == ord(' '):
            vl, vr = 0.0, 0.0

        # FATA (W)
        elif key == ord('w'):
            vl, vr = VITEZA_DEPLASARE, VITEZA_DEPLASARE

        # SPATE (S)
        elif key == ord('s'):
            vl, vr = -VITEZA_DEPLASARE, -VITEZA_DEPLASARE

        # STANGA (A)
        elif key == ord('a'):
            vl, vr = -VITEZA_ROTIRE, VITEZA_ROTIRE

        # DREAPTA (D)
        elif key == ord('d'):
            vl, vr = VITEZA_ROTIRE, -VITEZA_ROTIRE

        # CAPTURA (C)
        elif key == ord('c'):
            # Generam un nume unic bazat pe timestamp
            timestamp = int(time.time() * 1000)
            filename = f"{OUTPUT_DIR}/img_{timestamp}.jpg"

            # Salvam imaginea CURATA (fara textul verde de pe ecran)
            # frame are text pe el, asa ca procesam din nou raw-ul sau facem o copie inainte
            # Mai eficient: salvam frame-ul curent, dar textul e util pentru tine.
            # Voi regenera imaginea curata pentru salvare.
            img_clean = proceseaza_imagine(img_raw, res)
            cv2.imwrite(filename, img_clean)

            count += 1
            print(f"[SALVAT] {filename}")
            # Flash visual (ecran alb scurt)
            cv2.imshow("Dataset Collector", np.ones_like(frame) * 255)
            cv2.waitKey(50)

        # IESIRE (Q)
        elif key == ord('q'):
            break

        # Trimite comanda la motoare
        sim.setJointTargetVelocity(h['left'], vl)
        sim.setJointTargetVelocity(h['right'], vr)

    # Finalizare
    sim.setJointTargetVelocity(h['left'], 0)
    sim.setJointTargetVelocity(h['right'], 0)
    sim.stopSimulation()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()