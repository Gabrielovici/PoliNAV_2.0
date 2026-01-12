import time
import math
import cv2
import sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import src.config as config
import src.vision as vision
import src.control as control
import src.memory as memory
import src.planner as planner


def main():
    print("=== START PROIECT POLINAV (FINAL: FIX NAVIGARE + FILTRU) ===")

    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.setStepping(False)

    try:
        motor_l = sim.getObject('/PioneerP3DX/leftMotor')
        motor_r = sim.getObject('/PioneerP3DX/rightMotor')
        camera = sim.getObject('/PioneerP3DX/visionSensor')
        robot = sim.getObject('/PioneerP3DX')

        s_front = sim.getObject('/PioneerP3DX/sensor_front')
        s_left = sim.getObject('/PioneerP3DX/sensor_left')
        s_right = sim.getObject('/PioneerP3DX/sensor_right')
        s_diag_left = sim.getObject('/PioneerP3DX/sensor_diag_left')
        s_diag_right = sim.getObject('/PioneerP3DX/sensor_diag_right')
    except Exception as e:
        print(f"[EROARE] {e}")
        return

    sim.startSimulation()

    memorie_lista = memory.incarca_harta()
    grid_map = config.genereaza_harta_L()

    my_planner = planner.AStarPlanner(grid_map, config.MAP_RESOLUTION, config.MAP_ORIGIN_X, config.MAP_ORIGIN_Y)

    stare = "EXPLORE"
    current_path = []
    target_name = ""
    is_wall_following = False

    print("Comenzi: [n] Navigare | [q] Iesire")

    while True:
        # --- A. CITIRE SENZORI ---
        def read_prox(h):
            res, dist, _, _, _ = sim.readProximitySensor(h)
            return dist if res > 0 else 10.0

        d_front = read_prox(s_front)
        d_l, d_r = read_prox(s_left), read_prox(s_right)
        d_dl, d_dr = read_prox(s_diag_left), read_prox(s_diag_right)
        sensors = (d_front, d_l, d_r, d_dl, d_dr)

        rx, ry, r_theta = control.get_robot_pose(sim, robot)

        # --- B. PROCESARE VIZUALA (DOAR AFISARE, FARA MEMORARE INCA) ---
        img_raw, res = sim.getVisionSensorImg(camera)
        if len(img_raw) > 0:
            img_display = vision.process_camera(img_raw, res)

            # Desenam doar info general
            txt_info = f"Mod: {stare} | Obiecte: {len(memorie_lista)}"
            cv2.putText(img_display, txt_info, (10, res[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        else:
            img_display = None

        # --- C. CONTROL & LOGICA PRINCIPALA ---
        key = cv2.waitKey(1) & 0xFF

        # =========================================================
        # CAZ 1: EXPLORARE (Aici memoram!)
        # =========================================================
        if stare == "EXPLORE":
            control.avoid_obstacles(sim, motor_l, motor_r, sensors)

            # --- LOGICA DE MEMORARE ESTE ACUM AICI ---
            if img_display is not None:
                obiecte_detectate = vision.detect_objects(img_display)

                for obj in obiecte_detectate:
                    nume_clasa = obj['name']
                    conf = obj['conf']

                    # 1. Marime pe ecran
                    h_box = obj['box'][3]
                    raport_vizual = h_box / res[1]

                    # 2. Senzori (Cine vede?)
                    dist_grup_fata = min(d_front, d_dl, d_dr)

                    # Decidem directia
                    dist_finala = dist_grup_fata
                    unghi_offset = 0.0

                    # Verificam lateralele
                    if d_l < dist_grup_fata and d_l < 1.0:
                        dist_finala = d_l
                        unghi_offset = 1.57  # +90 grade
                    elif d_r < dist_grup_fata and d_r < 1.0:
                        dist_finala = d_r
                        unghi_offset = -1.57  # -90 grade

                    # 3. FILTRU DE CONSISTENTA (CRITIC!)
                    # Luam pragul din CONFIG (ex: fotoliu=0.35, cutie=0.10)
                    prag_vizual = config.VISUAL_MIN_HEIGHT.get(nume_clasa, config.VISUAL_MIN_HEIGHT['default'])

                    # ESTE VALID DACA:
                    # A. Senzorii il vad aproape (< 2m) SI obiectul e destul de mare pt clasa lui
                    valid_senzorial = (dist_finala < config.DIST_MEMORARE) and (raport_vizual > prag_vizual * 0.5)

                    # B. Visual Override: Senzorii sunt orbi, dar obiectul e URIAS (mai mare decat pragul definit)
                    valid_vizual = (raport_vizual > prag_vizual)

                    if conf > 0.60 and (valid_senzorial or valid_vizual):

                        # Estimare distanta daca senzorul e orb
                        if valid_vizual and not valid_senzorial:
                            dist_reala = 0.8 + 0.2
                            unghi_offset = 0.0  # Presupunem fata
                        else:
                            dist_reala = dist_finala + 0.2

                        # Calcul final
                        unghi_total = r_theta + unghi_offset
                        ox = rx + dist_reala * math.cos(unghi_total)
                        oy = ry + dist_reala * math.sin(unghi_total)

                        memorie_lista, msg = memory.proceseaza_obiect(memorie_lista, nume_clasa, ox, oy)
                        if "NOU!" in msg or "Actualizat" in msg:
                            memory.salveaza_harta(memorie_lista)
                            if "NOU!" in msg:
                                print(f"--> {msg} (H={raport_vizual:.2f})")
                                cv2.circle(img_display, (30, 30), 20, (0, 0, 255), -1)

            # Comanda Navigare
            if key == ord('n'):
                control.stop_robot(sim, motor_l, motor_r)
                tipuri = sorted(list(set([o['tip'] for o in memorie_lista])))
                print(f"\nObiecte in memorie: {tipuri}")

                if not tipuri:
                    print("Nu stiu nimic inca.")
                else:
                    cmd = input("Mergi la: ").strip().lower()
                    tinte = [o for o in memorie_lista if o['tip'] == cmd]
                    if tinte:
                        # Mergem la cel mai apropiat
                        tinta = min(tinte, key=lambda o: math.sqrt((o['x'] - rx) ** 2 + (o['y'] - ry) ** 2))
                        path = my_planner.plan(rx, ry, tinta['x'], tinta['y'])
                        if path:
                            current_path = path
                            target_name = f"{cmd} #{tinta['id']}"
                            stare = "NAVIGATE"
                            is_wall_following = False
                            print(f"Traseu calculat catre {target_name}!")
                    else:
                        print("Nu am gasit obiectul.")

        # =========================================================
        # CAZ 2: NAVIGARE (FARA MEMORARE!)
        # =========================================================
        elif stare == "NAVIGATE":
            if len(current_path) > 0:
                nx, ny = current_path[0]

                # Logica Wall Follow temporar
                if not is_wall_following:
                    # Daca e obstacol, trecem pe ocolire
                    if d_front < 0.6 or sensors[3] < 0.5 or sensors[4] < 0.5:
                        print("[NAV] Obstacol! Ocolesc...")
                        is_wall_following = True
                        sim.setJointTargetVelocity(motor_l, -1.0)
                        sim.setJointTargetVelocity(motor_r, 1.0)
                        time.sleep(0.4)
                    else:
                        # Mergem pe puncte
                        ajuns = control.navigate_to_point(sim, motor_l, motor_r, robot, nx, ny)
                        if ajuns:
                            current_path.pop(0)
                else:
                    # Suntem in ocolire
                    control.follow_wall(sim, motor_l, motor_r, sensors)

                    # Verificam daca am revenit la traseu
                    # ... logica unghiulara ...
                    angle_target = math.atan2(ny - ry, nx - rx)
                    angle_rob = r_theta
                    diff = angle_target - angle_rob
                    while diff > math.pi: diff -= 2 * math.pi
                    while diff < -math.pi: diff += 2 * math.pi

                    if abs(diff) < 0.5 and d_front > 1.0:
                        print("[NAV] Drum liber. Revin la A*.")
                        is_wall_following = False
            else:
                print(f"🎉 AJUNS LA DESTINATIE: {target_name}")
                control.stop_robot(sim, motor_l, motor_r)
                stare = "EXPLORE"

        # Afisare imagine (comuna)
        if img_display is not None:
            cv2.imshow("Robot Vision", img_display)

        if key == ord('q'):
            break

    sim.stopSimulation()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()