import math
import src.config as config

# Variabile globale
last_error = 0
follow_side = 1  # 1 = Dreapta, -1 = Stanga


def get_robot_pose(sim, robot_handle):
    pos = sim.getObjectPosition(robot_handle, -1)
    _, _, yaw = sim.getObjectOrientation(robot_handle, -1)
    return pos[0], pos[1], yaw


def stop_robot(sim, left_motor, right_motor):
    sim.setJointTargetVelocity(left_motor, 0)
    sim.setJointTargetVelocity(right_motor, 0)


def avoid_obstacles(sim, left_motor, right_motor, sensors_data):
    """ Modul EXPLORE: Evitare simpla """
    d_front, d_left, d_right, d_diag_l, d_diag_r = sensors_data
    vl, vr = config.VITEZA_BASE, config.VITEZA_BASE

    if d_front < 0.6 or d_diag_l < 0.45 or d_diag_r < 0.45:
        if d_left > d_right:
            vl, vr = -0.5, 0.5
        else:
            vl, vr = 0.5, -0.5

    elif d_left < 0.4:
        vl, vr = 0.6, 0.4
    elif d_right < 0.4:
        vl, vr = 0.4, 0.6

    sim.setJointTargetVelocity(left_motor, vl)
    sim.setJointTargetVelocity(right_motor, vr)


def follow_wall(sim, left_motor, right_motor, sensors_data):
    """ WALL FOLLOWER INTELIGENT - VITEZA CONSTANTA """
    global last_error, follow_side

    d_front, d_left, d_right, d_diag_l, d_diag_r = sensors_data

    # PARAMETRI PID
    TARGET_DIST = 0.50

    # --- MODIFICARE: Folosim viteza maxima din config si la ocolire ---
    BASE_SPEED = config.VITEZA_BASE

    Kp = 2.0
    Kd = 1.5

    # DECIZIE INITIALA
    if d_front < 0.7:
        if d_left > d_right:
            follow_side = 1
        else:
            follow_side = -1

            # ROTIRE DE URGENTA
    if d_front < 0.6:
        if follow_side == 1:
            sim.setJointTargetVelocity(left_motor, -0.5)
            sim.setJointTargetVelocity(right_motor, 0.8)
        else:
            sim.setJointTargetVelocity(left_motor, 0.8)
            sim.setJointTargetVelocity(right_motor, -0.5)
        last_error = 0
        return

    # CALCUL PID
    if follow_side == 1:
        dist_to_wall = min(d_right, d_diag_r)
    else:
        dist_to_wall = min(d_left, d_diag_l)

    # Cautare perete (Viteza constanta)
    if dist_to_wall > 1.2:
        if follow_side == 1:
            sim.setJointTargetVelocity(left_motor, BASE_SPEED)
            sim.setJointTargetVelocity(right_motor, BASE_SPEED * 0.5)  # Doar o usoara curbura
        else:
            sim.setJointTargetVelocity(left_motor, BASE_SPEED * 0.5)
            sim.setJointTargetVelocity(right_motor, BASE_SPEED)
        last_error = 0
        return

    # PID Standard
    error = TARGET_DIST - dist_to_wall
    derivative = error - last_error
    turn_adjustment = (Kp * error) + (Kd * derivative)

    if follow_side == 1:
        vl = BASE_SPEED - turn_adjustment
        vr = BASE_SPEED + turn_adjustment
    else:
        vl = BASE_SPEED + turn_adjustment
        vr = BASE_SPEED - turn_adjustment

    # Saturatie (Le permitem sa mearga blana)
    vl = max(min(vl, 3.0), 0.1)
    vr = max(min(vr, 3.0), 0.1)

    sim.setJointTargetVelocity(left_motor, vl)
    sim.setJointTargetVelocity(right_motor, vr)

    last_error = error


def navigate_to_point(sim, left_motor, right_motor, robot_handle, target_x, target_y):
    """ Controler P - FARA INCETINIRE """
    rx, ry, ryaw = get_robot_pose(sim, robot_handle)
    dist = math.sqrt((target_x - rx) ** 2 + (target_y - ry) ** 2)
    angle_to_target = math.atan2(target_y - ry, target_x - rx)
    angle_diff = angle_to_target - ryaw

    while angle_diff > math.pi: angle_diff -= 2 * math.pi
    while angle_diff < -math.pi: angle_diff += 2 * math.pi

    if dist < config.TOLERANTA_TINTA:
        return True

    # Rotire pe loc daca unghiul e mare
    if abs(angle_diff) > 0.45:
        turn_speed = 1.0  # Rotire mai rapida
        if angle_diff > 0:
            vl, vr = -turn_speed, turn_speed
        else:
            vl, vr = turn_speed, -turn_speed
    else:
        # --- MODIFICARE: VITEZA CONSTANTA ---
        # Am scos linia: if dist < 0.5: speed *= 0.5
        speed = config.VITEZA_BASE

        k_turn = 2.0
        vl = speed - (angle_diff * k_turn)
        vr = speed + (angle_diff * k_turn)

    sim.setJointTargetVelocity(left_motor, vl)
    sim.setJointTargetVelocity(right_motor, vr)
    return False