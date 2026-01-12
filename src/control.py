import math
import src.config as config


def get_robot_pose(sim, robot_handle):
    pos = sim.getObjectPosition(robot_handle, -1)
    _, _, yaw = sim.getObjectOrientation(robot_handle, -1)
    return pos[0], pos[1], yaw


def stop_robot(sim, left_motor, right_motor):
    sim.setJointTargetVelocity(left_motor, 0)
    sim.setJointTargetVelocity(right_motor, 0)


def avoid_obstacles(sim, left_motor, right_motor, sensors_data):
    """
    VARIANTA SIMPLA (Pentru EXPLORE):
    Praguri marite pentru siguranta.
    """
    d_front, d_left, d_right, d_diag_l, d_diag_r = sensors_data

    vl, vr = config.VITEZA_BASE, config.VITEZA_BASE

    # 1. Pericol Frontal (Marit la 0.7m)
    if d_front < 0.7 or d_diag_l < 0.5 or d_diag_r < 0.5:
        if d_left > d_right:
            vl, vr = -0.5, 0.5
        else:
            vl, vr = 0.5, -0.5

            # 2. Pericol Lateral (Marit la 0.5m)
    # Daca trece la mai putin de jumatate de metru, se corecteaza.
    elif d_left < 0.5:
        vl, vr = 0.5, 0.2  # Usor Dreapta
    elif d_right < 0.5:
        vl, vr = 0.2, 0.5  # Usor Stanga

    sim.setJointTargetVelocity(left_motor, vl)
    sim.setJointTargetVelocity(right_motor, vr)


def follow_wall(sim, left_motor, right_motor, sensors_data):
    """
    WALL FOLLOWER (Navigare)
    Set Point marit la 0.75 metri.
    """
    d_front, d_left, d_right, d_diag_l, d_diag_r = sensors_data

    # --- MODIFICARE IMPORTANTA ---
    TARGET_DIST = 0.75  # Tinem obstacolul la 75cm distanta (era 50cm)
    BASE_SPEED = 0.8

    # 1. Rotire de urgenta daca e ceva in fata (Marit la 0.6m)
    if d_front < 0.6 or d_diag_l < 0.5:
        sim.setJointTargetVelocity(left_motor, -0.8)
        sim.setJointTargetVelocity(right_motor, 0.8)
        return

    # 2. Urmarire perete dreapta
    dist_to_wall = min(d_right, d_diag_r)

    # Daca a pierdut peretele (e prea departe), il cauta lin
    if dist_to_wall > 1.5:
        sim.setJointTargetVelocity(left_motor, BASE_SPEED)
        sim.setJointTargetVelocity(right_motor, BASE_SPEED * 0.5)
        return

    # PID Control
    error = TARGET_DIST - dist_to_wall
    kp = 2.5  # Putin mai relaxat ca sa nu oscileze la distanta mare

    correction = error * kp

    vl = BASE_SPEED - correction
    vr = BASE_SPEED + correction

    # Saturatie viteze
    vl = max(min(vl, 2.0), 0.1)
    vr = max(min(vr, 2.0), 0.1)

    sim.setJointTargetVelocity(left_motor, vl)
    sim.setJointTargetVelocity(right_motor, vr)


def navigate_to_point(sim, left_motor, right_motor, robot_handle, target_x, target_y):
    # Controler P standard (neschimbat)
    rx, ry, ryaw = get_robot_pose(sim, robot_handle)
    dist = math.sqrt((target_x - rx) ** 2 + (target_y - ry) ** 2)
    angle_to_target = math.atan2(target_y - ry, target_x - rx)
    angle_diff = angle_to_target - ryaw

    while angle_diff > math.pi: angle_diff -= 2 * math.pi
    while angle_diff < -math.pi: angle_diff += 2 * math.pi

    if dist < config.TOLERANTA_TINTA:
        return True

    k_turn = 2.5
    if abs(angle_diff) > 0.4:
        vl = -angle_diff * 1.5
        vr = angle_diff * 1.5
    else:
        speed = config.VITEZA_BASE
        vl = speed - (angle_diff * k_turn)
        vr = speed + (angle_diff * k_turn)

    sim.setJointTargetVelocity(left_motor, vl)
    sim.setJointTargetVelocity(right_motor, vr)
    return False