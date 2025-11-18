#!/usr/bin/env python3
"""
IK a velocità con Pinocchio del manipolatore aereo (drone + braccio) usando un modello a base flottante.
- Carica l'URDF completo del sistema (t960a.urdf) con JointModelFreeFlyer
- Calcola qdot = J_arm^+ * v_EE_des (pseudoinversa smorzata), ignorando per ora i 6 DoF della base
- Genera e insegue un segmento orizzontale nel frame WORLD a velocità costante (solo feedforward)
- Visualizzazione con MeshCat (opzionale)

Nota: Per semplicità, carico il file URDF già generato (t960a.urdf). Se vuoi usare direttamente lo xacro,
è possibile espanderlo a runtime (xacro -> URDF) e passarlo a Pinocchio, ma richiede xacro installato.

Requisiti Python: pinocchio, meshcat, numpy
"""
import os
import time
import numpy as np

import pinocchio as pin
import pinocchio.visualize

# Parametri controllo/traiettoria
RATE = 200.0          # Hz di integrazione/visualizzazione
DAMPING = 1e-4        # smorzamento per la pseudoinversa
SEG_DURATION = 15.0    # durata del moto [s]
RADIUS = 0.10         # raggio della traiettoria circolare [m]
CIRC_T = SEG_DURATION # durata di un giro completo [s]

# Nomi frame e giunti
EE_FRAME = "mobile_wx250s/ee_gripper_link"
ARM_JOINTS = [
    "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"
]


def _skew(v: np.ndarray) -> np.ndarray:
    """Restituisce la matrice skew-symmetric di un vettore 3D."""
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    return np.array([[0.0, -z, y],
                     [z, 0.0, -x],
                     [-y, x, 0.0]])


def find_urdf_and_pkg_dir():
    """Restituisce (urdf_filename, pkg_dir) per t960a.urdf.
    Prova nel path install della workspace e poi nella workspace sorgente.
    """
    ws_install = "/home/mattia/interbotix_ws/install"
    cand1 = os.path.join(ws_install, "clik1_node_pkg", "share", "clik1_node_pkg", "model", "t960a.urdf")
    pkg1 = os.path.join(ws_install, "clik1_node_pkg", "model")

    here = os.path.dirname(os.path.abspath(__file__))
    cand2 = os.path.normpath(os.path.join(here, "..", "model", "t960a.urdf"))
    pkg2 = os.path.normpath(os.path.join(here, "..", "model"))

    if os.path.exists(cand1):
        return cand1, pkg1
    if os.path.exists(cand2):
        return cand2, pkg2

    raise FileNotFoundError(f"URDF t960a.urdf non trovato. Cercati: {cand1} e {cand2}")


def damped_pseudoinverse(J: np.ndarray, lamb: float = DAMPING) -> np.ndarray:
    """Pseudoinversa smorzata (Tikhonov) per J in R^{m x n}.
    Restituisce J^+ = J^T (J J^T + λ^2 I)^{-1} se m <= n, altrimenti (J^T J + λ^2 I)^{-1} J^T.
    """
    m, n = J.shape
    if m <= n:
        JJt = J @ J.T
        return J.T @ np.linalg.inv(JJt + (lamb ** 2) * np.eye(m))
    else:
        JtJ = J.T @ J
        return np.linalg.inv(JtJ + (lamb ** 2) * np.eye(n)) @ J.T


def get_arm_velocity_indices(model: pin.Model, arm_joint_names: list[str]) -> list[int]:
    """Restituisce la lista degli indici di velocità (idx_v) nei quali scrivere/leggere le colonne
    del Jacobiano per i giunti del braccio (escludendo i 6 DoF del free-flyer).
    """
    idxs = []
    for jname in arm_joint_names:
        if not model.existJointName(jname):
            raise ValueError(f"Joint '{jname}' non trovato nel modello.")
        jid = model.getJointId(jname)
        idx_v = model.joints[jid].idx_v
        if idx_v < 0 or idx_v >= model.nv:
            raise RuntimeError(f"idx_v fuori range per joint '{jname}' (idx_v={idx_v}, nv={model.nv})")
        idxs.append(int(idx_v))
    return idxs


def main():
    urdf_filename, pkg_dir = find_urdf_and_pkg_dir()

    # Costruisci il modello con base flottante (FreeFlyer)
    # Modello + collision/visual per MeshCat
    model, cmodel, vmodel = pin.buildModelsFromUrdf(
        urdf_filename,
        package_dirs=[pkg_dir],
        root_joint=pin.JointModelFreeFlyer()
    )
    data = model.createData()

    # Visualizzatore Meshcat (opzionale)
    viz = pin.visualize.MeshcatVisualizer(model, cmodel, vmodel)
    viz.initViewer(open=True)
    viz.loadViewerModel()

    # Configurazione iniziale: neutra (base all'origine, quaternione unitario, braccio neutro)
    q = pin.neutral(model)

    # Verifica frame EE disponibile
    assert model.existFrame(EE_FRAME), f"Frame EE non trovato: {EE_FRAME}"
    ee_frame_id = model.getFrameId(EE_FRAME)

    # Ricava indici di velocità per i giunti del braccio
    arm_idx_v = get_arm_velocity_indices(model, ARM_JOINTS)

    # FK iniziale per ottenere p0
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    T_we = data.oMf[ee_frame_id]
    p0 = np.array(T_we.translation).reshape(3)
    # Log traiettoria (WORLD)
    p_log = [p0.copy()]
    t_log = [0.0]

    # Visualizza stato iniziale
    viz.display(q)

    # Stampa Jacobiano iniziale del solo braccio Jm0 (espresso in LOCAL_WORLD_ALIGNED)
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)
    J0_lwa = pin.computeFrameJacobian(model, data, q, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    # Seleziona solo le colonne dei giunti del braccio tramite gli indici di velocità
    Jm0 = J0_lwa[:, arm_idx_v]
    np.set_printoptions(precision=5, suppress=True)
    print("Jacobiano iniziale braccio Jm0 (LOCAL_WORLD_ALIGNED), shape=", Jm0.shape)
    print(Jm0)

    #     # Calcolo e stampa dell'Adjoint dalla base all'end-effector e del prodotto J0_base * Ad_be
    #     # Pose WORLD della base (free-flyer) e dell'EE
    #     try:
    #         base_jid = 1  # per JointModelFreeFlyer, il primo joint dopo 'universe' è la base
    #         T_wb = data.oMi[base_jid]
    #     except Exception:
    #         # fallback: usa il jointId del primo joint
    #         T_wb = data.oMi[1]
    #     # T_be = T_wb^{-1} * T_we
    #     T_we = data.oMf[ee_frame_id]
    #     T_be = T_wb.inverse() * T_we
    #     # Adjoint via Pinocchio API (toActionMatrix)
    #     Ad_be_pin = np.array(T_be.toActionMatrix())
    #     print("Adjoint(base->ee) via toActionMatrix, shape=", Ad_be_pin.shape)
    #     print(Ad_be_pin)
    #     # Adjoint inverso via Pinocchio API
    #     Ad_eb_pin = np.array((T_be.inverse()).toActionMatrix())
    #     print("Adjoint(ee->base) via toActionMatrix, shape=", Ad_eb_pin.shape)
    #     print(Ad_eb_pin)

    # Integrazione
    dt = 1.0 / RATE
    # Pausa per permettere la visualizzazione iniziale nel viewer prima di iniziare il controllo
    time.sleep(2.0)
    t0 = time.time()
    last_draw = t0

    # Traiettoria: circolare su piano WORLD x-z, r=RADIUS, orientazione EE costante (ω_des=0)
    # Centro scelto per passare da p0 al tempo t=0 sul bordo della circonferenza lungo +X
    center = p0.copy()
    center[0] = p0[0] - RADIUS
    center[2] = p0[2]
    center[1] = p0[1]
    omega_y = 2.0 * np.pi / CIRC_T  # velocità angolare lungo asse Y_world [rad/s]
    e_y = np.array([0.0, 1.0, 0.0])

    print(
        "IK UAM (free-flyer): traiettoria circolare WORLD (piano x-z), R=%.2f m, T=%.1f s, rate=%.1f Hz"
        % (RADIUS, CIRC_T, RATE)
    )

    try:
        while True:
            t = time.time() - t0

            # Kinematics
            pin.forwardKinematics(model, data, q)
            pin.computeJointJacobians(model, data, q)
            pin.updateFramePlacements(model, data)

            # Jacobiano del frame EE espresso in LOCAL_WORLD_ALIGNED (spatial: [lin; ang])
            J_spatial = pin.computeFrameJacobian(
                model, data, q, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )

            # Posizione attuale dell'EE
            T_we = data.oMf[ee_frame_id]
            p = np.array(T_we.translation).reshape(3)
            # Log posizione
            p_log.append(p.copy())
            t_log.append(t)

            # Velocità desiderata del frame EE (LOCAL_WORLD_ALIGNED): ω_des=0, v_des = ω_y × (p - center)
            if t <= SEG_DURATION:
                r = p - center
                v_lin_world = omega_y * np.cross(e_y, r)
                omega_des = np.zeros(3)
                # spatial order [ang; lin]
                v_ee_des_spatial = np.hstack([v_lin_world, omega_des])
            else:
                v_ee_des_spatial = np.zeros(6)

            # Usa SOLO il Jacobiano del manipolatore (colonne dopo la sesta) per muovere il braccio
            Jm = J_spatial[:, 6:]  # 6 x n_arm_all
            Jm_pinv = damped_pseudoinverse(Jm, DAMPING)  # (n_arm_all x 6)
            qdot_arm = Jm_pinv @ v_ee_des_spatial

            # Clipping per stabilità sui giunti del braccio [rad/s]
            qdot_arm = np.clip(qdot_arm, -2.0, 2.0)

            # Costruisci vettore velocità completo (nv): base ferma, solo braccio attivo
            v_full = np.zeros(model.nv)
            v_full[6:] = qdot_arm

            # Integrazione
            q = pin.integrate(model, q, v_full * dt)

            # Visualizzazione a rate fisso
            now = time.time()
            if now - last_draw >= dt:
                viz.display(q)
                last_draw = now

            # Diagnostica a 10Hz
            if int(t * 10) != int((t - dt) * 10):
                # Stampa: solo parte lineare di v_ee_des e angoli di giunto del braccio in gradi
                v_ee_des_lin = v_ee_des_spatial[:3]
                # Estrai angoli giunti braccio (nq=1 per ciascuno) e converti in gradi
                arm_q_rad = []
                for jname in ARM_JOINTS:
                    jid = model.getJointId(jname)
                    idx_q = model.joints[jid].idx_q
                    arm_q_rad.append(float(q[idx_q]))
                arm_q_deg = np.rad2deg(np.array(arm_q_rad))
                print(f"t={t:5.2f}s | v_ee_des_lin={v_ee_des_lin} | q_arm_deg={arm_q_deg} | qdot_arm={qdot_arm}")

            # Mantieni loop
            time.sleep(max(0.0, dt - (time.time() - now)))
            if t >= SEG_DURATION:
                break
    except KeyboardInterrupt:
        pass

    # Plot traiettoria dell'end-effector nel frame WORLD (3D)
    try:
        import matplotlib.pyplot as plt
        P = np.array(p_log)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(P[:, 0], P[:, 1], P[:, 2], label='EE path (WORLD)')
        ax.scatter(P[0, 0], P[0, 1], P[0, 2], c='g', s=40, label='start')
        ax.scatter(P[-1, 0], P[-1, 1], P[-1, 2], c='r', s=40, label='end')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_title('Traiettoria EE in WORLD')
        ax.legend(loc='best')
        # Aspetto isometrico
        try:
            # Disponibile su Matplotlib >= 3.3
            ranges = P.max(axis=0) - P.min(axis=0)
            max_range = max(ranges.max(), 1e-6)
            x_mid = (P[:, 0].max() + P[:, 0].min()) / 2.0
            y_mid = (P[:, 1].max() + P[:, 1].min()) / 2.0
            z_mid = (P[:, 2].max() + P[:, 2].min()) / 2.0
            ax.set_box_aspect((1, 1, 1))
            ax.set_xlim(x_mid - max_range / 2, x_mid + max_range / 2)
            ax.set_ylim(y_mid - max_range / 2, y_mid + max_range / 2)
            ax.set_zlim(z_mid - max_range / 2, z_mid + max_range / 2)
        except Exception:
            pass
        plt.show()
    except Exception as e:
        print(f"Plot della traiettoria non disponibile: {e}")

    print("Terminato.")


if __name__ == "__main__":
    main()
