#!/usr/bin/env python3
"""
Test CRBA e confronto della matrice d'inerzia generalizzata M(q)
in convenzione LOCAL (default) vs WORLD con Pinocchio,
per il manipolatore aereo (drone + braccio) modellato con base flottante.

Cosa fa:
- Carica l'URDF completo (t960a.urdf) con JointModelFreeFlyer
- Calcola la matrice di massa generalizzata M_local(q) via CRBA (convenzione LOCAL)
- Trasforma M_local in M_world usando l'Adjoint della posa della base: M_world = S^T M_local S
    dove S = blockdiag(Ad_bw, I) e Ad_bw è l'Adjoint da WORLD a BASE
- Stampa a schermo dimensioni e sottomatrici per il confronto

Nota: Pinocchio fornisce CRBA in coordinate di velocità del free-flyer espresse nel frame LOCAL.
Per ottenere la convenzione WORLD si applica un cambio di coordinate tramite l'Adjoint della base.

Requisiti Python: pinocchio, numpy
"""
import os
import sys
import time
import numpy as np

import pinocchio as pin
import pinocchio.visualize

EE_FRAME = "mobile_wx250s/ee_gripper_link"


def find_urdf_and_pkg_dir():
    """Restituisce (urdf_filename, pkg_dir) per t960a.urdf.
    Prova nel path install della workspace e poi nella workspace sorgente.
    Allineato agli altri script in "scripts/".
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


## Nessuna trasformazione manuale: useremo direttamente l'opzione Convention di crba


def main():
    # Carica modello URDF con base flottante
    urdf_filename, pkg_dir = find_urdf_and_pkg_dir()
    model, cmodel, vmodel = pin.buildModelsFromUrdf(
        urdf_filename,
        package_dirs=[pkg_dir],
        root_joint=pin.JointModelFreeFlyer(),
    )
    data = model.createData()

    # Configurazione iniziale: neutra
    q = pin.neutral(model)

    # Verifica frame EE e id (serve anche per la stampa a q neutra)
    if not model.existFrame(EE_FRAME):
        raise RuntimeError(f"Frame EE non trovato: {EE_FRAME}")
    ee_frame_id = model.getFrameId(EE_FRAME)

    # Stampa Jgen nella configurazione iniziale neutra
    q0 = q.copy()
    pin.forwardKinematics(model, data, q0)
    pin.computeJointJacobians(model, data, q0)
    pin.updateFramePlacements(model, data)
    J0 = pin.computeFrameJacobian(model, data, q0, ee_frame_id, pin.ReferenceFrame.WORLD)
    Jb0 = J0[:, :6]
    Jm0 = J0[:, 6:]
    M_world0 = pin.crba(model, data, q0, pin.Convention.WORLD)
    M_world0 = (M_world0 + M_world0.T) * 0.5
    Hb0 = M_world0[:6, :6]
    Hbm0 = M_world0[:6, 6:]
    eps_Hb0 = 1e-9
    Hb0_reg = Hb0 + eps_Hb0 * np.eye(6)
    Hb0_inv_Hbm0 = np.linalg.solve(Hb0_reg, Hbm0)
    Jgen0 = Jm0 - Jb0 @ Hb0_inv_Hbm0
    np.set_printoptions(precision=5, suppress=True)
    print("\nJgen (neutro) shape:", Jgen0.shape)
    print("Jgen (neutro):\n", Jgen0)

    # # Imposta una posa non banale della base (per evidenziare la differenza LOCAL vs WORLD)
    # # Traslazione
    # q[0:3] = np.array([0.3, -0.2, 0.4])
    # # Rotazione da RPY (roll, pitch, yaw)
    # R = pin.rpy.rpyToMatrix(0.2, -0.1, 0.5)
    # quat = pin.Quaternion(R)  # XYZW
    # q[3:7] = np.array([quat.x, quat.y, quat.z, quat.w])

    # # Calcolo CRBA (matrice di massa generalizzata) in convenzione LOCAL
    # M_local = pin.crba(model, data, q, pin.Convention.LOCAL).copy()
    # # Assicura simmetria numerica
    # M_local = (M_local + M_local.T) * 0.5

    # np.set_printoptions(precision=5, suppress=True)
    # sl = slice(0, min(12, model.nv))
    # print("M_local(q) shape:", M_local.shape)
    # print("\nM_local (prime 12x12):\n", M_local[sl, sl])

    # Visualizzazione con MeshCat (posa drone+braccio)
    try:
        viz = pin.visualize.MeshcatVisualizer(model, cmodel, vmodel)
        # Prova ad aprire il browser automaticamente; se fallisce, resta in background
        viz.initViewer(open=True)

        # Carica il modello
        viz.loadViewerModel()

        # Mostra solo i visual (non le collisioni)
        viz.displayCollisions(False)
        viz.displayVisuals(True)

        # Visualizza la posa corrente
        viz.display(q)

        # Prova a stampare l'URL del viewer
        viewer_url = None
        try:
            viewer_url = viz.viewer.url()
        except Exception:
            pass
        if viewer_url:
            print(f"MeshCat URL: {viewer_url}")
            print("Apri il link sopra se il browser non si è aperto automaticamente.")
        else:
            print("MeshCat: visualizzazione inizializzata e posa mostrata (URL non disponibile).")

        # Mantieni la finestra viva per qualche secondo per permettere il rendering
        time.sleep(5.0)
    except Exception as e:
        print(f"MeshCat non disponibile: {e}")

    # =============================
    # Loop: Jgen IK + integrazione
    # =============================
    def damped_pinv(J: np.ndarray, lamb: float = 1e-6) -> np.ndarray:
        m, n = J.shape
        if m <= n:
            JJt = J @ J.T
            return J.T @ np.linalg.inv(JJt + (lamb ** 2) * np.eye(m))
        else:
            JtJ = J.T @ J
            return np.linalg.inv(JtJ + (lamb ** 2) * np.eye(n)) @ J.T

    rate_hz = 100.0
    dt = 1.0 / rate_hz
    T_total = 12.0  # durata traiettoria circolare [s]
    lamb_pinv = 1e-5
    eps_Hb = 1e-9

    # Logging
    t_log = []
    base_p_log = []  # posizione base
    yaw_log = []
    joints_log = []  # angoli giunti del braccio (ultimi nv-6)
    ee_p_log = []    # posizione end-effector

    # Parametri traiettoria circolare (WORLD): piano x-z, orientazione costante
    # Centro scelto per passare da p0 al tempo t=0 sul bordo lungo +X
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    T_we0 = data.oMf[ee_frame_id]
    p0 = np.array(T_we0.translation).reshape(3)
    RADIUS = 0.10
    center = p0.copy()
    center[0] = p0[0] - RADIUS
    center[1] = p0[1]
    center[2] = p0[2]
    omega_y = 2.0 * np.pi / T_total  # [rad/s]
    e_y = np.array([0.0, 1.0, 0.0])

    t0 = time.time()
    last_draw = t0
    print("\nAvvio loop Jgen IK...")

    while True:
        t = time.time() - t0

        # Cinematica e Jacobiani
        pin.forwardKinematics(model, data, q)
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)

        # Jacobiano frame EE espresso in LOCAL_WORLD_ALIGNED (righe [lin; ang] usate qui)
        J = pin.computeFrameJacobian(model, data, q, ee_frame_id, pin.ReferenceFrame.WORLD)

        # Posizione end-effector (WORLD)
        p_ee = np.array(data.oMf[ee_frame_id].translation).reshape(3)

        # Partizioni Jacobiano
        Jb = J[:, :6]
        Jm = J[:, 6:]

        # Matrice d'inerzia (WORLD) e partizioni
        M_world = pin.crba(model, data, q, pin.Convention.WORLD)
        M_world = (M_world + M_world.T) * 0.5
        Hb = M_world[:6, :6]
        Hbm = M_world[:6, 6:]

        # Jgen = Jm - Jb * Hb^{-1} * Hbm
        Hb_reg = Hb + eps_Hb * np.eye(6)
        Hb_inv_Hbm = np.linalg.solve(Hb_reg, Hbm)
        Jgen = Jm - Jb @ Hb_inv_Hbm

        # Velocità desiderata dell'EE (WORLD/LWA): traiettoria circolare su piano x-z
        if t <= T_total:
            r = p_ee - center
            v_lin_des = omega_y * np.cross(e_y, r)
            omega_des = np.zeros(3)  # orientazione costante
            v_ee_des = np.hstack([v_lin_des, omega_des])  # [lin; ang]
        else:
            v_ee_des = np.zeros(6)

        # IK a velocità sui soli giunti del braccio
        Jgen_pinv = damped_pinv(Jgen, lamb=lamb_pinv)
        qdot_arm = Jgen_pinv @ v_ee_des
        # Clipping sicurezza
        qdot_arm = np.clip(qdot_arm, -2.0, 2.0)

        # v_base = - Hb^{-1} * Hbm * qdot_arm
        v_base = - np.linalg.solve(Hb_reg, Hbm @ qdot_arm)

        # Componi velocità generalizzata e integra
        v_full = np.zeros(model.nv)
        v_full[:6] = v_base
        v_full[6:] = qdot_arm
        q = pin.integrate(model, q, v_full * dt)

        # Visualizzazione a rate fisso
        now = time.time()
        if 'viz' in locals() and (now - last_draw) >= dt:
            try:
                viz.display(q)
            except Exception:
                pass
            last_draw = now

        # Log configurazioni
        T_wb = data.oMi[1]
        base_p = np.array(T_wb.translation).reshape(3)
        R_wb = T_wb.rotation
        # estrai yaw da R_wb (Z-Y-X yaw-pitch-roll)
        yaw = np.arctan2(R_wb[1,0], R_wb[0,0])
        t_log.append(t)
        base_p_log.append(base_p)
        yaw_log.append(yaw)
        joints_log.append(np.array(q[-(model.nv-6):]))  # parte giunti (approssimazione)
        ee_p_log.append(p_ee)

        # Esci
        if t >= T_total:
            break
        # Rispetta dt
        time.sleep(max(0.0, dt - (time.time() - now)))

    # Plot traiettoria EE (3D) e, opzionale, componenti x,y,z vs tempo
    try:
        import matplotlib.pyplot as plt
        t_arr = np.array(t_log)
        ee_p_arr = np.vstack(ee_p_log)
        joints_arr = np.vstack(joints_log)

        # Figura 1: traiettoria 3D dell'end-effector (WORLD)
        fig3d = plt.figure()
        ax3d = fig3d.add_subplot(111, projection='3d')
        ax3d.plot(ee_p_arr[:, 0], ee_p_arr[:, 1], ee_p_arr[:, 2], label='EE path (WORLD)')
        ax3d.scatter(ee_p_arr[0, 0], ee_p_arr[0, 1], ee_p_arr[0, 2], c='g', s=40, label='start')
        ax3d.scatter(ee_p_arr[-1, 0], ee_p_arr[-1, 1], ee_p_arr[-1, 2], c='r', s=40, label='end')
        ax3d.set_xlabel('X [m]')
        ax3d.set_ylabel('Y [m]')
        ax3d.set_zlabel('Z [m]')
        ax3d.set_title('Traiettoria EE in WORLD')
        ax3d.legend(loc='best')
        try:
            # Aspetto isometrico
            ranges = ee_p_arr.max(axis=0) - ee_p_arr.min(axis=0)
            max_range = max(ranges.max(), 1e-6)
            x_mid = (ee_p_arr[:, 0].max() + ee_p_arr[:, 0].min()) / 2.0
            y_mid = (ee_p_arr[:, 1].max() + ee_p_arr[:, 1].min()) / 2.0
            z_mid = (ee_p_arr[:, 2].max() + ee_p_arr[:, 2].min()) / 2.0
            ax3d.set_box_aspect((1, 1, 1))
            ax3d.set_xlim(x_mid - max_range / 2, x_mid + max_range / 2)
            ax3d.set_ylim(y_mid - max_range / 2, y_mid + max_range / 2)
            ax3d.set_zlim(z_mid - max_range / 2, z_mid + max_range / 2)
        except Exception:
            pass

        # Figura 2: componenti x,y,z dell'EE nel tempo e (opzionale) primi giunti
        n_j = joints_arr.shape[1] if joints_arr.ndim == 2 else 0
        nrows = 2 if n_j > 0 else 1
        fig2, axes = plt.subplots(nrows, 1, figsize=(10, 6), squeeze=False)
        axes = axes.flatten()

        axes[0].plot(t_arr, ee_p_arr[:, 0], label='x')
        axes[0].plot(t_arr, ee_p_arr[:, 1], label='y')
        axes[0].plot(t_arr, ee_p_arr[:, 2], label='z')
        axes[0].set_title('Posizione end-effector [m]')
        axes[0].set_xlabel('t [s]')
        axes[0].legend()

        if n_j > 0:
            max_show = min(4, n_j)
            for i in range(max_show):
                axes[1].plot(t_arr, joints_arr[:, i], label=f'joint{i+1}')
            axes[1].set_title('Giunti braccio (primi) [rad]')
            axes[1].set_xlabel('t [s]')
            axes[1].legend()

        plt.tight_layout()
        plt.show()
    except Exception as e:
        print(f"Plot non disponibile: {e}")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Errore: {e}")
        sys.exit(1)
