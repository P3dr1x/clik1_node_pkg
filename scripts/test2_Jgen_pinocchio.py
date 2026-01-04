#!/usr/bin/env python3
"""
Simulazione con Pinocchio del manipolatore aereo (base flottante + braccio)
utilizzando le sottomatrici della matrice d'inerzia generalizzata (CRBA) per
calcolare il Jacobiano generalizzato:

    Jgen = Jm - Jb * Hb^{-1} * Hbm
"""
import os
import sys
import time
import numpy as np
import argparse
import osqp
from scipy import sparse

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
    # Argomenti CLI: abilita/disabilita realtime e fattore scala
    parser = argparse.ArgumentParser(description="Simulazione Jgen IK per manipolatore su base flottante")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--realtime", dest="realtime", action="store_true",
                       help="Abilita sleep(dt) ad ogni passo per visualizzazione in tempo reale.")
    group.add_argument("--no-realtime", dest="realtime", action="store_false",
                       help="Disabilita sleep; esecuzione il più veloce possibile.")
    parser.set_defaults(realtime=True)
    parser.add_argument("--rt-scale", type=float, default=1.0,
                        help="Fattore di scala del tempo reale (1=tempo reale, >1 più lento, <1 più veloce).")
    parser.add_argument("--position-only", action="store_true",
                        help="Usa il task solo di posizione (3D). Di default esegue tracking di posa (6D).")
    parser.add_argument("--traj", choices=["circle", "line"], default="circle",
                        help="Seleziona la traiettoria desiderata: circolare o lineare.")
    parser.add_argument("--line-length", type=float, default=0.10,
                        help="Lunghezza del segmento orizzontale [m] per la traiettoria lineare.")
    parser.add_argument("--line-duration", type=float, default=8.0,
                        help="Durata del moto lungo il segmento [s].")
    parser.add_argument("--line-axis", choices=["x", "y"], default="x",
                        help="Asse orizzontale (WORLD) lungo cui muovere l'EE nella traiettoria lineare.")
    args = parser.parse_args()
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

    # DOF braccio: totali e attivi (escludi ultime 3 DOF del gripper)
    n_arm_total = model.nv - 6
    n_arm = min(6, n_arm_total)

    # Verifica frame EE e id (serve anche per la stampa a q neutra)
    if not model.existFrame(EE_FRAME):
        raise RuntimeError(f"Frame EE non trovato: {EE_FRAME}")
    ee_frame_id = model.getFrameId(EE_FRAME)

    # Stampa Jgen (usando CRBA) nella configurazione iniziale neutra
    q0 = q.copy()
    pin.forwardKinematics(model, data, q0)
    pin.computeJointJacobians(model, data, q0)
    pin.updateFramePlacements(model, data)
    J0 = pin.computeFrameJacobian(model, data, q0, ee_frame_id, pin.ReferenceFrame.WORLD)
    Jb0 = J0[:, :6]
    Jm0 = J0[:, 6:6 + n_arm]
    # Matrice d'inerzia generalizzata (WORLD) e partizioni Hb, Hbm
    M0 = pin.crba(model, data, q0, pin.Convention.WORLD).copy()
    M0 = (M0 + M0.T) * 0.5
    Hb0 = M0[:6, :6]
    Hbm0 = M0[:6, 6:6 + n_arm]
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
    # QP-based control setup (OSQP)
    # =============================
    # QP formulation (decision x = qdot_arm, size n_arm):
    # minimize 0.5 * x^T P x + q^T x
    #   P = Jgen^T * W * Jgen + lambda_reg * I
    #   q = - Jgen^T * W * v_ee_des
    # subject to box constraints A=I, l <= x <= u
    #   joint velocity limits: dq_min <= x <= dq_max
    #   discretized position limits: (q_min - q_arm)/dt <= x <= (q_max - q_arm)/dt
    # bounds are intersected elementwise.

    # Task selection: 6D (position+orientation) o 3D (position-only) via CLI
    POSITION_ONLY = args.position_only
    W_diag = np.ones(6 if not POSITION_ONLY else 3)  # identity weights initially
    lambda_reg = 1e-4
    VEL_MAX_DEFAULT = 2.0

    rate_hz = 100.0
    dt = 1.0 / rate_hz
    T_total = 12.0  # durata traiettoria circolare [s]
    eps_Hb = 1e-9

    # Dimensions and limits extraction
    # n_arm già definito come DOF attivi (senza gripper)
    # position limits from URDF via Pinocchio (nq-sized), slice last n_arm
    lower_q_arm = None
    upper_q_arm = None
    try:
        lower_q = np.array(model.lowerPositionLimit).reshape(-1)
        upper_q = np.array(model.upperPositionLimit).reshape(-1)
        # Prendi prima le limitazioni dei giunti del braccio (tutti) e poi taglia alle DOF attive
        if lower_q.size >= 7 + n_arm_total and upper_q.size >= 7 + n_arm_total:
            lower_q_arm = lower_q[-n_arm_total:][:n_arm]
            upper_q_arm = upper_q[-n_arm_total:][:n_arm]
    except Exception:
        pass
    # velocity limits (nv-sized), slice last n_arm
    vel_lim_arm = None
    try:
        vel_lim = np.array(model.velocityLimit).reshape(-1)
        if vel_lim.size >= 6 + n_arm_total:
            vel_lim_arm = vel_lim[-n_arm_total:][:n_arm]
    except Exception:
        pass

    # Pre-build OSQP problem with fixed sparsity
    # A = Identity (box constraints)
    A_sparse = sparse.eye(n_arm, format='csc')

    # Helper to map dense symmetric P to OSQP Px (upper-triangular column-wise)
    def dense_to_triu_px(P_dense: np.ndarray) -> np.ndarray:
        n = P_dense.shape[0]
        # Column-wise upper triangular entries: for col j, rows 0..j
        vals = []
        for j in range(n):
            for i in range(j + 1):
                vals.append(P_dense[i, j])
        return np.array(vals, dtype=float)

    # Helper: map arm velocity index k (0-based within qdot_arm) to joint name
    def arm_index_to_joint_name(model: pin.Model, k: int) -> str:
        # Global velocity index in qdot_full is 6 + k (free-flyer has 6 DoF)
        gv = 6 + k
        for j in range(model.njoints):
            jnv = model.joints[j].nv
            if jnv == 0:
                continue
            start = model.joints[j].idx_v
            if start <= gv < start + jnv:
                return model.names[j]
        return f"vel_idx_{gv}"

    # Imposta esplicitamente la posa iniziale delle dita del gripper
    # per evitare infeasibilità iniziale: left_finger=+0.026, right_finger=-0.026
    try:
        lf_id = model.getJointId("left_finger")
        if lf_id > 0:
            lf_idx_q = model.joints[lf_id].idx_q
            q[lf_idx_q] = 0.026
        rf_id = model.getJointId("right_finger")
        if rf_id > 0:
            rf_idx_q = model.joints[rf_id].idx_q
            q[rf_idx_q] = -0.026
    except Exception:
        pass

    # Indici di velocità (nel vettore dell'arm) corrispondenti alle dita da mantenere fisse
    fixed_arm_indices = []
    try:
        for jname in ("left_finger", "right_finger"):
            jid = model.getJointId(jname)
            if jid > 0:
                j = model.joints[jid]
                start = j.idx_v
                jnv = j.nv
                for d in range(jnv):
                    gv = start + d
                    arm_idx = gv - 6
                    if arm_idx >= 0:
                        fixed_arm_indices.append(arm_idx)
        if len(fixed_arm_indices):
            print(f"Fisserò a zero le velocità dei seguenti indici arm (dita): {fixed_arm_indices}")
    except Exception:
        pass

    # Create initial P with full upper-triangular sparsity pattern
    # Pattern uses ones but values set to lambda_reg on diagonal, zeros elsewhere
    P_pattern = sparse.triu(np.ones((n_arm, n_arm)), format='csc')
    P_init_px = []
    for j in range(n_arm):
        for i in range(j + 1):
            P_init_px.append(lambda_reg if i == j else 0.0)
    P_init = P_pattern.copy()
    P_init.data = np.array(P_init_px, dtype=float)

    # Porta q_arm (solo DOF attive) iniziale dentro i limiti di posizione (se disponibili)
    if lower_q_arm is not None and upper_q_arm is not None:
        start_arm_q = q.size - n_arm_total
        end_arm_q = start_arm_q + n_arm
        q_arm_init = q[start_arm_q:end_arm_q]
        q_arm_clipped = np.clip(q_arm_init, lower_q_arm, upper_q_arm)
        if np.any(np.abs(q_arm_init - q_arm_clipped) > 1e-12):
            print("Avviso: q_arm iniziale fuori limiti; applico clip ai giunti dell'arm.")
        q[start_arm_q:end_arm_q] = q_arm_clipped

    # Initial q, l, u using current state q
    q_init_vec = np.zeros(n_arm)
    start_arm_q = q.size - n_arm_total
    end_arm_q = start_arm_q + n_arm
    q_arm = q[start_arm_q:end_arm_q]
    # Velocity bounds
    if vel_lim_arm is not None:
        dq_min = -np.abs(vel_lim_arm)
        dq_max = np.abs(vel_lim_arm)
    else:
        dq_min = -VEL_MAX_DEFAULT * np.ones(n_arm)
        dq_max = VEL_MAX_DEFAULT * np.ones(n_arm)
    # Position bounds discretized
    if lower_q_arm is not None and upper_q_arm is not None:
        l_pos = (lower_q_arm - q_arm) / dt
        u_pos = (upper_q_arm - q_arm) / dt
        l_init = np.maximum(dq_min, l_pos)
        u_init = np.minimum(dq_max, u_pos)
    else:
        l_init = dq_min.copy()
        u_init = dq_max.copy()

    # Diagnostica iniziale fattibilità: l_init <= u_init
    infeas_init = l_init > u_init
    if np.any(infeas_init):
        print("Avviso: vincoli iniziali infeasible (l > u) sui seguenti indici dell'arm:")
        idx0 = np.where(infeas_init)[0]
        for k in idx0:
            name = arm_index_to_joint_name(model, int(k))
            l_k = float(l_init[k])
            u_k = float(u_init[k])
            dqmin_k = float(dq_min[k])
            dqmax_k = float(dq_max[k])
            diag = f"  i={int(k)} ({name}): l={l_k:.6f}, u={u_k:.6f}, dq_min={dqmin_k:.6f}, dq_max={dqmax_k:.6f}"
            if lower_q_arm is not None and upper_q_arm is not None:
                qk = float(q_arm[k])
                qmin_k = float(lower_q_arm[k])
                qmax_k = float(upper_q_arm[k])
                lpos_k = float((lower_q_arm[k] - q_arm[k]) / dt)
                upos_k = float((upper_q_arm[k] - q_arm[k]) / dt)
                diag += f" | q={qk:.6f}, q_min={qmin_k:.6f}, q_max={qmax_k:.6f}, l_pos={lpos_k:.6f}, u_pos={upos_k:.6f}"
            print(diag)
        # Evita errore di setup: imposta l == u sugli indici infeasible
        l_init[infeas_init] = u_init[infeas_init]

    # Initialize OSQP solver once
    solver = osqp.OSQP()
    solver.setup(P=P_init, q=q_init_vec, A=A_sparse, l=l_init, u=u_init,
                 warm_start=True, verbose=False)

    # Logging
    t_log = []
    base_p_log = []  # posizione base
    yaw_log = []
    joints_log = []  # angoli giunti del braccio (ultimi nv-6)
    ee_p_log = []    # posizione end-effector
    p_des_log = []   # posizione desiderata dell'EE per confronto

    # Parametri traiettoria (WORLD)
    # Circolare: piano x-z, orientazione costante
    # Lineare: segmento orizzontale lungo asse X/Y di lunghezza specificata
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
    # Dati traiettoria lineare
    L_line = float(args.line_length)
    T_line = float(args.line_duration)
    if args.line_axis == "x":
        d_hat = np.array([1.0, 0.0, 0.0])
    else:
        d_hat = np.array([0.0, 1.0, 0.0])
    p_start = p0.copy()

    # Imposta loop deterministico con numero di passi fissato
    N_steps = int(round(T_total / dt))
    draw_hz = min(30.0, rate_hz)
    draw_stride = max(1, int(round(rate_hz / draw_hz)))
    print("\nAvvio loop Jgen IK (for con N_steps)...")

    for i in range(N_steps + 1):
        loop_start = time.time()
        t = i * dt

        # Cinematica e Jacobiani
        pin.forwardKinematics(model, data, q)
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)

        # Jacobiano frame EE espresso in LOCAL_WORLD_ALIGNED (righe [lin; ang] usate qui)
        J = pin.computeFrameJacobian(model, data, q, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        # Posizione end-effector (WORLD)
        p_ee = np.array(data.oMf[ee_frame_id].translation).reshape(3)

        # Partizioni Jacobiano
        Jb = J[:, :6]
        Jm = J[:, 6:6 + n_arm]

        # Matrice d'inerzia generalizzata (WORLD) via CRBA e partizioni
        M = pin.crba(model, data, q, pin.Convention.WORLD).copy()
        M = (M + M.T) * 0.5
        Hb = M[:6, :6]
        Hbm = M[:6, 6:6 + n_arm]

        # Jgen = Jm - Jb * Hb^{-1} * Hbm
        Hb_reg = Hb + eps_Hb * np.eye(6)
        Hb_inv_Hbm = np.linalg.solve(Hb_reg, Hbm)
        Jgen = Jm - Jb @ Hb_inv_Hbm

        # Velocità desiderata dell'EE (WORLD/LWA)
        if args.traj == "circle":
            if t <= T_total:
                r = p_ee - center
                v_lin_des = omega_y * np.cross(e_y, r)
                omega_des = np.zeros(3)  # orientazione costante
                v_ee_des = np.hstack([v_lin_des, omega_des])  # [lin; ang]
            else:
                v_ee_des = np.zeros(6)
        else:  # line
            if T_line > 0.0 and t <= T_line:
                v_scalar = L_line / T_line
                v_lin_des = v_scalar * d_hat
                omega_des = np.zeros(3)
                v_ee_des = np.hstack([v_lin_des, omega_des])
            else:
                v_ee_des = np.zeros(6)

        # Build QP for x = qdot_arm
        if POSITION_ONLY:
            J_task = Jgen[0:3, :]
            v_task = v_ee_des[0:3]
        else:
            J_task = Jgen
            v_task = v_ee_des

        # Weights (identity)
        W = np.diag(W_diag)
        P_dense = J_task.T @ (W @ J_task) + lambda_reg * np.eye(n_arm)
        q_vec = - J_task.T @ (W @ v_task)

        # Bounds (recompute each iter)
        q_arm = q[start_arm_q:end_arm_q]
        if vel_lim_arm is not None:
            dq_min = -np.abs(vel_lim_arm)
            dq_max = np.abs(vel_lim_arm)
        else:
            dq_min = -VEL_MAX_DEFAULT * np.ones(n_arm)
            dq_max = VEL_MAX_DEFAULT * np.ones(n_arm)
        if lower_q_arm is not None and upper_q_arm is not None:
            l_pos = (lower_q_arm - q_arm) / dt
            u_pos = (upper_q_arm - q_arm) / dt
            l_bounds = np.maximum(dq_min, l_pos)
            u_bounds = np.minimum(dq_max, u_pos)
        else:
            l_bounds = dq_min.copy()
            u_bounds = dq_max.copy()

        # Mantieni dita fisse: già escluse dalle DOF attive; nulla da fare

        # Posizione desiderata per diagnostica e log
        if args.traj == "circle":
            theta = omega_y * min(t, T_total)
            p_des = center.copy()
            p_des[0] = center[0] + RADIUS * np.cos(theta)
            p_des[2] = center[2] + RADIUS * np.sin(theta)
        else:  # line
            if T_line > 0.0:
                s = min(L_line, (t / T_line) * L_line)
            else:
                s = 0.0
            p_des = p_start + s * d_hat
        p_des_log.append(p_des.copy())

        # Update OSQP matrices and solve
        try:
            Px_new = dense_to_triu_px(P_dense)
            solver.update(Px=Px_new, q=q_vec, l=l_bounds, u=u_bounds)
            res = solver.solve()
            status = res.info.status
            pos_err_norm = np.linalg.norm(p_des - p_ee)
            print(f"OSQP status: {status} | ||e_pos|| = {pos_err_norm:.5f}")
            if status in ("solved", "optimal") and res.x is not None:
                qdot_arm = res.x
            else:
                qdot_arm = np.zeros(n_arm)
        except Exception as e:
            print(f"OSQP failure: {e}")
            qdot_arm = np.zeros(n_arm)

        # v_base = - Hb^{-1} * Hbm * qdot_arm
        v_base = - np.linalg.solve(Hb_reg, Hbm @ qdot_arm)

        # Componi velocità generalizzata e integra
        v_full = np.zeros(model.nv)
        v_full[:6] = v_base
        # Ricostruisci qdot dei giunti del braccio: DOF attive seguite, gripper a zero
        qdot_arm_full = np.zeros(n_arm_total)
        qdot_arm_full[:n_arm] = qdot_arm
        v_full[6:] = qdot_arm_full
        q = pin.integrate(model, q, v_full * dt)

        # Visualizzazione a rate fisso (stride rispetto al rate di controllo)
        if 'viz' in locals() and (i % draw_stride) == 0:
            try:
                viz.display(q)
            except Exception:
                pass

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

        # Sleep opzionale per visualizzazione in tempo reale
        if args.realtime:
            dt_sleep = max(0.0, dt * args.rt_scale)
            elapsed = time.time() - loop_start
            remaining = dt_sleep - elapsed
            if remaining > 0:
                time.sleep(remaining)

    # Plot traiettoria EE (3D) e, opzionale, componenti x,y,z vs tempo
    try:
        import matplotlib.pyplot as plt
        ee_p_arr = np.vstack(ee_p_log)
        ee_des_arr = np.vstack(p_des_log) if len(p_des_log) >= 2 else None

        # Figura 1: traiettoria 3D dell'end-effector (WORLD)
        fig3d = plt.figure()
        ax3d = fig3d.add_subplot(111, projection='3d')
        ax3d.plot(ee_p_arr[:, 0], ee_p_arr[:, 1], ee_p_arr[:, 2], label='EE path (WORLD)')
        if ee_des_arr is not None:
            ax3d.plot(ee_des_arr[:, 0], ee_des_arr[:, 1], ee_des_arr[:, 2], 'g--', label='EE desired (WORLD)')
        ax3d.scatter(ee_p_arr[0, 0], ee_p_arr[0, 1], ee_p_arr[0, 2], c='g', s=40, label='start')
        ax3d.scatter(ee_p_arr[-1, 0], ee_p_arr[-1, 1], ee_p_arr[-1, 2], c='r', s=40, label='end')
        ax3d.set_xlabel('X [m]')
        ax3d.set_ylabel('Y [m]')
        ax3d.set_zlabel('Z [m]')
        ax3d.set_title('Traiettoria EE in WORLD')
        ax3d.legend(loc='best')
        try:
            # Aspetto isometrico considerando entrambe le traiettorie
            if ee_des_arr is not None:
                all_pts = np.vstack([ee_p_arr, ee_des_arr])
            else:
                all_pts = ee_p_arr
            ranges = all_pts.max(axis=0) - all_pts.min(axis=0)
            max_range = max(ranges.max(), 1e-6)
            mid = (all_pts.max(axis=0) + all_pts.min(axis=0)) / 2.0
            ax3d.set_box_aspect((1, 1, 1))
            ax3d.set_xlim(mid[0] - max_range / 2, mid[0] + max_range / 2)
            ax3d.set_ylim(mid[1] - max_range / 2, mid[1] + max_range / 2)
            ax3d.set_zlim(mid[2] - max_range / 2, mid[2] + max_range / 2)
        except Exception:
            pass
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
