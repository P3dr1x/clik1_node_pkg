#!/usr/bin/env python3
"""
Simulazione con Pinocchio del manipolatore aereo (base flottante + braccio)
utilizzando la Centroidal Momentum Matrix (CMM) Ag per calcolare il Jacobiano
generalizzato:

    Jgen = Jm - Jb * Ab^{-1} * Am
    
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
ARM_BASE_LINK = "mobile_wx250s/base_link"


def skew(v: np.ndarray) -> np.ndarray:
    """Ritorna la matrice skew-simmetrica 3x3 tale che skew(v) @ w = v x w."""
    v = np.asarray(v, dtype=float).reshape(3)
    return np.array(
        [[0.0, -v[2], v[1]],
         [v[2], 0.0, -v[0]],
         [-v[1], v[0], 0.0]],
        dtype=float,
    )


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


def find_wx250s_urdf_and_pkg_dir():
    """Restituisce (urdf_filename, pkg_dir) per wx250s.urdf (solo manipolatore).
    Prova nel path install della workspace e poi nella workspace sorgente.
    """
    ws_install = "/home/mattia/interbotix_ws/install"
    cand1 = os.path.join(ws_install, "clik1_node_pkg", "share", "clik1_node_pkg", "model", "wx250s.urdf")
    pkg1 = os.path.join(ws_install, "clik1_node_pkg", "model")

    here = os.path.dirname(os.path.abspath(__file__))
    cand2 = os.path.normpath(os.path.join(here, "..", "model", "wx250s.urdf"))
    pkg2 = os.path.normpath(os.path.join(here, "..", "model"))

    if os.path.exists(cand1):
        return cand1, pkg1
    if os.path.exists(cand2):
        return cand2, pkg2

    raise FileNotFoundError(f"URDF wx250s.urdf non trovato. Cercati: {cand1} e {cand2}")


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
    parser.add_argument(
        "--kin-constraint",
        choices=["pose", "position"],
        default="pose",
        help=(
            "Seleziona il vincolo cinematico di uguaglianza: "
            "'pose' = tracking posa EE 6D; 'position' = tracking solo posizione EE 3D."
        ),
    )
    args = parser.parse_args()

    # Carica modello URDF COMPLETO (UAM) con base flottante
    urdf_filename, pkg_dir = find_urdf_and_pkg_dir()
    model, cmodel, vmodel = pin.buildModelsFromUrdf(
        urdf_filename,
        package_dirs=[pkg_dir],
        root_joint=pin.JointModelFreeFlyer(),
    )
    data = model.createData()

    # Carica modello URDF del SOLO MANIPOLATORE con base flottante
    wx_urdf_filename, wx_pkg_dir = find_wx250s_urdf_and_pkg_dir()
    model_man, _, _ = pin.buildModelsFromUrdf(
        wx_urdf_filename,
        package_dirs=[wx_pkg_dir],
        root_joint=pin.JointModelFreeFlyer(),
    )
    data_man = model_man.createData()

    # Massa totale manipolatore (per il momento del peso rispetto a O)
    m_man_tot = 0.0
    for inertia in model_man.inertias[1:]:  # salta universe
        m_man_tot += float(inertia.mass)
    g_world = np.array(model_man.gravity.linear).reshape(3)

    # Configurazione iniziale: neutra
    q = pin.neutral(model)

    # Verifica frame EE e id (serve anche per la stampa a q neutra)
    if not model.existFrame(EE_FRAME):
        raise RuntimeError(f"Frame EE non trovato: {EE_FRAME}")
    ee_frame_id = model.getFrameId(EE_FRAME)

    if not model.existFrame(ARM_BASE_LINK):
        raise RuntimeError(f"Frame base braccio non trovato nel modello completo: {ARM_BASE_LINK}")
    arm_base_frame_id = model.getFrameId(ARM_BASE_LINK)

    # Verifica che anche il manipolatore separato contenga i frame attesi
    if not model_man.existFrame(EE_FRAME):
        raise RuntimeError(f"Frame EE non trovato nel modello manipolatore: {EE_FRAME}")
    if not model_man.existFrame(ARM_BASE_LINK):
        raise RuntimeError(f"Frame base braccio non trovato nel modello manipolatore: {ARM_BASE_LINK}")

    # Prepara mappa di corrispondenza joint (full <-> manipolatore) per riordinare colonne
    # Decision variable: qdot_arm = v_full[6:] (ordine del modello completo)
    n_arm = model.nv - 6
    n_arm_man = model_man.nv - 6
    if n_arm_man != n_arm:
        raise RuntimeError(f"nv mismatch: n_arm(full)={n_arm} vs n_arm(man)={n_arm_man}. Verifica URDF wx250s.")

    # map_full_to_man[k_full] = k_man (indici 0-based dentro i blocchi dopo i 6 DoF base)
    map_full_to_man = np.full(n_arm, -1, dtype=int)
    for k_full in range(n_arm):
        gv = 6 + k_full
        jname = None
        for j in range(model.njoints):
            if model.joints[j].nv == 0:
                continue
            start = model.joints[j].idx_v
            if start <= gv < start + model.joints[j].nv:
                jname = model.names[j]
                break
        if jname is None:
            continue
        jid_man = model_man.getJointId(jname)
        if jid_man <= 0:
            continue
        idx_v_man = model_man.joints[jid_man].idx_v
        map_full_to_man[k_full] = idx_v_man - 6
    if np.any(map_full_to_man < 0):
        bad = np.where(map_full_to_man < 0)[0]
        bad_names = []
        for k in bad[:10]:
            gv = 6 + int(k)
            # best-effort name
            nm = f"vel_idx_{gv}"
            for j in range(model.njoints):
                if model.joints[j].nv == 0:
                    continue
                start = model.joints[j].idx_v
                if start <= gv < start + model.joints[j].nv:
                    nm = model.names[j]
                    break
            bad_names.append(nm)
        raise RuntimeError(f"Impossibile mappare alcuni giunti full->man (es. {bad_names}).")



    def build_q_man_from_full(q_full: np.ndarray) -> np.ndarray:
        """Costruisce q_man (FreeFlyer+giunti) imponendo la posa di ARM_BASE_LINK dal modello completo."""
        q_man = pin.neutral(model_man)
        # serve che data contenga oMf aggiornati
        T_w_armbase = data.oMf[arm_base_frame_id]
        q_man[0:3] = np.array(T_w_armbase.translation).reshape(3)
        quat = pin.Quaternion(T_w_armbase.rotation)
        quat.normalize()
        q_man[3:7] = np.array([quat.x, quat.y, quat.z, quat.w])

        # Copia posizioni giunti per nome
        for jid_man in range(2, model_man.njoints):
            if model_man.joints[jid_man].nq == 0:
                continue
            jname = model_man.names[jid_man]
            jid_full = model.getJointId(jname)
            if jid_full <= 0:
                continue
            idx_q_man = model_man.joints[jid_man].idx_q
            idx_q_full = model.joints[jid_full].idx_q
            q_man[idx_q_man] = q_full[idx_q_full]
        return q_man



    def build_v_man_from_full(v_full: np.ndarray) -> np.ndarray:
        """Costruisce v_man (FreeFlyer+giunti) usando la twist LOCAL della ARM_BASE_LINK dal modello completo."""
        v_man = np.zeros(model_man.nv)
        # twist della base del braccio (LOCAL) dal modello completo
        V_arm = pin.getFrameVelocity(model, data, arm_base_frame_id, pin.ReferenceFrame.LOCAL)
        v_man[0:3] = np.array(V_arm.linear).reshape(3)
        v_man[3:6] = np.array(V_arm.angular).reshape(3)

        # Copia velocità giunti per nome
        for jid_man in range(2, model_man.njoints):
            if model_man.joints[jid_man].nv == 0:
                continue
            jname = model_man.names[jid_man]
            jid_full = model.getJointId(jname)
            if jid_full <= 0:
                continue
            idx_v_man = model_man.joints[jid_man].idx_v
            idx_v_full = model.joints[jid_full].idx_v
            v_man[idx_v_man] = v_full[idx_v_full]
        return v_man



    # Stampa Jgen (usando Ag) nella configurazione iniziale neutra
    q0 = q.copy()
    pin.forwardKinematics(model, data, q0)
    pin.computeJointJacobians(model, data, q0)
    pin.updateFramePlacements(model, data)
    J0 = pin.computeFrameJacobian(model, data, q0, ee_frame_id, pin.ReferenceFrame.WORLD)
    Jb0 = J0[:, :6]
    Jm0 = J0[:, 6:]
    pin.computeCentroidalMap(model, data, q0)
    Ag0 = data.Ag
    Ab0 = Ag0[:, :6]
    Am0 = Ag0[:, 6:]
    eps_Ab0 = 1e-9
    Ab0_reg = Ab0 + eps_Ab0 * np.eye(6)
    Ab0_inv_Am0 = np.linalg.solve(Ab0_reg, Am0)
    Jgen0 = Jm0 - Jb0 @ Ab0_inv_Am0
    np.set_printoptions(precision=5, suppress=True)
    print("\nJgen (neutro) shape:", Jgen0.shape)
    print("Jgen (neutro):\n", Jgen0)

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
    # minimize 0.5 * || J_mom x - b_mom ||^2 + 0.5*lambda_reg*||x||^2
    # subject to:
    #   (1) vincolo cinematico di uguaglianza: J_kin x = v_ee_des  (3D o 6D)
    #   (2) box constraints su velocità/posizione giunti: l <= x <= u
    lambda_reg = 1e-4
    VEL_MAX_DEFAULT = 2.0

    # Guadagni feedback per il vincolo cinematico (stile clik_uam_node.cpp)
    k_err = 10.0

    rate_hz = 100.0
    dt = 1.0 / rate_hz
    T_total = 12.0  # durata traiettoria circolare [s]
    eps_Ab = 1e-9

    kin_dim = 6 if args.kin_constraint == "pose" else 3

    # Dimensions and limits extraction
    # position limits from URDF via Pinocchio (nq-sized), slice last n_arm
    lower_q_arm = None
    upper_q_arm = None
    try:
        lower_q = np.array(model.lowerPositionLimit).reshape(-1)
        upper_q = np.array(model.upperPositionLimit).reshape(-1)
        if lower_q.size >= 7 + n_arm and upper_q.size >= 7 + n_arm:
            lower_q_arm = lower_q[-n_arm:]
            upper_q_arm = upper_q[-n_arm:]
    except Exception:
        pass
    # velocity limits (nv-sized), slice last n_arm
    vel_lim_arm = None
    try:
        vel_lim = np.array(model.velocityLimit).reshape(-1)
        if vel_lim.size >= 6 + n_arm:
            vel_lim_arm = vel_lim[-n_arm:]
    except Exception:
        pass

    # Pre-build OSQP problem with fixed sparsity
    # Vincoli: [J_kin; I] x in [v_ee_des (eq); bounds]
    n_constr = kin_dim + n_arm

    # Costruisci pattern CSC fisso per A: per ogni colonna j
    #  - righe 0..kin_dim-1 (dense)
    #  - riga kin_dim + j (identità)
    indptr = np.zeros(n_arm + 1, dtype=np.int32)
    indices = np.zeros(n_arm * (kin_dim + 1), dtype=np.int32)
    data_A_init = np.zeros(n_arm * (kin_dim + 1), dtype=float)
    nnz_per_col = kin_dim + 1
    for j in range(n_arm + 1):
        indptr[j] = j * nnz_per_col
    for j in range(n_arm):
        base = indptr[j]
        indices[base:base + kin_dim] = np.arange(kin_dim, dtype=np.int32)
        indices[base + kin_dim] = kin_dim + j
        # inizializza A con J_kin=0 e I=1
        data_A_init[base:base + kin_dim] = 0.0
        data_A_init[base + kin_dim] = 1.0
    A_sparse = sparse.csc_matrix((data_A_init, indices, indptr), shape=(n_constr, n_arm))

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

    # Porta q_arm iniziale dentro i limiti di posizione (se disponibili)
    if lower_q_arm is not None and upper_q_arm is not None:
        q_arm_init = q[-n_arm:]
        q_arm_clipped = np.clip(q_arm_init, lower_q_arm, upper_q_arm)
        if np.any(np.abs(q_arm_init - q_arm_clipped) > 1e-12):
            print("Avviso: q_arm iniziale fuori limiti; applico clip ai giunti dell'arm.")
        q[-n_arm:] = q_arm_clipped

    # Initial q, l, u using current state q
    q_init_vec = np.zeros(n_arm)
    q_arm = q[-n_arm:]
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

    # Bounds iniziali per OSQP: primi kin_dim vincoli di uguaglianza (inizialmente 0), poi bounds giunti
    l_osqp_init = np.zeros(n_constr)
    u_osqp_init = np.zeros(n_constr)
    l_osqp_init[:kin_dim] = 0.0
    u_osqp_init[:kin_dim] = 0.0
    l_osqp_init[kin_dim:] = l_init
    u_osqp_init[kin_dim:] = u_init

    # Mantieni dita fisse: imposta l=u=0 sulle relative componenti (nel blocco identità)
    if len(fixed_arm_indices):
        for idx in fixed_arm_indices:
            if 0 <= idx < n_arm:
                l_osqp_init[kin_dim + idx] = 0.0
                u_osqp_init[kin_dim + idx] = 0.0

    # Initialize OSQP solver once
    solver = osqp.OSQP()
    solver.setup(P=P_init, q=q_init_vec, A=A_sparse, l=l_osqp_init, u=u_osqp_init,
                 warm_start=True, verbose=False)

    # Logging
    t_log = []
    base_p_log = []  # posizione base
    yaw_log = []
    joints_log = []  # angoli giunti del braccio (ultimi nv-6)
    ee_p_log = []    # posizione end-effector
    p_des_log = []   # posizione desiderata dell'EE per confronto

    # Parametri traiettoria circolare (WORLD): piano x-z, orientazione costante
    # Centro scelto per passare da p0 al tempo t=0 sul bordo lungo +X
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    T_we0 = data.oMf[ee_frame_id]
    p0 = np.array(T_we0.translation).reshape(3)
    R_we_des = np.array(T_we0.rotation).copy()  # orientazione desiderata costante (WORLD)
    RADIUS = 0.10
    center = p0.copy()
    center[0] = p0[0] - RADIUS
    center[1] = p0[1]
    center[2] = p0[2]
    omega_y = 2.0 * np.pi / T_total  # [rad/s]
    e_y = np.array([0.0, 1.0, 0.0])

    # Imposta loop deterministico con numero di passi fissato
    N_steps = int(round(T_total / dt))
    draw_hz = min(30.0, rate_hz)
    draw_stride = max(1, int(round(rate_hz / draw_hz)))
    print("\nAvvio loop reaction-torque QP (costo momento, vincolo cinematica)...")

    # Velocità precedente (necessaria per stimare p_man, K_O, v_O al tempo t_k)
    v_full_prev = np.zeros(model.nv)

    for i in range(N_steps + 1):
        loop_start = time.time()
        t = i * dt

        # Cinematica e Jacobiani (q corrente)
        pin.forwardKinematics(model, data, q)
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)

        # Jacobiano frame EE espresso in LOCAL_WORLD_ALIGNED (righe [lin; ang] usate qui)
        J = pin.computeFrameJacobian(model, data, q, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        # Posizione end-effector (WORLD)
        p_ee = np.array(data.oMf[ee_frame_id].translation).reshape(3)
        R_we = np.array(data.oMf[ee_frame_id].rotation)

        # Partizioni Jacobiano
        Jb = J[:, :6]
        Jm = J[:, 6:]

        # Centroidal Momentum Matrix (WORLD) e partizioni
        pin.computeCentroidalMap(model, data, q)
        Ag = data.Ag
        Ab = Ag[:, :6]
        Am = Ag[:, 6:]

        # Jgen = Jm - Jb * Ab^{-1} * Am
        Ab_reg = Ab + eps_Ab * np.eye(6)
        Ab_inv_Am = np.linalg.solve(Ab_reg, Am)
        Jgen = Jm - Jb @ Ab_inv_Am

        # === Costruisci Momentum Matrix del solo manipolatore rispetto al punto O (ARM_BASE_LINK) ===
        # Impongo posa base del braccio dal modello completo
        q_man = build_q_man_from_full(q)
        pin.computeCentroidalMap(model_man, data_man, q_man)
        Ag_man = data_man.Ag  # 6 x (6+n_arm)

        # Trasporto del momento: A_KO = A_K + skew(G_mO) A_p
        # dove G_mO = O - G_m (espresso in WORLD)
        O_w = q_man[0:3].copy()
        Gm_w = np.array(data_man.com[0]).reshape(3)
        GmO_w = (O_w - Gm_w)

        A_p_man = Ag_man[0:3, :]
        A_K_man = Ag_man[3:6, :]
        A_KO_man = A_K_man + skew(GmO_w) @ A_p_man

        A_KO_b_man = A_KO_man[:, 0:6]   # 3x6
        A_KO_m_man = A_KO_man[:, 6:]    # 3xn_arm (ordine del modello manipolatore)

        # Riordina colonne per combaciare con qdot_arm (ordine del modello completo)
        A_KO_m_man_full_order = A_KO_m_man[:, map_full_to_man]

        # Blocco di momento (da Jext_instructions.md): (A_KO,b^man Ab^{-1} Am + A_KO,m^man)
        J_mom = A_KO_b_man @ Ab_inv_Am + A_KO_m_man_full_order  # 3 x n_arm

        # Velocità desiderata dell'EE (WORLD/LWA): traiettoria circolare su piano x-z
        if t <= T_total:
            r = p_ee - center
            v_lin_des = omega_y * np.cross(e_y, r)
            omega_des = np.zeros(3)  # orientazione costante
            v_ee_des = np.hstack([v_lin_des, omega_des])  # [lin; ang]
        else:
            v_ee_des = np.zeros(6)

        # Riferimento posizione (WORLD)
        theta = omega_y * min(t, T_total)
        p_des = center.copy()
        p_des[0] = center[0] + RADIUS * np.cos(theta)
        p_des[2] = center[2] + RADIUS * np.sin(theta)

        # === Vincolo cinematico di uguaglianza: J_kin x = v_kin ===
        # Stile clik_uam_node.cpp: e_pos per differenza, e_ang via log3(R_des*R_cur^T)
        e_pos = (p_des - p_ee)
        R_err_world = R_we_des @ R_we.T
        e_ang = np.array(pin.log3(R_err_world)).reshape(3)
        v_kin_6d = np.zeros(6)
        v_kin_6d[0:3] = v_ee_des[0:3] + k_err * e_pos
        v_kin_6d[3:6] = v_ee_des[3:6] + k_err * e_ang

        if kin_dim == 3:
            J_kin = Jgen[0:3, :]
            v_kin = v_kin_6d[0:3]
        else:
            J_kin = Jgen
            v_kin = v_kin_6d

        # Task 2: momento manipolatore rispetto a O (formula di trasporto).
        # Usa lo stato al tempo t_k stimato dalla (q corrente, v_full_prev).
        # tau_R è impostato a zero; tau_g (momento del peso rispetto a O) viene incluso.
        pin.forwardKinematics(model, data, q, v_full_prev)
        pin.updateFramePlacements(model, data)
        q_man_meas = build_q_man_from_full(q)
        v_man_meas = build_v_man_from_full(v_full_prev)

        pin.computeCentroidalMomentum(model_man, data_man, q_man_meas, v_man_meas)
        p_man = np.array(data_man.hg.linear).reshape(3)
        K_Gm_man = np.array(data_man.hg.angular).reshape(3)

        O_w_meas = q_man_meas[0:3].copy()
        Gm_w_meas = np.array(data_man.com[0]).reshape(3)
        GmO_w_meas = (O_w_meas - Gm_w_meas)
        K_O_man = K_Gm_man + np.cross(GmO_w_meas, p_man)

        V_O_world = pin.getFrameVelocity(model, data, arm_base_frame_id, pin.ReferenceFrame.WORLD)
        v_O_world = np.array(V_O_world.linear).reshape(3)

        # Momento del peso rispetto a O: tau_g = (GmO x (m_tot * g))
        Fg_man = m_man_tot * g_world
        tau_g = np.cross(GmO_w_meas, Fg_man)

        v_mom_task = K_O_man + dt * (np.cross(v_O_world, p_man) + tau_g)

        # === Costo: solo momento (reaction torque minimization surrogate) ===
        P_dense = (J_mom.T @ J_mom) + lambda_reg * np.eye(n_arm)
        q_vec = - (J_mom.T @ v_mom_task)

        # Bounds (recompute each iter)
        q_arm = q[-n_arm:]
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

        # Mantieni dita fisse: imposta l=u=0 per le relative componenti di velocità
        if len(fixed_arm_indices):
            for idx in fixed_arm_indices:
                if 0 <= idx < n_arm:
                    l_bounds[idx] = 0.0
                    u_bounds[idx] = 0.0

        # Log riferimento posizione (diagnostica)
        p_des_log.append(p_des.copy())

        # Costruisci vettore Ax (valori di A in CSC con pattern fisso)
        # Ordine: per ogni colonna j: J_kin[:,j] poi 1.0 (identità)
        Ax_new = np.zeros(n_arm * (kin_dim + 1), dtype=float)
        for j in range(n_arm):
            base = j * (kin_dim + 1)
            Ax_new[base:base + kin_dim] = J_kin[:, j]
            Ax_new[base + kin_dim] = 1.0

        # Bounds OSQP: primi kin_dim uguali (uguaglianza), poi bounds giunti
        l_osqp = np.zeros(n_constr)
        u_osqp = np.zeros(n_constr)
        l_osqp[:kin_dim] = v_kin
        u_osqp[:kin_dim] = v_kin
        l_osqp[kin_dim:] = l_bounds
        u_osqp[kin_dim:] = u_bounds

        # Update OSQP matrices and solve
        try:
            Px_new = dense_to_triu_px(P_dense)
            solver.update(Px=Px_new, q=q_vec, Ax=Ax_new, l=l_osqp, u=u_osqp)
            res = solver.solve()
            status = res.info.status
            pos_err_norm = np.linalg.norm(p_des - p_ee)
            ang_err_norm = float(np.linalg.norm(e_ang))
            print(
                f"OSQP status: {status} | kin_dim={kin_dim} | ||e_pos|| = {pos_err_norm:.5f} | ||e_ang|| = {ang_err_norm:.5f} | ||K_O|| = {float(np.linalg.norm(K_O_man)):.5f}"
            )
            if status in ("solved", "optimal") and res.x is not None:
                qdot_arm = res.x
            else:
                qdot_arm = np.zeros(n_arm)
        except Exception as e:
            print(f"OSQP failure: {e}")
            qdot_arm = np.zeros(n_arm)

        # v_base = - Ab^{-1} * Am * qdot_arm
        v_base = - np.linalg.solve(Ab_reg, Am @ qdot_arm)

        # Componi velocità generalizzata e integra
        v_full = np.zeros(model.nv)
        v_full[:6] = v_base
        v_full[6:] = qdot_arm
        q = pin.integrate(model, q, v_full * dt)

        # Salva v_full per stimare p_man, K_O, v_O al prossimo passo
        v_full_prev = v_full.copy()

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
