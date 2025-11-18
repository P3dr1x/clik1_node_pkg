import pinocchio as pin
import pinocchio.visualize
import numpy as np
import os

def skew(v):
    return np.array([[0.0,     -v[2],   v[1]],
                     [v[2],    0.0,    -v[0]],
                     [-v[1],   v[0],    0.0]])

def main():
    # Percorso del file URDF
    ws_path = '/home/mattia/interbotix_ws/install'
    pkg_path = os.path.join(ws_path, 'clik1_node_pkg', 'share', 'clik1_node_pkg')
    urdf_filename = os.path.join(pkg_path, 'model', 't960a.urdf')

    if not os.path.exists(urdf_filename):
        print(f"ERRORE: File URDF non trovato in '{urdf_filename}'")
        return

    # Caricamento del modello in Pinocchio
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        urdf_filename, 
        package_dirs=[os.path.join(ws_path, 'clik1_node_pkg', 'model')],
        root_joint=pin.JointModelFreeFlyer()
    )

    # Inizializzazione del visualizzatore MeshCat
    viz = pin.visualize.MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)  # Apri automaticamente il browser
    viz.loadViewerModel()

    # Calcolo della configurazione "home"
    q_home = pin.neutral(model)
    viz.display(q_home)

    # Stampa dei frame disponibili nel modello Pinocchio
    print("Frame disponibili nel modello Pinocchio:")
    for i, frame in enumerate(model.frames):
        print(i, frame.name, frame.parentJoint, frame.type)

    # Calcolo della posa relativa tra ee_gripper e base_link
    data = model.createData()
    pin.forwardKinematics(model, data, q_home)

    # Aggiorna le pose dei frame
    pin.updateFramePlacements(model, data)

    # Ottieni ID dei frame ee_gripper e base_link
    ee_frame_id = model.getFrameId("mobile_wx250s/ee_gripper_link")
    base_frame_id = model.getFrameId("mobile_wx250s/base_link")

    T_world_ee = data.oMf[ee_frame_id]
    T_world_base = data.oMf[base_frame_id]

    T_base_ee = T_world_base.inverse() * T_world_ee

    print("Posa relativa (mobile_wx250s/ee_gripper rispetto a mobile_wx250s/base_link):")
    print(f"Posizione: x={T_base_ee.translation[0]:.2f}, y={T_base_ee.translation[1]:.2f}, z={T_base_ee.translation[2]:.2f}")
    quat = pin.Quaternion(T_base_ee.rotation)
    print(f"Orientazione: qx={quat.x:.2f}, qy={quat.y:.2f}, qz={quat.z:.2f}, qw={quat.w:.2f}")

    # ==========================
    # Calcolo e stampa Jacobiani
    # ==========================
    # Nota: in Pinocchio, l'ordine delle righe dello Jacobiano è [angolare; lineare]
    # per tutte le ReferenceFrame. Le dimensioni sono (6 x nv).
    np.set_printoptions(precision=4, suppress=True)

    # Usiamo la configurazione neutra q_home già calcolata
    # LOCAL
    J_local = pin.computeFrameJacobian(
        model, data, q_home, ee_frame_id, pin.ReferenceFrame.LOCAL
    )
    # WORLD
    J_world = pin.computeFrameJacobian(
        model, data, q_home, ee_frame_id, pin.ReferenceFrame.WORLD
    )
    # LOCAL_WORLD_ALIGNED
    J_lwa = pin.computeFrameJacobian(
        model, data, q_home, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    )

    print("\nJacobiano (LOCAL) [righe: angolare; lineare], shape=", J_local.shape)
    print(J_local)
    print("\nJacobiano (WORLD) [righe: angolare; lineare], shape=", J_world.shape)
    print(J_world)
    print("\nJacobiano (LOCAL_WORLD_ALIGNED) [righe: angolare; lineare], shape=", J_lwa.shape)
    print(J_lwa)

    # ==========================
    # Verifica J_lwa ≈ Ad_rot * J_local (senza termine traslazionale)
    # dove Ad_rot = block_diag(R_ee, R_ee)
    # ==========================
    R_we = np.array(T_world_ee.rotation)
    Z3 = np.zeros((3, 3))
    Ad_rot = np.block([[R_we, Z3], [Z3, R_we]])
    J_lwa_from_local = Ad_rot @ J_local
    diff_lwa = J_lwa - J_lwa_from_local
    print("\nVerifica: ||J_lwa - Ad_rot*J_local||_max =", np.max(np.abs(diff_lwa)))

    # Analisi blocco base per LWA
    Jb_lwa = J_lwa[:, :6]
    Jb_lwa_from_local = Ad_rot @ J_local[:, :6]
    print("||Jb_lwa - Ad_rot*Jb_local||_max =", np.max(np.abs(Jb_lwa - Jb_lwa_from_local)))

    # ==========================
    # Test diretti con J_lwa: effetto velocità base su twist ee
    # Nota: in LWA il termine di accoppiamento ω×p non compare nella parte lineare
    # delle colonne angolari della base, quindi ci aspettiamo v_lin ≈ 0 per ω_base unitario.
    # Per v_base unitario, ci aspettiamo pass-through (≈ e_i).
    # ==========================
    nv = model.nv
    print("\n[LWA] Test: velocità angolare base unitaria (ω) su v_lin ee (atteso ≈ 0)")
    for ax in range(3):
        vq = np.zeros(nv)
        vq[ax] = 1.0  # ω_base = e_ax
        twist_lwa = J_lwa @ vq  # [ang; lin]
        v_lin_lwa = twist_lwa[3:6]
        print(f"ω_base = e{ax}: v_lin_lwa={v_lin_lwa} (atteso ≈ 0)")

    print("\n[LWA] Test: velocità lineare base unitaria su v_lin ee (atteso ≈ e_i)")
    for ax in range(3):
        vq = np.zeros(nv)
        vq[3 + ax] = 1.0  # v_base = e_ax
        twist_lwa = J_lwa @ vq
        v_lin_lwa = twist_lwa[3:6]
        print(f"v_base = e{ax}: v_lin_lwa={v_lin_lwa} (atteso ≈ e{ax})")

    # ==========================
    # Verifica J_world ≈ Ad_{oMf} * J_local
    # ==========================
    Ad_oMf = T_world_ee.toActionMatrix()
    J_world_from_local = Ad_oMf @ J_local
    diff_world = J_world - J_world_from_local
    print("\nVerifica: ||J_world - Ad(oMf)*J_local||_max =", np.max(np.abs(diff_world)))

    # ==========================
    # Analisi blocco base J_b (prime 6 colonne)
    # ==========================
    Jb_local = J_local[:, :6]
    Jb_world = J_world[:, :6]
    p_we = T_world_ee.translation
    print("\np_world_ee (vettore posizione ee in WORLD):", p_we)

    print("\nBlocco base J_b (LOCAL) [ang; lin]:")
    print(Jb_local)
    print("\nBlocco base J_b (WORLD) [ang; lin]:")
    print(Jb_world)

    # Confronto J_b_world vs Ad(oMf)*J_b_local
    Jb_world_from_local = Ad_oMf @ Jb_local
    print("\n||J_b_world - Ad(oMf)*J_b_local||_max =", np.max(np.abs(Jb_world - Jb_world_from_local)))

    # Stampa Ad(oMf) per riferimento
    print("\nAd(oMf) (azione su twist) del frame ee in WORLD:")
    print(Ad_oMf)

    # ==========================
    # Test diretti: effetto velocità base su twist ee
    # ==========================
    print("\nTest: effetto di velocità angolare base unitaria (ω) su velocità lineare ee (v = ω × p)")
    e = np.eye(3)
    for ax in range(3):
        vq = np.zeros(nv)
        # Indici base per FreeFlyer in Pinocchio: [0:3]=angolare, [3:6]=lineare
        vq[ax] = 1.0  # ω_base = e_ax
        twist = J_world @ vq  # [ang; lin]
        v_lin = twist[3:6]
        v_lin_expected = np.cross(e[ax], p_we)
        print(f"ω_base = e{ax}: v_lin_atteso={v_lin_expected}, v_lin_jac={v_lin}, diff={v_lin - v_lin_expected}")

    print("\nTest: effetto di velocità lineare base unitaria su v_lin ee (dovrebbe essere pass-through)")
    for ax in range(3):
        vq = np.zeros(nv)
        vq[3 + ax] = 1.0  # v_base = e_ax
        twist = J_world @ vq
        v_lin = twist[3:6]
        print(f"v_base = e{ax}: v_lin_jac={v_lin} (atteso ≈ e{ax})")

    # ==========================
    # Jacobiano WORLD shiftato all'origine mondo (cambio punto di applicazione)
    # X_shift = [[I, 0], [-skew(p_we), I]];
    # J_world_at_world_origin = X_shift @ J_world
    # ==========================
    I3 = np.eye(3)
    X_shift = np.block([[I3,             np.zeros((3,3))],
                        [-skew(p_we),    I3           ]])
    J_world_at_world_origin = X_shift @ J_world

    print("\nJacobiano (WORLD @ world origin) [righe: angolare; lineare], shape=", J_world_at_world_origin.shape)
    # Mostra solo il blocco base per compattezza
    print("Blocco base J_b (WORLD @ world origin) [ang; lin]:")
    print(J_world_at_world_origin[:, :6])

    # Test: ora l'effetto ω×p deve comparire nella parte lineare delle colonne angolari della base
    print("\n[WORLD @ world origin] Test: ω_base unitaria su v_lin ee (atteso ≈ ω × p_we)")
    for ax in range(3):
        vq = np.zeros(nv)
        vq[ax] = 1.0  # ω_base = e_ax
        twist_shifted = J_world_at_world_origin @ vq
        v_lin_shifted = twist_shifted[3:6]
        v_lin_expected_shifted = np.cross(e[ax], p_we)
        print(f"ω_base = e{ax}: v_lin_atteso={v_lin_expected_shifted}, v_lin_jac={v_lin_shifted}, diff={v_lin_shifted - v_lin_expected_shifted}")

    print("\n[WORLD @ world origin] Test: v_base unitaria su v_lin ee (atteso ≈ e_i)")
    for ax in range(3):
        vq = np.zeros(nv)
        vq[3 + ax] = 1.0  # v_base = e_ax
        twist_shifted = J_world_at_world_origin @ vq
        v_lin_shifted = twist_shifted[3:6]
        print(f"v_base = e{ax}: v_lin_jac={v_lin_shifted} (atteso ≈ e{ax})")

    # ==========================
    # Confronto con forma MATLAB del blocco base
    # Js = [[I3, -skew(p_we)]; [0, I3]]
    # ==========================
    Js_matlab = np.block([[I3,             -skew(p_we)],
                          [np.zeros((3,3)),  I3        ]])
    Jb_world_origin = J_world_at_world_origin[:, :6]
    print("\nConfronto Js (MATLAB) vs blocco base (WORLD @ world origin):")
    print("||Jb_world_origin - Js_matlab||_max =", np.max(np.abs(Jb_world_origin - Js_matlab)))

    print("Visualizzatore MeshCat avviato. Controlla la scheda del browser.")
    input("Premi INVIO per terminare.")

if __name__ == '__main__':
    main()