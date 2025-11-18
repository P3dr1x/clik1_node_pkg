#!/usr/bin/env python3
"""
DK (cinematica diretta) con Pinocchio per verificare il blocco di Jacobiano della base (free-flyer).
- Carica il modello completo (t960a.urdf) con JointModelFreeFlyer
- Impone una velocità angolare alla base (drone) lungo Z_world
- Calcola il twist dell'end-effector in WORLD con cinematica diretta (FK)
- Calcola il twist dell'EE anche come J_base * v_base
- Confronta i risultati e verifica che ω_b generi v_lin_e = ω_b × (p_e - p_b)

Requisiti Python: pinocchio, numpy
"""
import os
import time
import numpy as np
import pinocchio as pin
import pinocchio.visualize

EE_FRAME = "mobile_wx250s/ee_gripper_link"


def find_urdf_and_pkg_dir():
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


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def main():
    np.set_printoptions(precision=5, suppress=True)

    urdf_filename, pkg_dir = find_urdf_and_pkg_dir()
    model, cmodel, vmodel = pin.buildModelsFromUrdf(
        urdf_filename,
        package_dirs=[pkg_dir],
        root_joint=pin.JointModelFreeFlyer(),
    )
    data = model.createData()

    # Configurazione neutra
    q = pin.neutral(model)

    # Verifica frame EE
    assert model.existFrame(EE_FRAME), f"Frame EE non trovato: {EE_FRAME}"
    ee_frame_id = model.getFrameId(EE_FRAME)

    # Cinematica per ottenere posizioni
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    # Pose WORLD base ed EE
    base_jid = 1  # per free-flyer: primo joint dopo universe
    T_wb = data.oMi[base_jid]
    T_we = data.oMf[ee_frame_id]
    p_b = np.array(T_wb.translation).reshape(3)
    p_e = np.array(T_we.translation).reshape(3)
    r_be = p_e - p_b

    # Impongo solo velocità angolare della base lungo Z_world
    w_z = 0.5  # rad/s
    v_full = np.zeros(model.nv)
    # Assumiamo ordine spaziale [ang; lin] per il blocco free-flyer: [wx, wy, wz, vx, vy, vz]
    v_full[3:6] = np.array([0.0, 0.0, w_z])  # ω_b (WORLD)
    v_full[0:3] = np.array([0.0, 0.0, 0.0])  # v_b (WORLD)

    # FK con velocità per ottenere il twist dell'EE
    pin.forwardKinematics(model, data, q, v_full)
    pin.updateFramePlacements(model, data)
    v_ee_fk = pin.getFrameVelocity(model, data, ee_frame_id, pin.ReferenceFrame.WORLD)
    w_ee_fk = np.array(v_ee_fk.angular).reshape(3)
    v_ee_lin_fk = np.array(v_ee_fk.linear).reshape(3)

    # Jacobiano del frame EE in WORLD e blocco base
    J = pin.computeFrameJacobian(model, data, q, ee_frame_id, pin.ReferenceFrame.WORLD)
    J_base = J[:, :6]  # 6x6

    # Twist da Jacobiano: J * v_full e J_base * v_base
    v_ee_from_J = J @ v_full
    v_ee_from_Jbase = J_base @ v_full[:6]

    # Predizione teorica: [ω_b; v_b + ω_b × r]
    w_b = np.array([0.0, 0.0, w_z])
    v_b = np.zeros(3)

    print("=== Verifica Jacobiano base (WORLD) ===")
    print(f"ω_b (WORLD)         = {w_b}")
    print(f"r = p_e - p_b (m)    = {r_be}")
    print(f"J_base (6x6)         =\n{J_base}")

    # Visualizzazione del moto imposto (opzionale) con MeshCat
    try:
        viz = pin.visualize.MeshcatVisualizer(model, cmodel, vmodel)
        viz.initViewer(open=True)
        viz.loadViewerModel()
        viz.display(q)
        time.sleep(1.0)

        print("\nAvvio visualizzazione del moto imposto alla base (costante) ...")
        RATE = 100.0
        DURATION = 10.0
        dt = 1.0 / RATE
        t0 = time.time()
        last_draw = t0
        last_log_update = t0
        # Setup scia (trail) dell'end-effector in MeshCat
        trail_node = viz.viewer["traj"]["ee_trail"]
        have_meshcat_geom = False
        try:
            import meshcat.geometry as mg
            have_meshcat_geom = True
            # inizializza con un punto iniziale
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            T_we = data.oMf[ee_frame_id]
            p = np.array(T_we.translation).reshape(3)
            P = np.array(p).reshape(3, 1)
            geom = mg.PointsGeometry(P)
            mat = mg.PointsMaterial(size=0.01, color=0xff0000)
            trail_node.set_object(mg.Points(geom, mat))
        except Exception:
            pass
        last_trail_update = t0
        # Integra lo stato con la velocità imposta (v_full costante)
        while True:
            now = time.time()
            t = now - t0
            # Calcolo twist EE da FK (Pinocchio) al passo corrente
            pin.forwardKinematics(model, data, q, v_full)
            pin.updateFramePlacements(model, data)
            v_ee_fk = pin.getFrameVelocity(model, data, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            w_ee_fk = np.array(v_ee_fk.angular).reshape(3)
            v_ee_lin_fk = np.array(v_ee_fk.linear).reshape(3)

            # Calcolo twist EE da Jacobiano base * v_base
            J = pin.computeFrameJacobian(model, data, q, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            J_base = J[:, :6]
            v_base = v_full[:6]
            v_ee_from_Jb = J_base @ v_base  # spatial order [lin; ang]

            # Stampa a ~20 Hz: riordino in [v; ω] per leggibilità
            if (now - last_log_update) >= 0.05:
                twist_fk_vw = np.hstack([v_ee_lin_fk, w_ee_fk])
                twist_jb_vw = np.hstack([v_ee_from_Jb[:3], v_ee_from_Jb[3:]])
                print(f"t={t:5.2f}s | FK [v;ω]={twist_fk_vw} | Jb*qd [v;ω]={twist_jb_vw}")
                last_log_update = now

            # Integrazione stato
            q = pin.integrate(model, q, v_full * dt)
            now = time.time()
            if now - last_draw >= dt:
                viz.display(q)
                last_draw = now
            # aggiorna scia EE a 20 Hz
            if have_meshcat_geom and (now - last_trail_update) >= 0.05:
                try:
                    pin.forwardKinematics(model, data, q)
                    pin.updateFramePlacements(model, data)
                    T_we = data.oMf[ee_frame_id]
                    p = np.array(T_we.translation).reshape(3, 1)
                    # aggiungi punto
                    P = np.hstack([P, p])
                    import meshcat.geometry as mg
                    geom = mg.PointsGeometry(P)
                    mat = mg.PointsMaterial(size=0.01, color=0xff0000)
                    trail_node.set_object(mg.Points(geom, mat))
                    last_trail_update = now
                except Exception:
                    have_meshcat_geom = False
            time.sleep(max(0.0, dt - (time.time() - now)))
            if t >= DURATION:
                break
        print("Visualizzazione terminata.")
    except Exception as e:
        print(f"Visualizzazione non disponibile: {e}")


if __name__ == "__main__":
    main()
