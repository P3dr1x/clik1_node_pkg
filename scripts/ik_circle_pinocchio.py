#!/usr/bin/env python3
"""
IK a velocità con Pinocchio sul solo manipolatore mobile_wx250s (senza base volante).
- Carica il modello URDF del braccio (wx250s.urdf)
- Pianifica e insegue un segmento orizzontale nel frame WORLD con velocità lineare costante per 5 s
- Calcola qdot = J^+ * v_EE (pseudoinversa smorzata) e integra nel tempo
- Visualizzazione con MeshCat

Requisiti Python: pinocchio, meshcat, numpy
"""
import os
import time
import numpy as np

import pinocchio as pin
import pinocchio.visualize

# Parametri traiettoria e controllo
RATE = 200.0         # Hz di integrazione/visualizzazione
DAMPING = 1e-4       # smorzamento per la pseudoinversa
SEG_DURATION = 14.0  # durata del moto [s]
# Parametri per traiettoria circolare nel piano x-z (WORLD)
RADIUS = 0.10        # raggio [m]
CIRC_T = 12.0         # periodo [s]
CLOCKWISE = True     # senso di rotazione visto da +Y_world

# Nomi frame
BASE_FRAME = "mobile_wx250s/base_link"
EE_FRAME = "mobile_wx250s/ee_gripper_link"


def find_urdf_and_pkg_dir():
    """Prova più percorsi per trovare l'URDF e la cartella package per le mesh."""
    # 1) Percorso install della workspace dell'utente (se presente)
    ws_install = "/home/mattia/interbotix_ws/install"
    cand1 = os.path.join(ws_install, "clik1_node_pkg", "share", "clik1_node_pkg", "model", "wx250s.urdf")
    pkg1 = os.path.join(ws_install, "clik1_node_pkg", "model")

    # 2) Percorso relativo a questo script (workspace sorgente)
    here = os.path.dirname(os.path.abspath(__file__))
    cand2 = os.path.normpath(os.path.join(here, "..", "model", "wx250s.urdf"))
    pkg2 = os.path.normpath(os.path.join(here, "..", "model"))

    if os.path.exists(cand1):
        return cand1, pkg1
    if os.path.exists(cand2):
        return cand2, pkg2

    raise FileNotFoundError(
        f"URDF wx250s.urdf non trovato. Cercati: {cand1} e {cand2}"
    )


def damped_pseudoinverse(J, lamb=DAMPING):
    """Pseudoinversa smorzata: J^T (J J^T + λ^2 I)^-1."""
    m, n = J.shape
    if m <= n:
        JJt = J @ J.T
        return J.T @ np.linalg.inv(JJt + (lamb ** 2) * np.eye(m))
    else:
        # fallback per completezza (caso sovradimensionato)
        JtJ = J.T @ J
        return np.linalg.inv(JtJ + (lamb ** 2) * np.eye(n)) @ J.T


def main():
    urdf_filename, pkg_dir = find_urdf_and_pkg_dir()

    # Carica modello a base fissa (niente FreeFlyer)
    model, cmodel, vmodel = pin.buildModelsFromUrdf(
        urdf_filename,
        package_dirs=[pkg_dir]
    )
    data = model.createData()

    # Visualizzatore MeshCat
    viz = pin.visualize.MeshcatVisualizer(model, cmodel, vmodel)
    viz.initViewer(open=True)
    viz.loadViewerModel()

    # Configurazione iniziale: neutra
    q = pin.neutral(model)

    # Verifica frame esistenti
    assert model.existFrame(EE_FRAME), f"Frame EE non trovato: {EE_FRAME}"
    assert model.existFrame(BASE_FRAME), f"Frame di base non trovato: {BASE_FRAME}"
    ee_frame_id = model.getFrameId(EE_FRAME)

    # FK iniziale per ricavare la posizione di partenza
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    T_world_ee = data.oMf[ee_frame_id]
    p0 = np.array(T_world_ee.translation).reshape(3)

    # Jacobiano alla configurazione iniziale (espresso in WORLD)
    # Aggiorna jacobiani di giunto e pose dei frame prima del calcolo del frame Jacobian
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)
    J0 = pin.computeFrameJacobian(model, data, q, ee_frame_id, pin.ReferenceFrame.WORLD)
    np.set_printoptions(precision=5, suppress=True)
    print("Jacobiano iniziale (WORLD), shape=", J0.shape)
    print(J0)

    # Debug: vettore p_EE
    print("p_EE (WORLD)       =", p0)
    # Buffer per il plotting della traiettoria dell'origine dell'EE
    p_hist = [p0.copy()]
   

    # Centro del cerchio: lungo l'asse x del frame EE proiettato sul piano x-z del WORLD
    R_we = T_world_ee.rotation
    x_ee_world = np.array([R_we[0, 0], R_we[1, 0], R_we[2, 0]])
    x_ee_proj = np.array([x_ee_world[0], 0.0, x_ee_world[2]])
    if np.linalg.norm(x_ee_proj) < 1e-9:
        x_ee_proj = np.array([1.0, 0.0, 0.0])
    x_ee_proj = x_ee_proj / np.linalg.norm(x_ee_proj)
    c = p0 - RADIUS * x_ee_proj  # così p0 è sulla circonferenza lungo +x_ee_proj

    # Velocità angolare attorno a Y_world
    omega_mag = 2.0 * np.pi / CIRC_T
    omega_sgn = -1.0 if CLOCKWISE else 1.0
    omega_vec = np.array([0.0, omega_sgn * omega_mag, 0.0])

    # Visualizza stato iniziale
    viz.display(q)

    # Integrazione
    dt = 1.0 / RATE
    t0 = time.time()
    last_draw = t0

    # Manteniamo orientazione costante: velocità angolare desiderata = 0
    omega_des = np.zeros(3)

    # Log iniziale
    print(
        "Avvio traiettoria circolare su piano x-z: R=%.3f m, T=%.2f s, durata=%.2f s, rate=%.1f Hz"
        % (RADIUS, CIRC_T, SEG_DURATION, RATE)
    )
    print("Premi Ctrl+C per terminare anticipatamente")

    try:
        while True:
            t = time.time() - t0

            # FK e Jacobiano al passo corrente
            pin.forwardKinematics(model, data, q)
            pin.computeJointJacobians(model, data, q)
            pin.updateFramePlacements(model, data)
            # Jacobiano espresso in LOCAL_WORLD_ALIGNED:
            #  - parte lineare in WORLD (coerente con v_lin_des definita nel WORLD)
            #  - parte angolare allineata in modo da non dipendere dall'orientazione istantanea del tool
            J = pin.computeFrameJacobian(model, data, q, ee_frame_id, pin.ReferenceFrame.WORLD)
            
            # Posizione attuale dell'end-effector
            T_we = data.oMf[ee_frame_id]
            p = np.array(T_we.translation).reshape(3)
            # Salva per il plot finale
            p_hist.append(p.copy())

            # --- Calcolo velocità desiderata per traiettoria circolare ---
            # v = ω × (p - c), con ω allineato a Y_world
            if t <= SEG_DURATION:
                r = p - c
                v_lin_des = np.cross(omega_vec, r)
            else:
                v_lin_des = np.zeros(3)
            
            # Velocità EE desiderata (orientamento costante)
            v_ee_des = np.hstack([v_lin_des, omega_des])

            # Pseudoinversa smorzata e integrazione esplicita
            J_pinv = damped_pseudoinverse(J, DAMPING)
            qdot = J_pinv @ v_ee_des

            # Limita velocità giunti per stabilità (valori conservativi)
            qdot = np.clip(qdot, -2.0, 2.0)
            q = pin.integrate(model, q, qdot * dt)

            # Visualizza a rate fisso
            now = time.time()
            if now - last_draw >= dt:
                viz.display(q)
                last_draw = now

            # Stampa diagnostica a 10 Hz
            if int(t * 10) != int((t - dt) * 10):
                r = p - c
                r_norm = np.linalg.norm(r)
                dev = r_norm - RADIUS
                print(f"t={t:5.2f}s | p={p} | |p-c|={r_norm:.4f} (dev={dev:+.4f}) | v_des={v_lin_des}")

            # Mantieni loop
            time.sleep(max(0.0, dt - (time.time() - now)))
            # Termina dopo la durata richiesta
            if t >= SEG_DURATION:
                break
    except KeyboardInterrupt:
        pass
    # Plot della traiettoria dell'origine dell'EE nel piano X-Z del WORLD
    try:
        import matplotlib.pyplot as plt
        xs = [pt[0] for pt in p_hist]
        zs = [pt[2] for pt in p_hist]
        plt.figure()
        plt.plot(xs, zs, '-b', label='EE path')
        # Disegna il centro e il cerchio ideale di riferimento
        plt.scatter([p0[0]], [p0[2]], c='g', label='start')
        plt.scatter([c[0]], [c[2]], c='r', label='center')
        th = np.linspace(0, 2*np.pi, 200)
        cx = c[0] + RADIUS * np.cos(th)
        cz = c[2] + RADIUS * np.sin(th)
        plt.plot(cx, cz, '--r', alpha=0.5, label='ideal circle')
        plt.axis('equal')
        plt.xlabel('X [m]')
        plt.ylabel('Z [m]')
        plt.title('Traiettoria EE (origine ee_gripper_link) - Piano X-Z (WORLD)')
        plt.grid(True)
        plt.legend()
        plt.show()
    except ImportError:
        print("matplotlib non disponibile: salto il plot della traiettoria.")

    print("Terminato.")


if __name__ == "__main__":
    main()
