import pinocchio as pin
import pinocchio.visualize
import numpy as np
import os

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

    print("Visualizzatore MeshCat avviato. Controlla la scheda del browser.")
    input("Premi INVIO per terminare.")

if __name__ == '__main__':
    main()