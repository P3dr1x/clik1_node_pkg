#!/usr/bin/env python3

import argparse
import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import OperatingModes
from sensor_msgs.msg import JointState


DEFAULT_Q_START = [0.0, -0.78, 0.52, 0.0, 0.26, 0.0]
DEFAULT_PROFILE_VELOCITY = 3000
DEFAULT_PROFILE_ACCELERATION = 1000

ARM_JOINT_NAMES = [
    "waist",
    "shoulder",
    "elbow",
    "forearm_roll",
    "wrist_angle",
    "wrist_rotate",
]


def _try_load_modes_yaml(modes_yaml_path: str) -> Optional[dict]:
    try:
        import yaml  # type: ignore
    except Exception:
        return None

    try:
        with open(modes_yaml_path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except Exception:
        return None


def _default_modes_yaml_path() -> Optional[str]:
    # Prefer ament index if available (installed workspace).
    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore

        pkg_share = get_package_share_directory("clik1_node_pkg")
        return os.path.join(pkg_share, "config", "modes.yaml")
    except Exception:
        return None


def _extract_group_modes(modes_yaml: Optional[dict], group_name: str) -> Tuple[str, str, str, int, int]:
    """Return (cmd_type, mode, profile_type, profile_velocity, profile_acceleration) like modes.yaml.

    If parsing fails, fall back to (group, position, time, 0, 0).
    """
    mode = "position"
    profile_type = "time"
    profile_velocity = 0
    profile_acceleration = 0

    if isinstance(modes_yaml, dict):
        groups = modes_yaml.get("groups")
        if isinstance(groups, dict):
            grp = groups.get(group_name)
            if isinstance(grp, dict):
                mode = str(grp.get("operating_mode", mode))
                profile_type = str(grp.get("profile_type", profile_type))
                try:
                    profile_velocity = int(grp.get("profile_velocity", profile_velocity))
                except Exception:
                    pass
                try:
                    profile_acceleration = int(grp.get("profile_acceleration", profile_acceleration))
                except Exception:
                    pass

    return ("group", mode, profile_type, profile_velocity, profile_acceleration)


class GoToStartingConfiguration(Node):
    def __init__(
        self,
        robot_namespace: str,
        group_name: str,
        joint_positions: List[float],
        modes_yaml_path: Optional[str],
    ):
        super().__init__("go_to_starting_configuration")

        self.robot_namespace = robot_namespace.rstrip("/")
        if not self.robot_namespace.startswith("/"):
            self.robot_namespace = "/" + self.robot_namespace

        self.group_name = group_name
        self.joint_positions = joint_positions

        self._joint_state_msg: Optional[JointState] = None
        self._js_index_map: Dict[str, int] = {}

        self._pub = self.create_publisher(
            JointGroupCommand,
            "/commands/joint_group",
            10,
        )
        self._op_mode_client = self.create_client(
            OperatingModes,
            "/set_operating_modes",
        )
        self._sub_js = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_cb,
            10,
        )

        self._modes_yaml_path = modes_yaml_path
        self._modes_yaml = None
        if self._modes_yaml_path:
            self._modes_yaml = _try_load_modes_yaml(self._modes_yaml_path)

        if self._modes_yaml is None:
            self.get_logger().warn(
                "Impossibile leggere modes.yaml (PyYAML mancante o file non trovato). "
                "Il ripristino userà fallback: profile_type='time', vel=0, accel=0."
            )

    def _joint_state_cb(self, msg: JointState) -> None:
        self._joint_state_msg = msg
        if not self._js_index_map and msg.name:
            self._js_index_map = {name: i for i, name in enumerate(msg.name)}

    def _call_set_operating_modes(
        self,
        mode: str,
        profile_type: str,
        profile_velocity: int,
        profile_acceleration: int,
        timeout_sec: float = 10.0,
    ) -> bool:
        service_name = "/set_operating_modes"
        if not self._op_mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"Servizio '{service_name}' non disponibile.")
            return False

        req = OperatingModes.Request()
        req.cmd_type = "group"
        req.name = self.group_name
        req.mode = mode
        req.profile_type = profile_type
        req.profile_velocity = int(profile_velocity)
        req.profile_acceleration = int(profile_acceleration)

        future = self._op_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error(f"Timeout chiamando '{service_name}'.")
            return False

        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"Eccezione dal servizio '{service_name}': {e}")
            return False

        if resp is None:
            self.get_logger().error(f"Risposta None dal servizio '{service_name}'.")
            return False

        return True

    def _publish_joint_group_command(self) -> None:
        msg = JointGroupCommand()
        msg.name = self.group_name
        msg.cmd = list(self.joint_positions)
        self._pub.publish(msg)

    def _wait_until_reached(
        self,
        target_positions: List[float],
        tol_rad: float = 0.01,
        stable_required: int = 10,
        timeout_sec: float = 5.0,
    ) -> bool:
        start_time = time.time()
        stable_count = 0

        while rclpy.ok() and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self._joint_state_msg is None or not self._js_index_map:
                continue

            positions = self._joint_state_msg.position
            if not positions:
                continue

            ok = True
            for joint_name, target in zip(ARM_JOINT_NAMES, target_positions):
                idx = self._js_index_map.get(joint_name)
                if idx is None or idx >= len(positions):
                    ok = False
                    break
                if abs(float(positions[idx]) - float(target)) > tol_rad:
                    ok = False
                    break

            if ok:
                stable_count += 1
                if stable_count >= stable_required:
                    return True
            else:
                stable_count = 0

        return False

    def run(self) -> None:
        # Read defaults from modes.yaml (if possible) to keep consistent profile_type/mode.
        cmd_type, default_mode, default_profile_type, default_vel, default_accel = _extract_group_modes(
            self._modes_yaml,
            self.group_name,
        )
        _ = cmd_type  # reserved if needed in future

        self.get_logger().info(
            f"Impostazione profili lenti: profile_type='{default_profile_type}', "
            f"vel={DEFAULT_PROFILE_VELOCITY}, accel={DEFAULT_PROFILE_ACCELERATION}"
        )
        if not self._call_set_operating_modes(
            mode=default_mode,
            profile_type=default_profile_type,
            profile_velocity=DEFAULT_PROFILE_VELOCITY,
            profile_acceleration=DEFAULT_PROFILE_ACCELERATION,
        ):
            self.get_logger().error("Impostazione profili lenti fallita. Interrompo.")
            return

        time.sleep(0.5)

        self.get_logger().info(f"Invio configurazione di partenza: {self.joint_positions}")
        self._publish_joint_group_command()

        reached = self._wait_until_reached(self.joint_positions)
        if reached:
            self.get_logger().info("Configurazione raggiunta (entro tolleranza).")
        else:
            self.get_logger().warn(
                "Timeout nell'attesa del raggiungimento configurazione. "
                "Procedo comunque al ripristino profili."
            )

        self.get_logger().info(
            f"Ripristino profili da modes.yaml: profile_type='{default_profile_type}', vel={default_vel}, accel={default_accel}"
        )
        if self._call_set_operating_modes(
            mode=default_mode,
            profile_type=default_profile_type,
            profile_velocity=default_vel,
            profile_acceleration=default_accel,
        ):
            self.get_logger().info("Profili ripristinati correttamente.")
        else:
            self.get_logger().warn("Ripristino profili fallito.")


def _parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Imposta profili lenti per il gruppo 'arm', manda il braccio a una configurazione, "
            "poi ripristina i profili da modes.yaml."
        )
    )

    parser.add_argument(
        "--robot_namespace",
        type=str,
        default="/mobile_wx250s",
        help="Namespace del robot (default: /mobile_wx250s)",
    )
    parser.add_argument(
        "--group_name",
        type=str,
        default="arm",
        help="Nome del gruppo joint (default: arm)",
    )
    parser.add_argument(
        "--joint_positions",
        "--q",
        nargs=6,
        type=float,
        default=DEFAULT_Q_START,
        help=(
            "Configurazione desiderata (6 valori) in radianti. "
            "Default: [0.0, -0.78, 0.52, 0.0, 0.26, 0.0]"
        ),
    )
    parser.add_argument(
        "--modes_yaml",
        type=str,
        default=None,
        help="Percorso a modes.yaml (default: <share>/clik1_node_pkg/config/modes.yaml se disponibile)",
    )

    args, _unknown = parser.parse_known_args(argv)
    return args


def main(argv: Optional[List[str]] = None) -> None:
    args = _parse_args(argv)

    modes_yaml = args.modes_yaml
    if modes_yaml is None:
        modes_yaml = _default_modes_yaml_path()

    rclpy.init(args=None)
    node = GoToStartingConfiguration(
        robot_namespace=args.robot_namespace,
        group_name=args.group_name,
        joint_positions=list(args.joint_positions),
        modes_yaml_path=modes_yaml,
    )

    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
