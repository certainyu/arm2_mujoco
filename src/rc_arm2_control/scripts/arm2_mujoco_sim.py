#!/usr/bin/env python3
"""MuJoCo torque simulator with a viewer window for arm2."""

from __future__ import annotations

import math
import os
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


def parse_vec(text: Optional[str], default: Iterable[float]) -> List[float]:
    if text is None:
        return list(default)
    return [float(item) for item in text.split()]


def fmt(values: Iterable[float]) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def rpy_to_quat(rpy: Iterable[float]) -> np.ndarray:
    roll, pitch, yaw = [float(v) for v in rpy]
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return np.array([
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr,
    ], dtype=float)


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=float)


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    rotated = quat_mul(quat_mul(q, np.array([0.0, v[0], v[1], v[2]], dtype=float)), quat_conj(q))
    return rotated[1:4]


def strip_file_uri(path: str) -> str:
    if path.startswith("file://"):
        return path[len("file://") :]
    return path


def resolve_mesh_path(filename: str, urdf_path: Path) -> Path:
    candidates: List[Path] = []

    if filename.startswith("package://"):
        package_path = filename[len("package://") :]
        package_name, _, relative_path = package_path.partition("/")
        if package_name and relative_path:
            try:
                candidates.append(Path(get_package_share_directory(package_name)) / relative_path)
            except Exception:
                pass
    else:
        path = Path(strip_file_uri(filename))
        if path.is_absolute():
            candidates.append(path)
        else:
            candidates.append((urdf_path.parent / path).resolve())
            candidates.append((urdf_path.parent.parent / path).resolve())

    basename = Path(strip_file_uri(filename)).name
    if basename:
        candidates.append(urdf_path.parent / "meshes" / basename)
        candidates.append(urdf_path.parent.parent / "meshes" / basename)
        try:
            candidates.append(Path(get_package_share_directory("rc_arm2_description")) / "meshes" / basename)
        except Exception:
            pass

    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    return candidates[0] if candidates else Path(strip_file_uri(filename))


class UrdfModel:
    def __init__(self, urdf_path: Path) -> None:
        self.urdf_path = urdf_path
        self.root = ET.parse(urdf_path).getroot()
        self.links: Dict[str, ET.Element] = {link.attrib["name"]: link for link in self.root.findall("link")}
        self.joints: Dict[str, ET.Element] = {joint.attrib["name"]: joint for joint in self.root.findall("joint")}
        self.child_to_joint = {
            joint.find("child").attrib["link"]: joint for joint in self.root.findall("joint")
        }
        self.parent_to_children: Dict[str, List[Tuple[str, ET.Element]]] = {}
        for joint in self.root.findall("joint"):
            parent = joint.find("parent").attrib["link"]
            child = joint.find("child").attrib["link"]
            self.parent_to_children.setdefault(parent, []).append((child, joint))

    def inertial(self, link_name: str) -> Optional[ET.Element]:
        return self.links[link_name].find("inertial")

    def visual_mesh(self, link_name: str) -> Optional[str]:
        visual = self.links[link_name].find("visual")
        if visual is None:
            return None
        mesh = visual.find("./geometry/mesh")
        if mesh is None:
            return None
        return str(resolve_mesh_path(mesh.attrib["filename"], self.urdf_path))


class Arm2MujocoSim(Node):
    def __init__(self) -> None:
        super().__init__("arm2_mujoco_sim")

        description_share = Path(get_package_share_directory("rc_arm2_description"))
        default_urdf = description_share / "urdf" / "arm2.urdf"
        self.urdf_path = Path(self.declare_parameter("urdf_path", str(default_urdf)).value)
        self.command_topic = self.declare_parameter("command_topic", "/arm2/command/effort").value
        self.joint_state_topic = self.declare_parameter("joint_state_topic", "/joint_states").value
        self.payload_command_topic = self.declare_parameter(
            "payload_command_topic", "/arm2/payload_attached"
        ).value
        self.payload_active_topic = self.declare_parameter(
            "payload_active_topic", "/arm2/payload_active"
        ).value
        self.sync_payload_with_controller = bool(
            self.declare_parameter("sync_payload_with_controller", True).value
        )
        self.enable_viewer = bool(self.declare_parameter("enable_viewer", True).value)
        self.timestep = float(self.declare_parameter("timestep", 0.001).value)
        self.joint_state_rate_hz = float(self.declare_parameter("joint_state_rate_hz", 250.0).value)
        self.joint_names = list(self.declare_parameter("joint_names", ["j1", "j2", "j3", "j4"]).value)
        self.effort_limits = np.array(
            list(self.declare_parameter("effort_limits", [10.0, 10.0, 10.0, 10.0]).value),
            dtype=float,
        )
        self.kp = np.array(
            list(self.declare_parameter("kp", [20.0, 20.0, 15.0, 5.0]).value),
            dtype=float,
        )
        self.kd = np.array(
            list(self.declare_parameter("kd", [1.5, 1.5, 1.0, 0.25]).value),
            dtype=float,
        )
        self.payload_mass = float(self.declare_parameter("payload_mass", 0.2).value)
        self.payload_cube_side = float(self.declare_parameter("payload_cube_side", 0.08).value)
        self.tool0_frame_name = self.declare_parameter("tool0_frame_name", "tool0").value
        self.payload_center_from_tool = np.array(
            [0.0, 0.0, self.payload_cube_side * 0.5],
            dtype=float,
        )
        self.identity_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

        if len(self.joint_names) != 4:
            raise RuntimeError("joint_names must contain exactly 4 names")
        if len(self.effort_limits) != 4:
            raise RuntimeError("effort_limits must contain exactly 4 values")
        if len(self.kp) != 4:
            raise RuntimeError("kp must contain exactly 4 values")
        if len(self.kd) != 4:
            raise RuntimeError("kd must contain exactly 4 values")

        mjcf_path = self._write_mjcf()
        self.model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.timestep
        self.ctrl = np.zeros(4, dtype=float)
        self.joint_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.joint_names
        ]
        self.qpos_adr = [int(self.model.jnt_qposadr[joint_id]) for joint_id in self.joint_ids]
        self.qvel_adr = [int(self.model.jnt_dofadr[joint_id]) for joint_id in self.joint_ids]

        self.payload_weld_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "payload_weld"
        )
        self.payload_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "payload_free"
        )
        self.payload_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "payload_cube"
        )
        self.tool0_body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            self.tool0_frame_name,
        )
        if self.tool0_body_id < 0:
            raise RuntimeError(f"tool0 frame/body does not exist in URDF/MJCF: {self.tool0_frame_name}")
        self.payload_qpos_adr = int(self.model.jnt_qposadr[self.payload_joint_id])
        self.payload_qvel_adr = int(self.model.jnt_dofadr[self.payload_joint_id])
        self.payload_attached = False
        self.requested_payload_attached = False
        if self.payload_weld_id >= 0:
            self.data.eq_active[self.payload_weld_id] = 0

        self.joint_state_pub = self.create_publisher(JointState, self.joint_state_topic, 20)
        self.create_subscription(JointState, self.command_topic, self._effort_callback, 20)
        self.create_subscription(Bool, self.payload_command_topic, self._payload_command_callback, 10)
        self.create_subscription(Bool, self.payload_active_topic, self._payload_active_callback, 10)

        self.publish_period = 1.0 / max(self.joint_state_rate_hz, 1.0)
        self.last_publish_time = 0.0

        self.get_logger().info(
            f"MuJoCo model loaded from generated MJCF: {mjcf_path}. "
            f"viewer={'enabled' if self.enable_viewer else 'disabled'}"
        )

    def _effort_callback(self, msg: JointState) -> None:
        command = self._read_command_joint_state(msg)
        if command is None:
            return

        target_q, target_dq, target_tau = command
        current_q = np.array([float(self.data.qpos[adr]) for adr in self.qpos_adr], dtype=float)
        current_dq = np.array([float(self.data.qvel[adr]) for adr in self.qvel_adr], dtype=float)
        values = target_tau + self.kp * (target_q - current_q) + self.kd * (target_dq - current_dq)
        if not np.all(np.isfinite(values)):
            self.get_logger().warn("Ignoring MIT command that produced non-finite torque")
            return
        self.ctrl = np.clip(values, -self.effort_limits, self.effort_limits)

    def _read_command_joint_state(
        self,
        msg: JointState,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        if len(msg.position) < 4 or len(msg.velocity) < 4 or len(msg.effort) < 4:
            self.get_logger().warn(
                "Ignoring JointState command without 4 position, velocity, and effort values"
            )
            return None

        if not msg.name:
            indices = list(range(4))
        else:
            command_index = {name: index for index, name in enumerate(msg.name)}
            try:
                indices = [command_index[name] for name in self.joint_names]
            except KeyError as error:
                self.get_logger().warn(f"Ignoring JointState command missing joint {error.args[0]}")
                return None
            if any(
                index >= len(msg.position) or
                index >= len(msg.velocity) or
                index >= len(msg.effort)
                for index in indices
            ):
                self.get_logger().warn(
                    "Ignoring JointState command whose arrays do not match named joints"
                )
                return None

        target_q = np.array([float(msg.position[index]) for index in indices], dtype=float)
        target_dq = np.array([float(msg.velocity[index]) for index in indices], dtype=float)
        target_tau = np.array([float(msg.effort[index]) for index in indices], dtype=float)
        if not (
            np.all(np.isfinite(target_q)) and
            np.all(np.isfinite(target_dq)) and
            np.all(np.isfinite(target_tau))
        ):
            self.get_logger().warn("Ignoring JointState command with non-finite values")
            return None
        return target_q, target_dq, target_tau

    def _payload_command_callback(self, msg: Bool) -> None:
        self.requested_payload_attached = bool(msg.data)
        if not self.sync_payload_with_controller:
            self._set_payload_attached(self.requested_payload_attached)

    def _payload_active_callback(self, msg: Bool) -> None:
        if self.sync_payload_with_controller:
            self._set_payload_attached(bool(msg.data))

    def _set_payload_attached(self, attached: bool) -> None:
        if attached == self.payload_attached:
            return
        if attached:
            self._move_payload_to_tool()
            self.data.eq_active[self.payload_weld_id] = 1
            self.get_logger().info("Payload cube attached in MuJoCo")
        else:
            self.data.eq_active[self.payload_weld_id] = 0
            self.get_logger().info("Payload cube detached in MuJoCo")
        self.payload_attached = attached
        mujoco.mj_forward(self.model, self.data)

    def _move_payload_to_tool(self) -> None:
        mujoco.mj_forward(self.model, self.data)
        tool0_pos = np.array(self.data.xpos[self.tool0_body_id], dtype=float)
        tool0_quat = np.array(self.data.xquat[self.tool0_body_id], dtype=float)
        payload_pos = tool0_pos + quat_rotate(tool0_quat, self.payload_center_from_tool)
        payload_quat = tool0_quat
        payload_quat = payload_quat / np.linalg.norm(payload_quat)

        qadr = self.payload_qpos_adr
        vadr = self.payload_qvel_adr
        self.data.qpos[qadr : qadr + 3] = payload_pos
        self.data.qpos[qadr + 3 : qadr + 7] = payload_quat
        self.data.qvel[vadr : vadr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def step(self) -> None:
        self.data.ctrl[:4] = self.ctrl
        mujoco.mj_step(self.model, self.data)
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if now - self.last_publish_time >= self.publish_period:
            self.last_publish_time = now
            self._publish_joint_state()

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [float(self.data.qpos[adr]) for adr in self.qpos_adr]
        msg.velocity = [float(self.data.qvel[adr]) for adr in self.qvel_adr]
        msg.effort = [float(self.ctrl[i]) for i in range(4)]
        self.joint_state_pub.publish(msg)

    def _write_mjcf(self) -> Path:
        urdf = UrdfModel(self.urdf_path)
        mjcf = ET.Element("mujoco", {"model": "arm2"})
        ET.SubElement(
            mjcf,
            "compiler",
            {
                "angle": "radian",
                "coordinate": "local",
                "inertiafromgeom": "false",
            },
        )
        ET.SubElement(
            mjcf,
            "option",
            {
                "timestep": f"{self.timestep:.12g}",
                "gravity": "0 0 -9.81",
                "integrator": "RK4",
            },
        )

        asset = ET.SubElement(mjcf, "asset")
        for link_name in urdf.links:
            mesh_path = urdf.visual_mesh(link_name)
            if mesh_path:
                ET.SubElement(
                    asset,
                    "mesh",
                    {
                        "name": f"{link_name}_mesh",
                        "file": mesh_path,
                    },
                )

        worldbody = ET.SubElement(mjcf, "worldbody")
        ET.SubElement(
            worldbody,
            "light",
            {"name": "key_light", "pos": "0 -0.7 1.3", "dir": "0 0.6 -1", "diffuse": "0.9 0.9 0.85"},
        )
        ET.SubElement(
            worldbody,
            "camera",
            {
                "name": "overview",
                "pos": "0.75 -1.2 0.75",
                "xyaxes": "0.86 0.51 0 -0.25 0.42 0.87",
            },
        )
        ET.SubElement(
            worldbody,
            "geom",
            {"name": "floor", "type": "plane", "size": "1.5 1.5 0.02", "rgba": "0.72 0.72 0.68 1"},
        )

        self._append_body(worldbody, urdf, "base_link", None)
        self._append_payload_body(worldbody)

        actuator = ET.SubElement(mjcf, "actuator")
        for i, joint_name in enumerate(self.joint_names):
            limit = abs(float(self.effort_limits[i]))
            ET.SubElement(
                actuator,
                "motor",
                {
                    "name": f"{joint_name}_motor",
                    "joint": joint_name,
                    "gear": "1",
                    "ctrllimited": "true",
                    "ctrlrange": fmt([-limit, limit]),
                },
            )

        equality = ET.SubElement(mjcf, "equality")
        ET.SubElement(
            equality,
            "weld",
            {
                "name": "payload_weld",
                "body1": self.tool0_frame_name,
                "body2": "payload_cube",
                "relpose": fmt(list(self.payload_center_from_tool) + list(self.identity_quat)),
                "solref": "0.005 1",
                "solimp": "0.9 0.95 0.001",
            },
        )

        xml_path = Path(tempfile.gettempdir()) / "arm2_generated_mujoco.xml"
        ET.indent(mjcf, space="  ")
        ET.ElementTree(mjcf).write(xml_path, encoding="utf-8", xml_declaration=True)
        return xml_path

    def _append_body(
        self,
        parent_xml: ET.Element,
        urdf: UrdfModel,
        link_name: str,
        incoming_joint: Optional[ET.Element],
    ) -> None:
        attrs = {"name": link_name}
        if incoming_joint is not None:
            origin = incoming_joint.find("origin")
            xyz = parse_vec(origin.attrib.get("xyz") if origin is not None else None, [0.0, 0.0, 0.0])
            rpy = parse_vec(origin.attrib.get("rpy") if origin is not None else None, [0.0, 0.0, 0.0])
            attrs["pos"] = fmt(xyz)
            attrs["quat"] = fmt(rpy_to_quat(rpy))
        body = ET.SubElement(parent_xml, "body", attrs)

        self._append_inertial(body, urdf, link_name)
        mesh_name = f"{link_name}_mesh"
        if urdf.visual_mesh(link_name):
            ET.SubElement(
                body,
                "geom",
                {
                    "name": f"{link_name}_visual",
                    "type": "mesh",
                    "mesh": mesh_name,
                    "contype": "0",
                    "conaffinity": "0",
                    "rgba": "0.82 0.85 0.9 1",
                },
            )

        if incoming_joint is not None and incoming_joint.attrib.get("type") == "revolute":
            axis = parse_vec(incoming_joint.find("axis").attrib.get("xyz"), [0.0, 0.0, 1.0])
            limit = incoming_joint.find("limit")
            lower = float(limit.attrib.get("lower", "-3.14159"))
            upper = float(limit.attrib.get("upper", "3.14159"))
            ET.SubElement(
                body,
                "joint",
                {
                    "name": incoming_joint.attrib["name"],
                    "type": "hinge",
                    "axis": fmt(axis),
                    "limited": "true",
                    "range": fmt([lower, upper]),
                    "damping": "0.02",
                    "armature": "0.0005",
                },
            )

        for child_name, child_joint in urdf.parent_to_children.get(link_name, []):
            self._append_body(body, urdf, child_name, child_joint)

    def _append_inertial(self, body: ET.Element, urdf: UrdfModel, link_name: str) -> None:
        inertial = urdf.inertial(link_name)
        if inertial is None:
            return
        origin = inertial.find("origin")
        xyz = parse_vec(origin.attrib.get("xyz") if origin is not None else None, [0.0, 0.0, 0.0])
        mass = float(inertial.find("mass").attrib["value"])
        inertia = inertial.find("inertia").attrib
        fullinertia = [
            float(inertia["ixx"]),
            float(inertia["iyy"]),
            float(inertia["izz"]),
            float(inertia.get("ixy", 0.0)),
            float(inertia.get("ixz", 0.0)),
            float(inertia.get("iyz", 0.0)),
        ]
        ET.SubElement(
            body,
            "inertial",
            {
                "pos": fmt(xyz),
                "mass": f"{mass:.12g}",
                "fullinertia": fmt(fullinertia),
            },
        )

    def _append_payload_body(self, worldbody: ET.Element) -> None:
        side = self.payload_cube_side
        inertia_diag = (1.0 / 6.0) * self.payload_mass * side * side
        body = ET.SubElement(
            worldbody,
            "body",
            {
                "name": "payload_cube",
                "pos": fmt([0.45, 0.0, side * 0.5]),
            },
        )
        ET.SubElement(body, "freejoint", {"name": "payload_free"})
        ET.SubElement(
            body,
            "inertial",
            {
                "pos": "0 0 0",
                "mass": f"{self.payload_mass:.12g}",
                "diaginertia": fmt([inertia_diag, inertia_diag, inertia_diag]),
            },
        )
        ET.SubElement(
            body,
            "geom",
            {
                "name": "payload_cube_geom",
                "type": "box",
                "size": fmt([side * 0.5, side * 0.5, side * 0.5]),
                "rgba": "0.95 0.58 0.18 0.92",
            },
        )


def main() -> None:
    rclpy.init()
    node = Arm2MujocoSim()

    def run_loop(viewer=None) -> None:
        next_step = time.monotonic()
        while rclpy.ok():
            if viewer is not None and not viewer.is_running():
                break
            rclpy.spin_once(node, timeout_sec=0.0)
            node.step()
            if viewer is not None:
                viewer.sync()
            next_step += node.timestep
            sleep_time = next_step - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_step = time.monotonic()

    try:
        if node.enable_viewer:
            os.environ.setdefault("MUJOCO_GL", "glfw")
            with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                viewer.cam.fixedcamid = mujoco.mj_name2id(
                    node.model, mujoco.mjtObj.mjOBJ_CAMERA, "overview"
                )
                node.get_logger().info("MuJoCo viewer window opened")
                run_loop(viewer)
        else:
            node.get_logger().warn("MuJoCo viewer disabled by parameter")
            run_loop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
