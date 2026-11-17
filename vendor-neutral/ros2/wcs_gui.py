#!/usr/bin/env python3
"""WCS Control GUI — v6 (Jacobian DLS IK, orientation locked on JOG/DELTA)."""
import math
import sys
import threading
import tkinter as tk
from tkinter import ttk

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    import ikpy.chain
except ImportError:
    print("ERROR: pip install ikpy scipy", file=sys.stderr)
    sys.exit(1)

try:
    from scipy.spatial.transform import Rotation as SciRotation
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


URDF_PATH = "/workspace/kuka_kr210/urdf/kr210l150_fixed.urdf"
JOINT_NAMES = ['joint_a1', 'joint_a2', 'joint_a3',
               'joint_a4', 'joint_a5', 'joint_a6']
JOINT_BOUNDS_RAD = [
    (-3.228859, 3.228859), (-0.785398, 1.483530), (-3.665192, 1.134464),
    (-6.108653, 6.108653), (-2.181662, 2.181662), (-6.108653, 6.108653),
]
MIN_REACH, MAX_REACH = 0.5, 2.9
MIN_Z, MAX_Z = 0.1, 3.0
MAX_IK_ERROR_MM = 20.0
MAX_ORIENT_DRIFT_DEG = 3.0
PUBLISH_HZ = 20.0
GUI_REFRESH_MS = 100
JOG_STEP_M = 0.01


def _clean_rotation(R):
    U, _, Vt = np.linalg.svd(np.array(R, dtype=float))
    Rc = U @ Vt
    if np.linalg.det(Rc) < 0:
        Vt = Vt.copy()
        Vt[-1, :] *= -1
        Rc = U @ Vt
    return Rc


class WcsGuiNode(Node):
    def __init__(self):
        super().__init__('wcs_gui')
        chain_full = ikpy.chain.Chain.from_urdf_file(
            URDF_PATH, base_elements=["base_link"],
            last_link_vector=[0.0, 0.0, 0.0])
        mask = [link.name in JOINT_NAMES for link in chain_full.links]
        self.active_indices = [i for i, a in enumerate(mask) if a]
        self.chain = ikpy.chain.Chain.from_urdf_file(
            URDF_PATH, base_elements=["base_link"],
            last_link_vector=[0.0, 0.0, 0.0], active_links_mask=mask)
        for idx, (lo, hi) in zip(self.active_indices, JOINT_BOUNDS_RAD):
            self.chain.links[idx].bounds = (lo, hi)

        self._lock = threading.RLock()
        self._target_joints = [0.0] * 6
        self._actual_joints = [0.0] * 6
        self._has_target = False
        self._actual_stamp = 0.0
        self._axis_enabled = False
        self._estop = True
        self._last_wcs = None

        self.cmd_pub = self.create_publisher(JointState, '/plc/cmd_ang', 10)
        stream_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, '/joint_states', self._on_js, stream_qos)
        state_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.axis_pub = self.create_publisher(Bool, '/plc/axis_enable', state_qos)
        self.estop_pub = self.create_publisher(Bool, '/plc/emergency_stop', state_qos)
        self.create_subscription(Bool, '/plc/axis_enable', self._on_axis, state_qos)
        self.create_subscription(Bool, '/plc/emergency_stop', self._on_estop, state_qos)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

        self.get_logger().info(f"wcs_gui v6 ready — scipy={_HAS_SCIPY}")

    # ---------- subscriptions ----------
    def _on_js(self, msg):
        m = dict(zip(msg.name, msg.position))
        with self._lock:
            for i, n in enumerate(JOINT_NAMES):
                if n in m:
                    try:
                        self._actual_joints[i] = float(m[n])
                    except (TypeError, ValueError):
                        pass
            self._actual_stamp = self.get_clock().now().nanoseconds / 1e9

    def _on_axis(self, msg):
        with self._lock:
            self._axis_enabled = bool(msg.data)

    def _on_estop(self, msg):
        with self._lock:
            self._estop = bool(msg.data)

    def _publish(self):
        with self._lock:
            if not self._has_target:
                return
            j = list(self._target_joints)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "p;0;0;0;0"
        msg.name = list(JOINT_NAMES)
        msg.position = j
        self.cmd_pub.publish(msg)

    # ---------- helpers ----------
    def _get_current_joints_full(self):
        with self._lock:
            c = list(self._actual_joints)
        q = [0.0] * len(self.chain.links)
        for i, idx in enumerate(self.active_indices):
            q[idx] = c[i]
        return q

    def _get_current_wcs(self):
        q = self._get_current_joints_full()
        try:
            return self.chain.forward_kinematics(q)[:3, 3].tolist()
        except Exception:
            return None

    # ---------- Jacobian ----------
    def _compute_jacobian(self, q):
        """6 x N Jacobian by finite differences."""
        n = len(q)
        J = np.zeros((6, n))
        eps = 1e-5
        fk0 = self.chain.forward_kinematics(q)
        pos0 = fk0[:3, 3]
        R0 = _clean_rotation(fk0[:3, :3])
        for i in range(n):
            qp = list(q)
            qp[i] += eps
            fkp = self.chain.forward_kinematics(qp)
            posp = fkp[:3, 3]
            Rp = _clean_rotation(fkp[:3, :3])
            J[:3, i] = (posp - pos0) / eps
            J[3:, i] = SciRotation.from_matrix(Rp @ R0.T).as_rotvec() / eps
        return J

    def _jacobian_ik(self, target_pos, target_R, q_seed, max_iter=100, lam=0.02):
        """Damped Least Squares IK with orientation lock."""
        q = list(q_seed)
        n_active = len(self.active_indices)
        for it in range(max_iter):
            fk = self.chain.forward_kinematics(q)
            pos = fk[:3, 3]
            R = _clean_rotation(fk[:3, :3])
            e_pos = target_pos - pos
            e_rot = SciRotation.from_matrix(target_R @ R.T).as_rotvec()
            err = np.concatenate([e_pos, e_rot])
            if np.linalg.norm(err) < 1e-4:
                break
            J = self._compute_jacobian(q)[:, self.active_indices]
            JJT = J @ J.T + lam * np.eye(6)
            try:
                dq = J.T @ np.linalg.solve(JJT, err)
            except np.linalg.LinAlgError:
                break
            max_step = 0.05
            if np.max(np.abs(dq)) > max_step:
                dq = dq / np.max(np.abs(dq)) * max_step
            for i, idx in enumerate(self.active_indices):
                q[idx] += dq[i]
                lo, hi = JOINT_BOUNDS_RAD[i]
                q[idx] = max(lo, min(hi, q[idx]))

        fk = self.chain.forward_kinematics(q)
        pos_err = float(np.linalg.norm(fk[:3, 3] - target_pos) * 1000)
        R_new = _clean_rotation(fk[:3, :3])
        drift = float(math.degrees(
            SciRotation.from_matrix(target_R).inv().magnitude() -
            SciRotation.from_matrix(target_R).inv().magnitude()))  # placeholder
        drift = float(math.degrees(
            (SciRotation.from_matrix(target_R).inv() *
             SciRotation.from_matrix(R_new)).magnitude()))
        joints = [float(q[idx]) for idx in self.active_indices]
        return joints, pos_err, drift

    # ---------- IK ----------
    def _solve_ik(self, x, y, z, lock_orientation=False):
        r = math.sqrt(x*x + y*y + z*z)
        if r < MIN_REACH:
            return None, f"too close (r={r:.2f})"
        if r > MAX_REACH:
            return None, f"too far (r={r:.2f} > {MAX_REACH})"
        if z < MIN_Z:
            return None, f"below (z={z:.2f})"
        if z > MAX_Z:
            return None, f"above (z={z:.2f})"

        target = np.array([x, y, z])
        q_seed = self._get_current_joints_full()

        self.get_logger().info(
            f"[IK] target=({x:.3f},{y:.3f},{z:.3f}) lock={lock_orientation}")

        if not lock_orientation:
            try:
                ik_sol = self.chain.inverse_kinematics(
                    target_position=target,
                    initial_position=q_seed,
                    max_iter=5000)
                if ik_sol is None:
                    return None, "pos-only None"
                joints = [float(ik_sol[idx]) for idx in self.active_indices]
                for i, (j, (lo, hi)) in enumerate(zip(joints, JOINT_BOUNDS_RAD)):
                    if not (lo <= j <= hi):
                        return None, f"{JOINT_NAMES[i]} out of bounds"
                fk = self.chain.forward_kinematics(ik_sol)
                err_mm = float(np.linalg.norm(target - fk[:3, 3]) * 1000)
                if err_mm > MAX_IK_ERROR_MM:
                    return None, f"pos err {err_mm:.1f}mm"
                return joints, f"pos-only err={err_mm:.1f}mm"
            except Exception as e:
                return None, f"pos-only: {e}"

        # ---------- 6-DOF ----------
        if not _HAS_SCIPY:
            return None, "6-DOF requires scipy"

        fk_current = self.chain.forward_kinematics(q_seed)
        R_current = _clean_rotation(fk_current[:3, :3])

        # Method 1: Jacobian DLS (robust, industrial)
        try:
            joints, pos_err, drift = self._jacobian_ik(
                target, R_current, q_seed, max_iter=100, lam=0.02)
            self.get_logger().info(
                f"[DLS] pos_err={pos_err:.1f}mm drift={drift:.1f}°")
            if pos_err <= MAX_IK_ERROR_MM and drift <= MAX_ORIENT_DRIFT_DEG:
                return joints, f"DLS err={pos_err:.1f}mm drift={drift:.1f}°"
        except Exception as e:
            self.get_logger().info(f"[DLS] failed: {e}")

        # Method 2: ikpy 6-DOF
        try:
            quat_xyzw = SciRotation.from_matrix(R_current).as_quat()
            quat_wxyz = [float(quat_xyzw[3]), float(quat_xyzw[0]),
                         float(quat_xyzw[1]), float(quat_xyzw[2])]
            ik_sol = self.chain.inverse_kinematics(
                target_position=target,
                target_orientation=quat_wxyz,
                orientation_mode="all",
                initial_position=q_seed,
                max_iter=2000)
            if ik_sol is not None:
                joints = [float(ik_sol[idx]) for idx in self.active_indices]
                bounds_ok = all(lo <= j <= hi
                                for j, (lo, hi) in zip(joints, JOINT_BOUNDS_RAD))
                if bounds_ok:
                    fk = self.chain.forward_kinematics(ik_sol)
                    err_mm = float(np.linalg.norm(target - fk[:3, 3]) * 1000)
                    R_new = _clean_rotation(fk[:3, :3])
                    drift = float(math.degrees(
                        (SciRotation.from_matrix(R_current).inv() *
                         SciRotation.from_matrix(R_new)).magnitude()))
                    self.get_logger().info(
                        f"[ikpy6] err={err_mm:.1f}mm drift={drift:.1f}°")
                    if err_mm <= MAX_IK_ERROR_MM and drift <= MAX_ORIENT_DRIFT_DEG:
                        return joints, f"ikpy6 err={err_mm:.1f}mm drift={drift:.1f}°"
        except Exception as e:
            self.get_logger().info(f"[ikpy6] failed: {e}")

        return None, "6-DOF failed (target unreachable with current orientation)"

    def _set_absolute(self, x, y, z, lock_orient):
        joints, msg = self._solve_ik(x, y, z, lock_orientation=lock_orient)
        if joints is None:
            self.get_logger().warn(f"[FAIL] ({x:.3f},{y:.3f},{z:.3f}): {msg}")
            return False, msg
        with self._lock:
            self._last_wcs = [x, y, z]
            self._target_joints = joints
            self._has_target = True
        return True, msg

    def set_wcs_target_absolute(self, x, y, z, lock_orientation=False):
        return self._set_absolute(x, y, z, lock_orientation)

    def _delta_move(self, dx, dy, dz):
        base = self._ensure_last_wcs()
        if base is None:
            return False, "no WCS ref — do Absolute first"
        nx, ny, nz = base[0] + dx, base[1] + dy, base[2] + dz
        self.get_logger().info(
            f"[JOG] delta=({dx:+.3f},{dy:+.3f},{dz:+.3f}) → target=({nx:.3f},{ny:.3f},{nz:.3f})")
        ok, msg = self._set_absolute(nx, ny, nz, lock_orient=True)
        if ok:
            return True, f"→({nx:.3f},{ny:.3f},{nz:.3f}) {msg}"
        return False, msg

    def set_wcs_target_delta(self, dx, dy, dz):
        return self._delta_move(dx, dy, dz)

    def jog(self, axis, direction):
        s = JOG_STEP_M * direction
        if axis == 'x':
            return self._delta_move(s, 0, 0)
        if axis == 'y':
            return self._delta_move(0, s, 0)
        if axis == 'z':
            return self._delta_move(0, 0, s)
        return False, f"unknown axis {axis}"

    def _ensure_last_wcs(self):
        with self._lock:
            if self._last_wcs is not None:
                return list(self._last_wcs)
        cur = self._get_current_wcs()
        if cur is None:
            return None
        with self._lock:
            self._last_wcs = list(cur)
        return cur

    def set_joint_target(self, joints_rad):
        with self._lock:
            self._target_joints = list(joints_rad)
            self._has_target = True

    def compute_tool_wcs(self):
        return self._get_current_wcs()

    def get_target_wcs(self):
        with self._lock:
            return list(self._last_wcs) if self._last_wcs else None

    def get_state(self):
        with self._lock:
            return {
                'target': list(self._target_joints),
                'actual': list(self._actual_joints),
                'has_target': self._has_target,
                'stamp': self._actual_stamp,
                'axis': self._axis_enabled,
                'estop': self._estop,
            }

    def enable_axis(self, e):
        m = Bool(); m.data = e; self.axis_pub.publish(m)

    def set_estop(self, a):
        m = Bool(); m.data = a; self.estop_pub.publish(m)

    def hold_current(self):
        with self._lock:
            self._target_joints = list(self._actual_joints)
            self._has_target = True


class Gui:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        root.title("WCS Control — KUKA KR210 L150 (v6)")
        root.geometry("1000x900")

        status = ttk.LabelFrame(root, text="Status", padding=6)
        status.pack(fill=tk.X, padx=10, pady=3)
        self.axis_label = ttk.Label(status, text="Axis: ?", font=('Courier', 10, 'bold'))
        self.axis_label.pack(side=tk.LEFT, padx=8)
        self.estop_label = ttk.Label(status, text="E-stop: ?", font=('Courier', 10, 'bold'))
        self.estop_label.pack(side=tk.LEFT, padx=8)
        self.age_label = ttk.Label(status, text="joint_states: ?", font=('Courier', 9))
        self.age_label.pack(side=tk.LEFT, padx=8)
        self.target_wcs_label = ttk.Label(status, text="target WCS: —",
                                          font=('Courier', 9, 'bold'), foreground='blue')
        self.target_wcs_label.pack(side=tk.LEFT, padx=8)

        mode_frame = ttk.LabelFrame(root, text="Movement Mode", padding=6)
        mode_frame.pack(fill=tk.X, padx=10, pady=3)
        self.mode_var = tk.StringVar(value="absolute")
        for txt, val in [("Absolute", "absolute"),
                         ("Delta", "delta"),
                         ("Jog", "jog")]:
            ttk.Radiobutton(mode_frame, text=txt, variable=self.mode_var,
                            value=val, command=self._on_mode_change
                            ).pack(side=tk.LEFT, padx=10)
        self.mode_hint = ttk.Label(mode_frame, text="", foreground='gray')
        self.mode_hint.pack(side=tk.LEFT, padx=10)

        self.wcs_frame = ttk.LabelFrame(root, text="Target (base_link, meters)", padding=6)
        self.wcs_frame.pack(fill=tk.X, padx=10, pady=3)
        ttk.Label(self.wcs_frame, text="X:").grid(row=0, column=0, padx=3)
        self.wcs_x = ttk.Entry(self.wcs_frame, width=10); self.wcs_x.insert(0, "2.0")
        self.wcs_x.grid(row=0, column=1, padx=3)
        ttk.Label(self.wcs_frame, text="Y:").grid(row=0, column=2, padx=3)
        self.wcs_y = ttk.Entry(self.wcs_frame, width=10); self.wcs_y.insert(0, "0.0")
        self.wcs_y.grid(row=0, column=3, padx=3)
        ttk.Label(self.wcs_frame, text="Z:").grid(row=0, column=4, padx=3)
        self.wcs_z = ttk.Entry(self.wcs_frame, width=10); self.wcs_z.insert(0, "1.5")
        self.wcs_z.grid(row=0, column=5, padx=3)
        ttk.Button(self.wcs_frame, text="Send", command=self.send_wcs
                   ).grid(row=0, column=6, padx=15)
        self.lock_orient_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(self.wcs_frame,
                        text="Lock tool orientation (Absolute only — JOG/Delta always locked)",
                        variable=self.lock_orient_var
                        ).grid(row=1, column=0, columnspan=7, pady=4, sticky='w')

        self.jog_frame = ttk.LabelFrame(root, text="Jog (1 cm per click, orientation LOCKED)", padding=6)
        self.jog_frame.pack(fill=tk.X, padx=10, pady=3)
        jg = ttk.Frame(self.jog_frame); jg.pack()
        ttk.Button(jg, text="↑ +Z", width=12, command=lambda: self.do_jog('z', +1)
                   ).grid(row=0, column=2, padx=3, pady=2)
        ttk.Button(jg, text="← -Y", width=12, command=lambda: self.do_jog('y', -1)
                   ).grid(row=1, column=1, padx=3, pady=2)
        ttk.Button(jg, text="→ +X (fwd)", width=12, command=lambda: self.do_jog('x', +1)
                   ).grid(row=1, column=2, padx=3, pady=2)
        ttk.Button(jg, text="→ +Y", width=12, command=lambda: self.do_jog('y', +1)
                   ).grid(row=1, column=3, padx=3, pady=2)
        ttk.Button(jg, text="↓ -Z", width=12, command=lambda: self.do_jog('z', -1)
                   ).grid(row=2, column=2, padx=3, pady=2)
        ttk.Button(jg, text="← -X (back)", width=12, command=lambda: self.do_jog('x', -1)
                   ).grid(row=2, column=0, padx=3, pady=2)

        jf = ttk.LabelFrame(root, text="Joints (degrees)", padding=6)
        jf.pack(fill=tk.BOTH, expand=True, padx=10, pady=3)
        hdr = ttk.Frame(jf); hdr.pack(fill=tk.X)
        for i, h in enumerate(['Joint', 'Target (°)', 'Actual (°)', 'Diff (°)']):
            ttk.Label(hdr, text=h, width=15 if i else 10,
                      font=('Courier', 10, 'bold')).pack(side=tk.LEFT, padx=2)
        self.joint_entries = []
        self.target_labels = []
        self.actual_labels = []
        self.diff_labels = []
        for i, name in enumerate(JOINT_NAMES):
            row = ttk.Frame(jf); row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=name, width=10, font=('Courier', 10)).pack(side=tk.LEFT)
            e = ttk.Entry(row, width=15); e.insert(0, "0.0")
            e.pack(side=tk.LEFT, padx=2)
            self.joint_entries.append(e)
            for lst in (self.target_labels, self.actual_labels, self.diff_labels):
                lbl = ttk.Label(row, text="---.--", width=15, font=('Courier', 10))
                lbl.pack(side=tk.LEFT, padx=2)
                lst.append(lbl)
        btns = ttk.Frame(jf); btns.pack(pady=6)
        ttk.Button(btns, text="Send Joints", command=self.send_joints).pack(side=tk.LEFT, padx=5)
        ttk.Button(btns, text="Load Current", command=self.load_current).pack(side=tk.LEFT, padx=5)
        ttk.Button(btns, text="HOLD", command=self.hold).pack(side=tk.LEFT, padx=5)

        tw = ttk.LabelFrame(root, text="Tool WCS (FK from actual)", padding=6)
        tw.pack(fill=tk.X, padx=10, pady=3)
        self.tool_label = ttk.Label(tw, text="X: ---.--   Y: ---.--   Z: ---.--  (m)",
                                    font=('Courier', 12, 'bold'))
        self.tool_label.pack()

        sf = ttk.LabelFrame(root, text="Safety", padding=6)
        sf.pack(fill=tk.X, padx=10, pady=3)
        ttk.Button(sf, text="Enable Axis", command=self.enable_axis).pack(side=tk.LEFT, padx=5)
        ttk.Button(sf, text="Release E-stop", command=self.release_estop).pack(side=tk.LEFT, padx=5)
        tk.Button(sf, text="E-STOP!", command=self.estop, bg='#cc0000', fg='white',
                  font=('Courier', 12, 'bold')
                  ).pack(side=tk.LEFT, padx=20, fill=tk.X, expand=True)

        lf = ttk.LabelFrame(root, text="Log", padding=6)
        lf.pack(fill=tk.BOTH, expand=True, padx=10, pady=3)
        self.log_text = tk.Text(lf, height=12, font=('Courier', 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)

        self.log("v6 — Jacobian DLS IK with orientation lock")
        self.log(f"  scipy={_HAS_SCIPY}, drift_tol={MAX_ORIENT_DRIFT_DEG}°")
        self.log("  JOG/Delta: ALWAYS lock orientation")
        self.log("  Absolute: lock controlled by checkbox")
        self.log("Enable Axis + Release E-stop to start.")
        self._on_mode_change()
        self.root.after(GUI_REFRESH_MS, self.refresh)

    def _on_mode_change(self):
        hints = {
            "absolute": "(Absolute: goes to X,Y,Z)",
            "delta": "(Delta: Δ added — orientation LOCKED)",
            "jog": "(Jog: buttons — orientation LOCKED)"
        }
        self.mode_hint.config(text=hints.get(self.mode_var.get(), ""))

    def log(self, msg):
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)

    def send_wcs(self):
        try:
            x = float(self.wcs_x.get()); y = float(self.wcs_y.get()); z = float(self.wcs_z.get())
        except ValueError:
            self.log("ERR: invalid numbers"); return
        m = self.mode_var.get()
        try:
            if m == "absolute":
                ok, msg = self.node.set_wcs_target_absolute(x, y, z,
                    lock_orientation=self.lock_orient_var.get())
                self.log(f"{'OK ' if ok else 'ERR'} ABS ({x},{y},{z}) lock={self.lock_orient_var.get()}: {msg}")
            elif m == "delta":
                ok, msg = self.node.set_wcs_target_delta(x, y, z)
                self.log(f"{'OK ' if ok else 'ERR'} DELTA (Δ{x},{y},{z}): {msg}")
            else:
                self.log("INFO: in Jog mode — use buttons")
        except Exception as e:
            self.log(f"EXC send_wcs: {type(e).__name__}: {e}")

    def do_jog(self, axis, direction):
        sign = '+' if direction > 0 else '-'
        try:
            ok, msg = self.node.jog(axis, direction)
            self.log(f"{'OK ' if ok else 'ERR'} JOG {sign}{axis.upper()}: {msg}")
        except Exception as e:
            self.log(f"EXC jog({axis},{direction}): {type(e).__name__}: {e}")

    def send_joints(self):
        try:
            deg = [float(e.get()) for e in self.joint_entries]
        except ValueError:
            self.log("ERR Joints: invalid"); return
        rad = [math.radians(d) for d in deg]
        for i, (j, (lo, hi)) in enumerate(zip(rad, JOINT_BOUNDS_RAD)):
            if not (lo <= j <= hi):
                self.log(f"ERR {JOINT_NAMES[i]} out of bounds"); return
        self.node.set_joint_target(rad)
        self.log(f"OK Joints: {[round(d, 1) for d in deg]}")

    def load_current(self):
        s = self.node.get_state()
        for i, e in enumerate(self.joint_entries):
            e.delete(0, tk.END)
            e.insert(0, f"{math.degrees(s['actual'][i]):.2f}")
        self.log("OK Loaded current")

    def hold(self):
        self.node.hold_current(); self.log("OK HOLD")

    def enable_axis(self):
        self.node.enable_axis(True); self.log("OK Enable Axis")

    def release_estop(self):
        self.node.set_estop(False); self.log("OK Release E-stop")

    def estop(self):
        self.node.set_estop(True); self.log("E-STOP ACTIVATED")

    def refresh(self):
        try:
            s = self.node.get_state()
            self.axis_label.config(text=f"Axis: {'ENABLED' if s['axis'] else 'disabled'}",
                                   foreground='green' if s['axis'] else 'gray')
            self.estop_label.config(text=f"E-stop: {'ACTIVE' if s['estop'] else 'OK'}",
                                    foreground='red' if s['estop'] else 'green')
            now = self.node.get_clock().now().nanoseconds / 1e9
            age = now - s['stamp'] if s['stamp'] > 0 else -1
            self.age_label.config(text=f"joint_states: {age:.2f}s")
            tw = self.node.get_target_wcs()
            if tw:
                self.target_wcs_label.config(
                    text=f"target WCS: ({tw[0]:.3f},{tw[1]:.3f},{tw[2]:.3f})")
            else:
                self.target_wcs_label.config(text="target WCS: —")
            for i in range(6):
                if s['has_target']:
                    self.target_labels[i].config(text=f"{math.degrees(s['target'][i]):8.2f}")
                else:
                    self.target_labels[i].config(text="   ---.--")
                self.actual_labels[i].config(text=f"{math.degrees(s['actual'][i]):8.2f}")
                if s['has_target']:
                    d = math.degrees(s['target'][i] - s['actual'][i])
                    self.diff_labels[i].config(text=f"{d:+8.2f}",
                        foreground='green' if abs(d) < 1.0 else 'orange')
                else:
                    self.diff_labels[i].config(text="   ---.--")
            wcs = self.node.compute_tool_wcs()
            if wcs:
                self.tool_label.config(
                    text=f"X: {wcs[0]:7.3f}   Y: {wcs[1]:7.3f}   Z: {wcs[2]:7.3f}  (m)")
        except Exception as e:
            self.log(f"EXC refresh: {type(e).__name__}: {e}")
        self.root.after(GUI_REFRESH_MS, self.refresh)


def main():
    try:
        root = tk.Tk()
    except tk.TclError as e:
        print(f"ERROR: {e}", file=sys.stderr); sys.exit(1)
    rclpy.init()
    node = WcsGuiNode()
    Gui(root, node)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()