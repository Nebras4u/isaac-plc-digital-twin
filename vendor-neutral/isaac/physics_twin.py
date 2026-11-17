# physics_twin.py
# v14.6 — detach-safe: exclude joint from articulation + settle window + robust reinit

import asyncio
import math
import sys
import time

import numpy as np
import yaml

import omni.kit.app
import omni.usd
from omni.timeline import get_timeline_interface

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from isaacsim.core.prims import Articulation
try:
    from isaacsim.core.prims import RigidPrim
except ImportError:
    from omni.isaac.core.prims import RigidPrim

from pxr import UsdPhysics, UsdGeom, Sdf, Gf, Usd
try:
    from pxr import PhysxSchema
except ImportError:
    PhysxSchema = None

import os as _os
_SCRIPT_DIR = _os.path.dirname(_os.path.abspath(__file__))
for _p in (_SCRIPT_DIR, '/workspace'):
    if _os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
del _os, _SCRIPT_DIR, _p

from joint_mapper import JointMapper

DEFAULT_ROBOT_CFG       = "/workspace/robot.yaml"
DEFAULT_PUBLISH_HZ      = 120.0
DEFAULT_TIMEOUT_S       = 0.5
TAG_VALID_S             = 0.5
MAX_MSG_PER_FRAME       = 8
SUCTION_CHECK_PERIOD_S  = 0.1
RIGID_RETRY_S           = 1.0
TRACK_TIMES             = (0.0, 0.1, 0.25, 0.5, 1.0, 2.0)
SUCTION_JOINT_PREFIX    = "SuctionJoint"
SUCTION_JOINT_ROOT      = "/World"
DETACH_SETTLE_S         = 0.15          # NEW: hold current pose after detach
REINIT_COOLDOWN_S       = 1.0           # NEW: don't spam reinit


def load_robot_cfg(path: str) -> dict:
    with open(path) as f:
        loaded = yaml.safe_load(f)
    if loaded is None: raise ValueError("robot.yaml empty")
    if "robot" not in loaded: raise KeyError("missing robot")
    return loaded["robot"]

def inspect_limits(joint_paths: dict) -> dict:
    stage = omni.usd.get_context().get_stage()
    limits = {}
    for name, path in joint_paths.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid(): continue
        joint = UsdPhysics.RevoluteJoint(prim)
        if joint:
            lo = joint.GetLowerLimitAttr().Get()
            hi = joint.GetUpperLimitAttr().Get()
            if lo is not None and hi is not None:
                limits[name] = (float(lo), float(hi))
    return limits

def configure_drives(drives: dict, joint_paths: dict):
    stage = omni.usd.get_context().get_stage()
    for joint_name, params in drives.items():
        path = joint_paths[joint_name]
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid(): continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive: continue
        drive.GetStiffnessAttr().Set(float(params["stiffness"]))
        drive.GetDampingAttr().Set(float(params["damping"]))
        drive.GetMaxForceAttr().Set(float(params["max_force"]))

class PhysicsTwin(Node):
    def __init__(self, robot, mapper, cfg, params, joint_limits=None):
        super().__init__("physics_twin")
        self.robot  = robot
        self.mapper = mapper
        self.cfg    = cfg

        self.publish_hz = max(1.0, float(params["publish_hz"]))
        self.timeout_s  = max(0.0, float(params["setpoint_timeout_s"]))
        self._joint_limits   = dict(joint_limits or {})
        self._enforce_limits = bool(params.get("enforce_joint_limits", False))
        self._known_joints   = set(mapper.dof_order)
        n = mapper.n

        self._latest_setpoint   = None
        self._last_sp_t         = 0.0
        self._tag               = ""
        self._tag_t             = 0.0
        self._applied_tag       = ""
        self._applied_tag_t     = 0.0
        self._fresh_this_frame  = False
        self._estop_active      = False
        self._needs_reinit      = False
        self._last_reinit_t     = 0.0
        self._last_pub_t        = 0.0
        self._seed_done         = False
        self._estop_hold_isaac  = None

        # NEW: post-detach settle window
        self._detach_settle_until = 0.0

        self._target_yaml      = np.zeros(n, dtype=np.float32)
        self._target_isaac     = np.zeros(n, dtype=np.float32)
        self._last_valid_yaml  = np.zeros(n, dtype=np.float32)
        self._last_valid_isaac = np.zeros(n, dtype=np.float32)
        self._current_isaac    = np.zeros(n, dtype=np.float32)
        self._velocity_isaac   = np.zeros(n, dtype=np.float32)
        self._joint_indices    = np.arange(n, dtype=np.int32)
        self._target_isaac_2d  = self._target_isaac.reshape(1, -1)

        suction_cfg = cfg.get("suction", {}) or {}
        self._suction_enabled = bool(suction_cfg.get("enabled", False))
        robot_scope = cfg.get("isaac_scope", "")

        self._min_engaged_cups = int(suction_cfg.get("min_engaged_cups", 3))
        self._contact_tol_m    = float(suction_cfg.get("contact_tol_m", 0.004))
        self._max_pen_m        = float(suction_cfg.get("max_penetration_m", 0.06))
        self._plate_iters      = int(suction_cfg.get("plate_solver_iters", 32))
        self._contact_tol_u    = self._contact_tol_m
        self._max_pen_u        = self._max_pen_m
        self._snap_to_face     = bool(suction_cfg.get("snap_to_face", False))
        self._auto_tips        = bool(suction_cfg.get("auto_cup_tips", True))
        self._last_diag_t      = 0.0
        self._cup_pts          = []
        self._ee_at_on         = None
        self._filter_collision = bool(suction_cfg.get("filter_plate_gripper_collision", False))
        self._mpu              = 1.0
        self._track            = None
        self._plate_at_on      = {}

        self._rigid_prim_cache = {}
        self._rigid_fail_t = {}
        self._joint_paths = []

        if self._suction_enabled:
            ee_tpl = suction_cfg.get("ee_prim", "{scope}/link_6")
            self._ee_prim_path = ee_tpl.replace("{scope}", robot_scope)
            self._payload_candidates = list(suction_cfg.get("payload_candidates", []))
            self._cups_local = [Gf.Vec3d(*c) for c in suction_cfg.get("cups_local", [])]

            stage = omni.usd.get_context().get_stage()
            mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
            self._mpu = mpu

            self._cup_pts = [np.array([[c[0], c[1], c[2]]]) for c in self._cups_local]
            if self._auto_tips:
                pts = self._collect_cup_points(stage)
                if pts is not None:
                    self._cup_pts = pts

            if self._filter_collision:
                for pth in self._payload_candidates:
                    pprim = stage.GetPrimAtPath(pth)
                    if pprim.IsValid():
                        fp = UsdPhysics.FilteredPairsAPI.Apply(pprim)
                        fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(self._ee_prim_path))
                        print(f"[physics_twin] collision filtered: {pth} <-> {self._ee_prim_path}")
            self._contact_tol_u = self._contact_tol_m / mpu
            self._max_pen_u     = self._max_pen_m / mpu

            self._remove_suction_joints(stage)
            for p in self._payload_candidates:
                prim = stage.GetPrimAtPath(p)
                if prim.IsValid():
                    rb = UsdPhysics.RigidBodyAPI(prim)
                    if rb:
                        rb.CreateKinematicEnabledAttr().Set(False)
                        print(f"[physics_twin] Cleaned up {p} on startup.")
        else:
            self._ee_prim_path = ""
            self._payload_candidates = []
            self._cups_local = []

        self._suction_on = False
        self._attached_payload_path = None
        self._last_suction_check_t = 0.0
        self._last_logged_count = {}

        self._seed_from_current()

        sp_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/servo_setpoints", self.on_setpoint, sp_qos)
        estop_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/plc/emergency_stop", self.on_estop, estop_qos)
        if self._suction_enabled:
            suction_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, "/plc/suction_cmd", self.on_suction_cmd, suction_qos)

        js_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.js_pub = self.create_publisher(JointState, "/joint_states", js_qos)

        self.get_logger().info(
            f"PhysicsTwin v14.6 ready. cups={len(self._cups_local)} "
            f"min_cups={self._min_engaged_cups} gap_tol={self._contact_tol_m}m "
            f"max_pen={self._max_pen_m}m")

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _validate_setpoint(self, msg: JointState):
        if len(msg.name) != len(msg.position): return None
        sp = {}
        for name, pos in zip(msg.name, msg.position):
            if name not in self._known_joints: continue
            try: f = float(pos)
            except: continue
            if not math.isfinite(f): continue
            if self._joint_limits:
                lo, hi = self._joint_limits.get(name, (-math.inf, math.inf))
                if f < lo or f > hi:
                    if self._enforce_limits: continue
            sp[name] = f
        return sp if sp else None

    def on_setpoint(self, msg: JointState):
        if self._estop_active: return
        sp = self._validate_setpoint(msg)
        if sp is None: return
        self._latest_setpoint = sp
        self._tag = msg.header.frame_id
        self._tag_t = time.monotonic()
        self._last_sp_t = self._tag_t

    def on_estop(self, msg: Bool):
        was = self._estop_active
        self._estop_active = bool(msg.data)
        if self._estop_active and not was:
            self._latest_setpoint = None
            self._estop_hold_isaac = None

    def on_suction_cmd(self, msg: Bool):
        if not self._suction_enabled: return
        new_state = bool(msg.data)
        if new_state == self._suction_on: return
        self._suction_on = new_state
        if new_state:
            print("[physics_twin] SUCTION ON")
            self._last_logged_count.clear()
            self._record_plate_positions()
        else:
            print("[physics_twin] SUCTION OFF")
            self._detach_payload()

    # ------------------------------------------------------------------
    # Suction helpers
    # ------------------------------------------------------------------
    def _get_rigid_prim(self, path):
        rp = self._rigid_prim_cache.get(path)
        if rp is not None:
            return rp
        t_fail = self._rigid_fail_t.get(path)
        if t_fail is not None and (time.monotonic() - t_fail) < RIGID_RETRY_S:
            return None
        try:
            rp = RigidPrim(prim_paths_expr=path)
            rp.initialize()
            self._rigid_prim_cache[path] = rp
            self._rigid_fail_t.pop(path, None)
            return rp
        except Exception:
            self._rigid_fail_t[path] = time.monotonic()
            return None

    def _get_prim_world_transform(self, prim_path):
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                return None
            usd_m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            rp = self._get_rigid_prim(prim_path)
            if rp is None:
                return usd_m
            try:
                pos, quat = rp.get_world_poses()
                p, q = pos[0], quat[0]
                sc = [usd_m.GetRow3(k).GetLength() for k in range(3)]
                S = Gf.Matrix4d(1.0); S.SetScale(Gf.Vec3d(*sc))
                R = Gf.Matrix4d(1.0)
                R.SetRotate(Gf.Quatd(float(q[0]), Gf.Vec3d(float(q[1]), float(q[2]), float(q[3]))))
                T = Gf.Matrix4d(1.0)
                T.SetTranslate(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
                return S * R * T
            except Exception:
                return usd_m
        except Exception:
            return None

    def _remove_suction_joints(self, stage):
        root = stage.GetPrimAtPath(SUCTION_JOINT_ROOT)
        if root.IsValid():
            for child in list(root.GetChildren()):
                if child.GetName().startswith(SUCTION_JOINT_PREFIX):
                    stage.RemovePrim(child.GetPath())
        self._joint_paths = []

    def _plate_geometry(self, payload_path):
        plate_w = self._get_prim_world_transform(payload_path)
        if plate_w is None:
            return None
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(payload_path)
        if not prim.IsValid():
            return None
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        rng = cache.ComputeUntransformedBound(prim).ComputeAlignedRange()
        if rng.IsEmpty():
            return None
        lo, hi = rng.GetMin(), rng.GetMax()
        scale  = [plate_w.GetRow3(k).GetLength() for k in range(3)]
        half   = [(hi[k] - lo[k]) * 0.5 for k in range(3)]
        center = [(hi[k] + lo[k]) * 0.5 for k in range(3)]
        k_thin = int(np.argmin([half[k] * scale[k] for k in range(3)]))
        return {"plate_w": plate_w, "lo": lo, "hi": hi, "scale": scale,
                "half": half, "center": center, "k": k_thin}

    def _collect_cup_points(self, stage):
        try:
            ee_prim = stage.GetPrimAtPath(self._ee_prim_path)
            if not ee_prim.IsValid() or not self._cups_local:
                return None
            tc = Usd.TimeCode.Default()
            ee_inv = UsdGeom.Xformable(ee_prim).ComputeLocalToWorldTransform(tc).GetInverse()

            cup_prims = []
            for pr in Usd.PrimRange(ee_prim, Usd.TraverseInstanceProxies()):
                if pr.GetName().startswith("Suction_Cup"):
                    cup_prims.append(pr)
            cup_prims.sort(key=lambda x: x.GetName())
            if len(cup_prims) != len(self._cups_local):
                print(f"[physics_twin] found {len(cup_prims)} Suction_Cup prims, "
                      f"yaml has {len(self._cups_local)}; using yaml points")
                return None

            out = []
            for pr in cup_prims:
                chunks = []
                for m in Usd.PrimRange(pr, Usd.TraverseInstanceProxies()):
                    if not m.IsA(UsdGeom.Mesh):
                        continue
                    pts = UsdGeom.Mesh(m).GetPointsAttr().Get()
                    if pts is None or len(pts) == 0:
                        continue
                    P = np.array(pts, dtype=np.float64).reshape(-1, 3)
                    M = UsdGeom.Xformable(m).ComputeLocalToWorldTransform(tc) * ee_inv
                    Mn = np.array([[M[r][c] for c in range(4)] for r in range(4)])
                    chunks.append(P @ Mn[:3, :3] + Mn[3, :3])
                if not chunks:
                    print(f"[physics_twin] no mesh points under {pr.GetPath()}; using yaml points")
                    return None
                Q = np.vstack(chunks)
                if len(Q) > 20000:
                    Q = Q[::len(Q) // 20000 + 1]
                out.append(Q)

            allp = np.vstack(out)
            lo_, hi_ = allp.min(axis=0), allp.max(axis=0)
            Y = np.array([[c[0], c[1], c[2]] for c in self._cups_local])
            print(f"[physics_twin] cup meshes: {len(out)} cups, {len(allp)} pts | link_6 extent "
                  f"x[{lo_[0]:.3f},{hi_[0]:.3f}] y[{lo_[1]:.3f},{hi_[1]:.3f}] z[{lo_[2]:.3f},{hi_[2]:.3f}] "
                  f"| yaml cup pts x[{Y[:,0].min():.3f},{Y[:,0].max():.3f}]")
            return out
        except Exception as e:
            print(f"[physics_twin] cup mesh collection failed ({e}); using yaml points")
            return None

    def _cup_contacts(self, payload_path):
        g = self._plate_geometry(payload_path)
        ee_w = self._get_prim_world_transform(self._ee_prim_path)
        if g is None or ee_w is None or not self._cup_pts:
            return []

        M = ee_w * g["plate_w"].GetInverse()
        Mn = np.array([[M[r][c] for c in range(4)] for r in range(4)])
        R, T = Mn[:3, :3], Mn[3, :3]
        k, sc = g["k"], g["scale"]
        center, half, lo, hi = g["center"], g["half"], g["lo"], g["hi"]
        tol_u = self._contact_tol_u
        rim_band = 0.005 / self._mpu

        loc = [pts @ R + T for pts in self._cup_pts]
        side = float(np.mean([L[:, k].mean() for L in loc])) - center[k]
        s = 1.0 if side >= 0.0 else -1.0

        out = []
        for i, L in enumerate(loc):
            hv = (s * (L[:, k] - center[k]) - half[k]) * sc[k]
            h = float(hv.min())
            rim = hv <= h + rim_band
            q = L[rim].mean(axis=0)
            ok = True
            for a in range(3):
                if a == k:
                    continue
                t = tol_u / sc[a] if sc[a] > 1e-9 else tol_u
                if not (lo[a] - t <= q[a] <= hi[a] + t):
                    ok = False
                    break
            if not ok:
                continue
            rl = self._cup_pts[i][rim].mean(axis=0)
            rim_local = Gf.Vec3d(float(rl[0]), float(rl[1]), float(rl[2]))
            out.append((i, ee_w.Transform(rim_local), h, rim_local))
        return out

    def _engaged_cups(self, payload_path):
        return [c for c in self._cup_contacts(payload_path)
                if -self._max_pen_u <= c[2] <= self._contact_tol_u]

    def _motion_str(self, payload_path):
        try:
            k = self._mpu * 1000.0
            pl = self._get_prim_world_transform(payload_path)
            ee = self._get_prim_world_transform(self._ee_prim_path)
            t_pl = self._plate_at_on.get(payload_path)
            if pl is None or ee is None or t_pl is None or self._ee_at_on is None:
                return ""
            dp = (pl.ExtractTranslation() - t_pl) * k
            de = (ee.ExtractTranslation() - self._ee_at_on) * k
            return (f" | since ON (mm): plate=({dp[0]:.0f},{dp[1]:.0f},{dp[2]:.0f}) "
                    f"gripper=({de[0]:.0f},{de[1]:.0f},{de[2]:.0f})")
        except Exception:
            return ""

    def _handle_suction(self):
        if not self._suction_enabled or not self._suction_on or self._attached_payload_path is not None:
            return
        now = time.monotonic()
        if (now - self._last_suction_check_t) < SUCTION_CHECK_PERIOD_S: return
        self._last_suction_check_t = now

        total = len(self._cups_local)
        for payload_path in self._payload_candidates:
            contacts = self._cup_contacts(payload_path)
            engaged = [c for c in contacts if -self._max_pen_u <= c[2] <= self._contact_tol_u]
            if contacts and (now - self._last_diag_t) >= 0.5:
                self._last_diag_t = now
                hs = [c[2] * self._mpu * 1000.0 for c in contacts]
                print(f"[suction-diag] {payload_path}: {len(contacts)} cups over plate, "
                      f"h(mm) min/mean/max = {min(hs):.1f}/{sum(hs)/len(hs):.1f}/{max(hs):.1f} "
                      f"(+ gap, - penetration)" + self._motion_str(payload_path))

            if self._last_logged_count.get(payload_path) != len(engaged):
                self._last_logged_count[payload_path] = len(engaged)
                print(f"[suction] {payload_path}: {len(engaged)}/{total} cups on plate "
                      f"| need {self._min_engaged_cups}")

            if len(engaged) >= self._min_engaged_cups:
                self._attach_payload(payload_path, engaged)
                return

    def _attach_payload(self, payload_path, engaged):
        try:
            stage = omni.usd.get_context().get_stage()
            self._remove_suction_joints(stage)

            ee_w = self._get_prim_world_transform(self._ee_prim_path)
            g = self._plate_geometry(payload_path)
            if ee_w is None or g is None:
                return
            plate_w = g["plate_w"]
            k = g["k"]

            t_on = self._plate_at_on.get(payload_path)
            if t_on is not None:
                d = (plate_w.ExtractTranslation() - t_on) * self._mpu * 1000.0
                print(f"[physics_twin] plate moved since SUCTION ON (world, mm): "
                      f"({d[0]:.1f}, {d[1]:.1f}, {d[2]:.1f})")

            rp = self._get_rigid_prim(payload_path)
            if rp:
                rp.set_linear_velocities(np.zeros((1, 3), dtype=np.float32))
                rp.set_angular_velocities(np.zeros((1, 3), dtype=np.float32))

            if PhysxSchema is not None:
                try:
                    api = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(payload_path))
                    api.CreateSolverPositionIterationCountAttr().Set(self._plate_iters)
                    api.CreateSolverVelocityIterationCountAttr().Set(4)
                except Exception:
                    pass

            n = len(engaged)
            c_local = Gf.Vec3d(0.0, 0.0, 0.0)
            for c in engaged:
                c_local += c[3]
            c_local = c_local / n
            c_world = ee_w.Transform(c_local)
            mean_h = sum(c[2] for c in engaged) / n

            q = (ee_w.RemoveScaleShear() * plate_w.RemoveScaleShear().GetInverse()).ExtractRotationQuat()
            qi = q.GetImaginary()
            qr = Gf.Quatf(float(q.GetReal()), float(qi[0]), float(qi[1]), float(qi[2]))

            p1 = plate_w.GetInverse().Transform(c_world)

            s = 1.0 if (p1[k] - g["center"][k]) >= 0.0 else -1.0
            if self._snap_to_face:
                face = g["center"][k] + s * g["half"][k]
                p1 = Gf.Vec3d(p1[0], p1[1], p1[2])
                p1[k] = face

            sc_p = [round(plate_w.GetRow3(j).GetLength(), 4) for j in range(3)]
            sc_e = [round(ee_w.GetRow3(j).GetLength(), 4) for j in range(3)]
            try:
                mass = UsdPhysics.MassAPI(stage.GetPrimAtPath(payload_path)).GetMassAttr().Get()
            except Exception:
                mass = None
            print(f"[physics_twin] plate_scale={sc_p} ee_scale={sc_e} plate_mass={mass} "
                  f"thin_axis={'xyz'[k]} face_sign={s:+.0f} mean_h={mean_h*self._mpu*1000:.1f}mm snap={self._snap_to_face} "
                  f"(neg = penetration) p1={p1}")

            path = f"{SUCTION_JOINT_ROOT}/{SUCTION_JOINT_PREFIX}"
            j = UsdPhysics.FixedJoint.Define(stage, path)
            j.CreateBody0Rel().SetTargets([Sdf.Path(self._ee_prim_path)])
            j.CreateBody1Rel().SetTargets([Sdf.Path(payload_path)])
            j.CreateLocalPos0Attr().Set(Gf.Vec3f(float(c_local[0]), float(c_local[1]), float(c_local[2])))
            j.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            j.CreateLocalPos1Attr().Set(Gf.Vec3f(float(p1[0]), float(p1[1]), float(p1[2])))
            j.CreateLocalRot1Attr().Set(qr)
            j.CreateCollisionEnabledAttr().Set(False)

            # NEW: keep the joint OUT of the articulation tree.
            # This prevents PhysX from corrupting articulation state when we
            # delete the joint on detach (which is what froze the arm before).
            try:
                j.CreateExcludeFromArticulationAttr().Set(True)
            except Exception:
                pass

            self._joint_paths.append(path)

            self._attached_payload_path = payload_path
            print(f"[physics_twin] ATTACHED {payload_path} with {n} cups: {[c[0] for c in engaged]}")
            self._track_start(payload_path)
        except Exception as e:
            print(f"[physics_twin] Attach error: {e}")

    def _record_plate_positions(self):
        self._plate_at_on = {}
        em = self._get_prim_world_transform(self._ee_prim_path)
        self._ee_at_on = em.ExtractTranslation() if em is not None else None
        for pth in self._payload_candidates:
            m = self._get_prim_world_transform(pth)
            if m is not None:
                self._plate_at_on[pth] = m.ExtractTranslation()

    def _track_start(self, payload_path):
        self._track = {"path": payload_path, "t0": time.monotonic(), "k": 0,
                       "ref_w": None, "ref_rel": None}

    def _track_step(self):
        tr = self._track
        if tr is None:
            return
        if self._attached_payload_path != tr["path"] or tr["k"] >= len(TRACK_TIMES):
            self._track = None
            return
        dt = time.monotonic() - tr["t0"]
        if dt < TRACK_TIMES[tr["k"]]:
            return
        ee_w = self._get_prim_world_transform(self._ee_prim_path)
        pl_w = self._get_prim_world_transform(tr["path"])
        if ee_w is None or pl_w is None:
            return
        pw = pl_w.ExtractTranslation()
        rel = ee_w.RemoveScaleShear().GetInverse().Transform(pw)
        if tr["ref_w"] is None:
            tr["ref_w"], tr["ref_rel"] = pw, rel
        k = self._mpu * 1000.0
        dw = (pw - tr["ref_w"]) * k
        dr = (rel - tr["ref_rel"]) * k
        print(f"[track] t={dt:.2f}s  d_world(mm)=({dw[0]:.1f},{dw[1]:.1f},{dw[2]:.1f})  "
              f"d_in_link6(mm)=({dr[0]:.1f},{dr[1]:.1f},{dr[2]:.1f})")
        tr["k"] += 1

    def _detach_payload(self):
        if self._attached_payload_path is None: return
        try:
            stage = omni.usd.get_context().get_stage()
            self._remove_suction_joints(stage)
            print(f"[physics_twin] DETACHED {self._attached_payload_path}")
        except Exception as e:
            print(f"[physics_twin] Detach error: {e}")
        finally:
            self._attached_payload_path = None
            self._last_logged_count.clear()
            self._track = None
            # NEW: give the PD controller time to rebalance without the load.
            # During this window _build_target() will hold the current pose
            # instead of chasing the last setpoint, avoiding an over-correction
            # that could destabilize the articulation.
            self._detach_settle_until = time.monotonic() + DETACH_SETTLE_S

    def shutdown(self):
        try:
            if self._suction_enabled:
                stage = omni.usd.get_context().get_stage()
                self._remove_suction_joints(stage)
                self._attached_payload_path = None
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Joint control
    # ------------------------------------------------------------------
    def _seed_from_current(self) -> bool:
        try: cur = self.robot.get_joint_positions()
        except: return False
        if cur is None: return False
        self._last_valid_isaac[:] = cur[0]
        self._last_valid_yaml[:]  = cur[0][self.mapper.yaml_to_isaac]
        self._seed_done = True
        return True

    def _refresh_current(self) -> bool:
        try: cur = self.robot.get_joint_positions()
        except:
            self._needs_reinit = True
            return False
        if cur is None: return False
        self._current_isaac[:] = cur[0]
        return True

    def _build_target(self) -> bool:
        self._fresh_this_frame = False

        # NEW: post-detach settle window — hold current pose, ignore setpoints.
        # This prevents the PD controller from violently over-correcting right
        # after the payload mass disappears.
        if time.monotonic() < self._detach_settle_until:
            if not self._refresh_current():
                return False
            self._target_isaac[:]      = self._current_isaac
            self._last_valid_isaac[:]  = self._current_isaac
            self._last_valid_yaml[:]   = self._current_isaac[self.mapper.yaml_to_isaac]
            return True

        if self._estop_active:
            if self._estop_hold_isaac is None:
                if not self._refresh_current(): return False
                self._estop_hold_isaac = self._current_isaac.copy()
                self._last_valid_isaac[:] = self._estop_hold_isaac
                self._last_valid_yaml[:]  = self._estop_hold_isaac[self.mapper.yaml_to_isaac]
                self._seed_done = True
            self._target_isaac[:] = self._estop_hold_isaac
            return True
        if self._latest_setpoint is None:
            if not self._seed_done:
                if not self._refresh_current(): return False
                self._last_valid_isaac[:] = self._current_isaac
                self._last_valid_yaml[:]  = self._current_isaac[self.mapper.yaml_to_isaac]
                self._seed_done = True
            self._target_isaac[:] = self._last_valid_isaac
            return True
        if time.monotonic() - self._last_sp_t > self.timeout_s:
            self._target_isaac[:] = self._last_valid_isaac
            return True
        sp = self._latest_setpoint
        for j, jname in enumerate(self.mapper.dof_order):
            v = sp.get(jname)
            self._target_yaml[j] = v if v is not None else self._last_valid_yaml[j]
        self._target_isaac[self.mapper.yaml_to_isaac] = self._target_yaml
        self._last_valid_yaml[:]  = self._target_yaml
        self._last_valid_isaac[:] = self._target_isaac
        self._fresh_this_frame    = True
        return True

    def step(self):
        try:
            if get_timeline_interface().is_stopped(): return
        except: pass
        if not self._build_target(): return
        try:
            self.robot.set_joint_position_targets(positions=self._target_isaac_2d, joint_indices=self._joint_indices)
        except:
            self._needs_reinit = True
            return
        try:
            self._handle_suction()
            self._track_step()
        except Exception as e:
            print(f"[physics_twin] step error: {e}")
        if self._fresh_this_frame:
            self._applied_tag   = self._tag
            self._applied_tag_t = self._tag_t

    def publish_states(self):
        try:
            if get_timeline_interface().is_stopped(): return
        except: pass
        now = time.monotonic()
        if (now - self._last_pub_t) < (1.0 / self.publish_hz): return
        self._last_pub_t = now
        try:
            pos = self.robot.get_joint_positions()
            vel = self.robot.get_joint_velocities()
        except:
            self._needs_reinit = True
            return
        if pos is None: return
        if vel is not None: self._velocity_isaac[:] = vel[0]
        else: self._velocity_isaac[:] = 0.0
        self._current_isaac[:] = pos[0]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.mapper.dof_order
        msg.position = self._current_isaac[self.mapper.yaml_to_isaac].tolist()
        msg.velocity = self._velocity_isaac[self.mapper.yaml_to_isaac].tolist()
        msg.effort = []
        tag = self._applied_tag
        if tag and not self._estop_active and (now - self._applied_tag_t) <= TAG_VALID_S:
            msg.header.frame_id = f"{tag};{time.monotonic_ns()}"
        else:
            msg.header.frame_id = ""
            self._applied_tag = ""
        self.js_pub.publish(msg)


def make_robot(robot_path: str) -> Articulation:
    robot = Articulation(robot_path); robot.initialize(); return robot


async def main():
    if not rclpy.ok():
        try: rclpy.init()
        except RuntimeError: pass
    tmp = Node("physics_twin_params")
    tmp.declare_parameter("robot_cfg", DEFAULT_ROBOT_CFG)
    tmp.declare_parameter("publish_hz", DEFAULT_PUBLISH_HZ)
    tmp.declare_parameter("setpoint_timeout_s", DEFAULT_TIMEOUT_S)
    tmp.declare_parameter("warn_cooldown", 5.0)
    tmp.declare_parameter("enforce_joint_limits", False)
    cfg_path = tmp.get_parameter("robot_cfg").value
    params = {
        "publish_hz": tmp.get_parameter("publish_hz").value,
        "setpoint_timeout_s": tmp.get_parameter("setpoint_timeout_s").value,
        "warn_cooldown": tmp.get_parameter("warn_cooldown").value,
        "enforce_joint_limits": bool(tmp.get_parameter("enforce_joint_limits").value),
    }
    tmp.destroy_node()

    cfg = load_robot_cfg(cfg_path)
    robot_path = cfg["isaac_path"]
    robot_scope = cfg["isaac_scope"]
    dof_order = list(cfg["all_joints"])
    joint_paths = {k: v.replace("{scope}", robot_scope) for k, v in cfg["joint_paths"].items()}

    joint_limits = inspect_limits(joint_paths)
    configure_drives(cfg["drives"], joint_paths)

    print("Initializing articulation...")
    robot = make_robot(robot_path)
    mapper = JointMapper(robot.dof_names, dof_order)

    node = PhysicsTwin(robot, mapper, cfg, params, joint_limits=joint_limits)

    try:
        while True:
            # NEW: on reinit, actually try to recover instead of blocking the
            # whole loop with `continue` (which is what froze the arm before).
            if node._needs_reinit:
                now = time.monotonic()
                if (now - node._last_reinit_t) > REINIT_COOLDOWN_S:
                    node._last_reinit_t = now
                    try:
                        robot.initialize()
                        node._needs_reinit = False
                        print("[physics_twin] articulation re-initialized")
                    except Exception as e:
                        print(f"[physics_twin] reinit failed: {e}")
                # Keep draining ROS so callbacks still fire while we recover.
                for _ in range(MAX_MSG_PER_FRAME):
                    try: rclpy.spin_once(node, timeout_sec=0.0)
                    except: break
                await omni.kit.app.get_app().next_update_async()
                continue

            for _ in range(MAX_MSG_PER_FRAME):
                try: rclpy.spin_once(node, timeout_sec=0.0)
                except: break
            node.step()
            node.publish_states()
            await omni.kit.app.get_app().next_update_async()
    finally:
        try: node.shutdown()
        except: pass
        try: node.destroy_node()
        except: pass


_prev = getattr(sys, "_physics_twin_task", None)
if _prev is not None and not _prev.done(): _prev.cancel()
sys._physics_twin_task = asyncio.ensure_future(main())