import struct
import time
import threading
import zlib
import numpy as np
import snap7
import omni.usd
import omni.physx
import omni.timeline
import omni.physics.tensors as tensors
from pxr import UsdPhysics

# ── 1. تنظيف أي تشغيل سابق ──────────────────────────────────────────
if "physx_sub" in globals() and physx_sub is not None:
    try:
        physx_sub.unsubscribe()
    except Exception:
        pass
    physx_sub = None
if "timeline_sub" in globals() and timeline_sub is not None:
    timeline_sub = None
if "plant_bridge" in globals() and plant_bridge is not None:
    plant_bridge.stop()

# ── 2. إعدادات البروتوكول ────────────────────────────────────────────
PLC_IP = "192.168.99.56"
PLC_RACK = 0
PLC_SLOT = 1
DB_FROM_PLC = 37        # 64 bytes
DB_TO_PLC = 39          # 128 bytes
NUM_JOINTS = 6
CYCLE_PERIOD_S = 0.020
PLC_HB_TIMEOUT_S = 0.150

MAX_VEL_DEG_S = np.array([115.0, 115.0, 120.0, 179.0, 172.0, 219.0], dtype=np.float64)
MAX_STEP_DEG = MAX_VEL_DEG_S * CYCLE_PERIOD_S


class IsaacPlantBridge:
    def __init__(self, plc_ip=PLC_IP):
        self.plc_ip = plc_ip
        self.client = snap7.Client()
        self.connected = False

        self._lock = threading.Lock()
        self._running = False
        self._thread = None

        self.actual_pos_deg = np.zeros(NUM_JOINTS, dtype=np.float64)
        self.actual_vel_deg_s = np.zeros(NUM_JOINTS, dtype=np.float64)
        self.plant_ready = True
        self.plant_fault = False
        self.plant_homed = True
        self.plant_hw_estop = False

        self.approved_pos_deg = np.zeros(NUM_JOINTS, dtype=np.float64)
        self.plc_axis_enable = False
        self.plc_estop = False
        self.plc_alive = False

        self._plant_hb = 0
        self._last_plc_hb = None
        self._last_plc_hb_time = 0.0
        self._initialized_pos = False

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._io_loop, daemon=True)
        self._thread.start()
        print(f"[IsaacPlant] Bridge thread started targeting PLC {self.plc_ip}")

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self.connected:
            try:
                self.client.disconnect()
            except Exception:
                pass
        print("[IsaacPlant] Bridge stopped safely.")

    def _io_loop(self):
        backoff_s = 0.1
        while self._running:
            t_start = time.monotonic()

            if not self.connected:
                try:
                    self.client.connect(self.plc_ip, PLC_RACK, PLC_SLOT)
                    self.connected = self.client.get_connected()
                    if self.connected:
                        print("[IsaacPlant] Connected to PLC!")
                        backoff_s = 0.1
                except Exception:
                    self.connected = False

                if not self.connected:
                    with self._lock:
                        self.plc_alive = False
                        self.plc_axis_enable = False
                    time.sleep(backoff_s)
                    backoff_s = min(backoff_s * 2.0, 2.0)
                    continue

            try:
                self._exchange_data(t_start)
            except Exception as e:
                print(f"[IsaacPlant] I/O Error: {e}")
                self.connected = False
                with self._lock:
                    self.plc_alive = False
                    self.plc_axis_enable = False
                try:
                    self.client.disconnect()
                except Exception:
                    pass

            elapsed = time.monotonic() - t_start
            sleep_time = CYCLE_PERIOD_S - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _exchange_data(self, now: float):
        raw_rx = bytes(self.client.db_read(DB_FROM_PLC, 0, 64))
        j0, j1, j2, j3, j4, j5, plc_hb, plc_flags, plc_st, pad_w, rx_crc = struct.unpack(">6diBBHI", raw_rx[:60])
        rx_pos = np.array([j0, j1, j2, j3, j4, j5], dtype=np.float64)

        calc_crc = zlib.crc32(raw_rx[:56]) & 0xFFFFFFFF
        if rx_crc == calc_crc:
            if self._last_plc_hb is None or plc_hb != self._last_plc_hb:
                self._last_plc_hb = plc_hb
                self._last_plc_hb_time = now

            hb_ok = (now - self._last_plc_hb_time) < PLC_HB_TIMEOUT_S
            axis_en = bool(plc_flags & 1)
            estop = bool(plc_flags & 2)

            with self._lock:
                self.plc_alive = hb_ok
                self.plc_axis_enable = axis_en and hb_ok
                self.plc_estop = estop

                if self.plc_axis_enable and not estop:
                    delta = rx_pos - self.actual_pos_deg
                    clamped_delta = np.clip(delta, -MAX_STEP_DEG, MAX_STEP_DEG)
                    self.approved_pos_deg = self.actual_pos_deg + clamped_delta

        self._plant_hb = (self._plant_hb + 1) & 0x7FFFFFFF
        with self._lock:
            act_pos = self.actual_pos_deg.flatten().tolist()[:NUM_JOINTS]
            act_vel = self.actual_vel_deg_s.flatten().tolist()[:NUM_JOINTS]
            p_flags = 0
            if self.plant_ready:    p_flags |= 1
            if self.plant_fault:    p_flags |= 2
            if self.plant_homed:    p_flags |= 4
            if self.plant_hw_estop: p_flags |= 8

        payload_104 = struct.pack(">6d6diB3x", *act_pos, *act_vel, self._plant_hb, p_flags)
        tx_crc = zlib.crc32(payload_104) & 0xFFFFFFFF
        tx_buf = payload_104 + struct.pack(">I20x", tx_crc)
        self.client.db_write(DB_TO_PLC, 0, bytearray(tx_buf))


# ── 3. اكتشاف الروبوت وتشغيل الحلقة الفيزيائية المباشرة ───────────────
def find_robot_root():
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            path = str(prim.GetPath())
            print(f"[IsaacPlant] Found ArticulationRoot at: {path}")
            return path
    raise RuntimeError("لم يتم العثور على روبوت في المشهد!")

robot_path = find_robot_root()
plant_bridge = IsaacPlantBridge(plc_ip=PLC_IP)
plant_bridge.start()

sim_view = None
art_view = None
robot_indices = np.array([0], dtype=np.uint32)

def on_physics_step(dt):
    global sim_view, art_view
    if art_view is None:
        sim_view = tensors.create_simulation_view("numpy")
        if sim_view is None:
            return
        art_view = sim_view.create_articulation_view(robot_path)
        print("[IsaacPlant] Physics ArticulationView initialized successfully!")

    pos_rad = art_view.get_dof_positions()[0, :NUM_JOINTS]
    vel_rad = art_view.get_dof_velocities()[0, :NUM_JOINTS]

    with plant_bridge._lock:
        plant_bridge.actual_pos_deg = np.rad2deg(pos_rad)
        plant_bridge.actual_vel_deg_s = np.rad2deg(vel_rad)

        if not plant_bridge._initialized_pos:
            plant_bridge.approved_pos_deg = plant_bridge.actual_pos_deg.copy()
            plant_bridge._initialized_pos = True

        if plant_bridge.plc_alive and plant_bridge.plc_axis_enable and not plant_bridge.plc_estop:
            target_deg = plant_bridge.approved_pos_deg.copy()
        else:
            target_deg = plant_bridge.actual_pos_deg.copy()

    full_targets = art_view.get_dof_position_targets()
    full_targets[0, :NUM_JOINTS] = np.deg2rad(target_deg)
    art_view.set_dof_position_targets(full_targets, robot_indices)

def on_timeline_event(e):
    global sim_view, art_view
    if e.type == int(omni.timeline.TimelineEventType.STOP):
        sim_view = None
        art_view = None
        plant_bridge._initialized_pos = False

timeline_sub = omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop(on_timeline_event)
physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)
print("[IsaacPlant] Ready! Press PLAY in Isaac Sim.")
