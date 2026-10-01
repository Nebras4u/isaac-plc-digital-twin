"""
PLC Control GUI — standalone tkinter app for OPC UA.
Reads/Writes variables defined in config.py.

Run:  python plc_gui.py
Deps: pip install asyncua
"""

import asyncio
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import time

from asyncua import Client, ua

from config import (
    PLC_URL, NS, DB_TCP,
    PLC_ARRAYS, PLC_FLAGS, PLC_COMMANDS,
)


# ================== NODE ID HELPER ==================
def node(name):
    return f'ns={NS};s="{DB_TCP}"."{name}"'


# ================== VARIABLE LISTS ==================
NODES_BOOL = [
    "MainRobotLeft_CmdEnable",
    "MainRobotLeft_EnableAll",
    "MainRobotLeft_EnableKinematics",
    "MainRobotLeft_Auto_Mode",
    "MainRobotLeft_Stacking2Left",
    "MainRobotLeft_Go2HomePos",
    "MainRobotLeft_Go2Pos4Maint",
    "MainRobotLeft_CmdAct",
    "MainRobotLeft_CmdAck",
    "MainRobotLeft_MoveEnable",
]

NODES_FAULTS = [
    "MainRobotLeft_Group_Fault",
    "MainRobotLeft_Axis1_Fault",
    "MainRobotLeft_Axis2_Fault",
    "MainRobotLeft_Axis3_Fault",
    "MainRobotLeft_Axis4_Fault",
]

NODES_LREAL = [
    "MainRobotLeft_Vlcty",
]

NODES_ARRAY_CMD = [
    "MainRobotLeft_CmdAng",
    "MainRobotLeft_Pos4Maint",
]

NODES_ARRAY_ACT = [
    "MainRobotLeft_ActAng",
    "MainRobotLeft_ActPos",
]

# --- Reset command (must exist in the DB as Bool, OPC UA writable) ---
NODE_RESET_CMD  = "MainRobotLeft_ResetCmd"
NODE_RESET_DONE = "MainRobotLeft_ResetDone"


# ================== OPC UA HELPERS ==================
async def opc_write_value(node_obj, value, variant_type):
    """Full WriteValue + write_params — same as plc_ros_bridge4.py."""
    wv = ua.WriteValue()
    wv.NodeId = node_obj.nodeid
    wv.AttributeId = ua.AttributeIds.Value
    wv.Value = ua.DataValue(ua.Variant(value, variant_type))
    wv.Value.StatusCode = None
    wv.Value.SourceTimestamp = None
    wv.Value.ServerTimestamp = None

    params = ua.WriteParameters()
    params.NodesToWrite = [wv]
    await node_obj.write_params(params)


# ================== OPC UA WORKER ==================
class PlcWorker:
    """Runs an asyncio loop in a background thread."""

    def __init__(self, url):
        self.url = url
        self.loop = asyncio.new_event_loop()
        self.client = None
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    # ---------- connect / disconnect ----------
    def connect(self):
        async def _c():
            self.client = Client(url=self.url)
            await self.client.connect()
        return self._submit(_c())

    def disconnect(self):
        async def _d():
            if self.client:
                await self.client.disconnect()
                self.client = None
        return self._submit(_d())

    # ---------- bulk read ----------
    def read_all(self, bool_names, fault_names, lreal_names, array_names):
        async def _r():
            bools  = {}
            faults = {}
            lreals = {}
            arrays = {}

            for name in bool_names:
                try:
                    n = self.client.get_node(node(name))
                    bools[name] = bool(await n.read_value())
                except Exception:
                    pass

            for name in fault_names:
                try:
                    n = self.client.get_node(node(name))
                    faults[name] = bool(await n.read_value())
                except Exception:
                    pass

            for name in lreal_names:
                try:
                    n = self.client.get_node(node(name))
                    lreals[name] = float(await n.read_value())
                except Exception:
                    pass

            for name in array_names:
                try:
                    n = self.client.get_node(node(name))
                    arrays[name] = list(await n.read_value())
                except Exception:
                    pass

            return bools, faults, lreals, arrays
        return self._submit(_r())

    # ---------- read single bool ----------
    def read_bool(self, name):
        async def _r():
            n = self.client.get_node(node(name))
            return bool(await n.read_value())
        return self._submit(_r())

    # ---------- write bool ----------
    def write_bool(self, name, value):
        async def _w():
            n = self.client.get_node(node(name))
            await opc_write_value(n, bool(value), ua.VariantType.Boolean)
        return self._submit(_w())

    # ---------- write lreal ----------
    def write_lreal(self, name, value):
        async def _w():
            n = self.client.get_node(node(name))
            await opc_write_value(n, float(value), ua.VariantType.Double)
        return self._submit(_w())

    # ---------- write array[1..4] of LReal ----------
    def write_array4(self, name, values):
        async def _w():
            n = self.client.get_node(node(name))
            await opc_write_value(
                n,
                [float(v) for v in values],
                ua.VariantType.Double,
            )
        return self._submit(_w())

    # ---------- fault reset (801) ----------
    def reset_faults(self, hold_ms=300):
        """Pulse ResetCmd TRUE → wait → FALSE, then wait for ResetDone."""
        async def _r():
            n_cmd = self.client.get_node(node(NODE_RESET_CMD))

            # Pulse TRUE
            await opc_write_value(n_cmd, True, ua.VariantType.Boolean)
            await asyncio.sleep(hold_ms / 1000.0)

            # The PLC normally clears it, but ensure it's FALSE anyway
            try:
                await opc_write_value(n_cmd, False, ua.VariantType.Boolean)
            except Exception:
                pass

            # Wait for ResetDone (max 3 s)
            try:
                n_done = self.client.get_node(node(NODE_RESET_DONE))
                for _ in range(30):
                    if bool(await n_done.read_value()):
                        return True
                    await asyncio.sleep(0.1)
                return False
            except Exception:
                return False
        return self._submit(_r())


# ================== GUI ==================
class PlcGui:
    def __init__(self, root, worker):
        self.root = root
        self.worker = worker
        self.root.title("PLC Control — MainRobotLeft")
        self.root.geometry("820x760")
        self.root.resizable(False, False)

        self.bool_vars   = {}
        self.fault_vars  = {}
        self.lreal_vars  = {}
        self.array_vars  = {}
        self.act_labels  = {}

        self._build_ui()
        self._refresh_loop()

    # ---------- UI ----------
    def _build_ui(self):
        # ============ TOP TOOLBAR (RESET) ============
        top = ttk.Frame(self.root)
        top.pack(fill="x", padx=8, pady=(8, 0))

        self.reset_btn = tk.Button(
            top, text="⚠  RESET FAULT 801  ⚠",
            font=("", 11, "bold"),
            bg="#c0392b", fg="white",
            activebackground="#e74c3c", activeforeground="white",
            relief="raised", bd=2,
            command=self._on_reset_fault,
        )
        self.reset_btn.pack(side="left", padx=(0, 10))

        self.reset_state = tk.StringVar(value="ResetDone: —")
        ttk.Label(top, textvariable=self.reset_state,
                  font=("", 10, "italic")).pack(side="left")

        # ============ NOTEBOOK ============
        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=8, pady=8)

        # --- Tab 1: Bools ---
        f1 = ttk.Frame(nb)
        nb.add(f1, text="Commands (Bool)")
        for i, name in enumerate(NODES_BOOL):
            v = tk.BooleanVar()
            self.bool_vars[name] = v
            cb = ttk.Checkbutton(
                f1, text=name, variable=v,
                command=lambda n=name: self._on_bool_toggle(n)
            )
            cb.grid(row=i, column=0, sticky="w", padx=10, pady=3)

        # --- Tab 2: Faults ---
        f2 = ttk.Frame(nb)
        nb.add(f2, text="Faults")
        for i, name in enumerate(NODES_FAULTS):
            ttk.Label(f2, text=name).grid(row=i, column=0,
                                          sticky="w", padx=10, pady=6)
            lb = ttk.Label(f2, text="—", width=8, relief="sunken",
                           anchor="center")
            lb.grid(row=i, column=1, padx=6)
            self.fault_vars[name] = lb

        # --- Tab 3: Speed ---
        f3 = ttk.Frame(nb)
        nb.add(f3, text="Speed")
        for i, name in enumerate(NODES_LREAL):
            ttk.Label(f3, text=name).grid(row=i, column=0,
                                          sticky="w", padx=10, pady=6)
            v = tk.StringVar(value="0.0")
            self.lreal_vars[name] = v
            ttk.Entry(f3, textvariable=v, width=15
                      ).grid(row=i, column=1, padx=6)
            ttk.Button(f3, text="Write",
                       command=lambda n=name: self._on_lreal_write(n)
                       ).grid(row=i, column=2, padx=6)

        # --- Tab 4: Cmd / Maint ---
        f4 = ttk.Frame(nb)
        nb.add(f4, text="Cmd / Maint")
        row = 0
        for name in NODES_ARRAY_CMD:
            ttk.Label(f4, text=name, font=("", 10, "bold")
                      ).grid(row=row, column=0, columnspan=9,
                             sticky="w", padx=10, pady=(10, 2))
            row += 1
            vars4 = []
            for j in range(4):
                ttk.Label(f4, text=f"[{j+1}]").grid(row=row, column=j*2,
                                                    padx=(10, 2))
                sv = tk.StringVar(value="0.0")
                ttk.Entry(f4, textvariable=sv, width=10
                          ).grid(row=row, column=j*2+1, padx=(0, 6))
                vars4.append(sv)
            self.array_vars[name] = vars4
            ttk.Button(f4, text="Write",
                       command=lambda n=name: self._on_array_write(n)
                       ).grid(row=row, column=8, padx=10)
            row += 1

        # --- Tab 5: Actual ---
        f5 = ttk.Frame(nb)
        nb.add(f5, text="Actual (read-only)")
        row = 0
        for name in NODES_ARRAY_ACT:
            ttk.Label(f5, text=name, font=("", 10, "bold")
                      ).grid(row=row, column=0, columnspan=9,
                             sticky="w", padx=10, pady=(10, 2))
            row += 1
            labs = []
            for j in range(4):
                ttk.Label(f5, text=f"[{j+1}]").grid(row=row, column=j*2,
                                                    padx=(10, 2))
                lb = ttk.Label(f5, text="—", width=10, relief="sunken",
                               anchor="e")
                lb.grid(row=row, column=j*2+1, padx=(0, 6))
                labs.append(lb)
            self.act_labels[name] = labs
            row += 1

        # --- Bottom status bar ---
        self.status = tk.StringVar(value="Disconnected")
        bar = ttk.Label(self.root, textvariable=self.status,
                        relief="sunken", anchor="w")
        bar.pack(fill="x", side="bottom")

    # ---------- Handlers ----------
    def _on_bool_toggle(self, name):
        val = self.bool_vars[name].get()
        try:
            self.worker.write_bool(name, val).result(timeout=1.0)
            self.status.set(f"Wrote {name} = {val}")
        except Exception as e:
            self.status.set(f"Write error: {e}")

    def _on_lreal_write(self, name):
        try:
            v = float(self.lreal_vars[name].get())
            self.worker.write_lreal(name, v).result(timeout=1.0)
            self.status.set(f"Wrote {name} = {v}")
        except Exception as e:
            self.status.set(f"Write error: {e}")

    def _on_array_write(self, name):
        try:
            vals = [float(sv.get()) for sv in self.array_vars[name]]
            self.worker.write_array4(name, vals).result(timeout=1.0)
            self.status.set(f"Wrote {name} = {vals}")
        except Exception as e:
            self.status.set(f"Write error: {e}")

    # ---------- RESET FAULT 801 ----------
    def _on_reset_fault(self):
        self.reset_btn.config(state="disabled", bg="#7f8c8d")
        self.reset_state.set("ResetDone: sending…")
        self.status.set("Reset 801 → sending pulse…")

        def _worker():
            try:
                ok = self.worker.reset_faults(hold_ms=300).result(timeout=6.0)
                if ok:
                    self.status.set("Reset 801 → OK (ResetDone=TRUE)")
                    self.reset_state.set("ResetDone: OK ✓")
                else:
                    self.status.set("Reset 801 → sent, but ResetDone never came")
                    self.reset_state.set("ResetDone: timeout ✗")
            except Exception as e:
                self.status.set(f"Reset error: {e}")
                self.reset_state.set("ResetDone: error ✗")
            finally:
                self.root.after(0, lambda: self.reset_btn.config(
                    state="normal", bg="#c0392b"))

        threading.Thread(target=_worker, daemon=True).start()

    # ---------- 1 Hz refresh ----------
    def _refresh_loop(self):
        def _read_all():
            try:
                bool_names  = list(NODES_BOOL)
                fault_names = list(NODES_FAULTS)
                lreal_names = list(NODES_LREAL)
                array_names = list(NODES_ARRAY_CMD) + list(NODES_ARRAY_ACT)

                fut = self.worker.read_all(
                    bool_names, fault_names, lreal_names, array_names
                )
                result = fut.result(timeout=5.0)
                bools, faults, lreals, arrays = result

                # bools
                for name, val in bools.items():
                    try:
                        self.bool_vars[name].set(val)
                    except Exception:
                        pass

                # faults
                for name, val in faults.items():
                    try:
                        self.fault_vars[name].config(
                            text="FAULT" if val else "OK",
                            foreground="red" if val else "green"
                        )
                    except Exception:
                        self.fault_vars[name].config(text="—",
                                                     foreground="black")

                # scalars
                for name, val in lreals.items():
                    try:
                        self.lreal_vars[name].set(f"{val:.2f}")
                    except Exception:
                        pass

                # writable arrays
                for name in NODES_ARRAY_CMD:
                    if name in arrays:
                        try:
                            for sv, v in zip(self.array_vars[name], arrays[name]):
                                sv.set(f"{v:.3f}")
                        except Exception:
                            pass

                # read-only arrays
                for name in NODES_ARRAY_ACT:
                    if name in arrays:
                        try:
                            for lb, v in zip(self.act_labels[name], arrays[name]):
                                lb.config(text=f"{v:.3f}")
                        except Exception:
                            pass

                # ResetDone indicator
                try:
                    done = self.worker.read_bool(NODE_RESET_DONE).result(timeout=1.0)
                    self.reset_state.set(f"ResetDone: {done}")
                except Exception:
                    pass

            except Exception as e:
                print(f"[refresh] {e}")

        threading.Thread(target=_read_all, daemon=True).start()
        self.root.after(1000, self._refresh_loop)


# ================== MAIN ==================
def main():
    worker = PlcWorker(PLC_URL)

    root = tk.Tk()
    gui = PlcGui(root, worker)

    def _connect():
        time.sleep(2)
        for attempt in range(3):
            try:
                worker.connect().result(timeout=5.0)
                gui.status.set(f"Connected to {PLC_URL}")
                return
            except Exception as e:
                if attempt < 2:
                    time.sleep(1)
                    continue
                gui.status.set(f"Connection failed: {e}")
                messagebox.showerror("Connection error",
                                     f"Could not connect to {PLC_URL}\n\n{e}")

    threading.Thread(target=_connect, daemon=True).start()

    def _on_close():
        try:
            worker.disconnect().result(timeout=2.0)
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _on_close)
    root.mainloop()


if __name__ == "__main__":
    main()