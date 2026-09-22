### Kinematic Mismatch: Isaac Sim vs. TIA Portal & The Custom URDF Solution

#### The Problem: Kinematic Discrepancy (Joint 1 & Joint 2 Misalignment)
During the initial integration phase, a significant kinematic mismatch was observed between the robot simulation in **Isaac Sim** and the digital twin in **TIA Portal (NX MCD/Simatic)**. 

The root cause was traced to the **geometric representation of Link 1** (the robot base/pedestal):
*   **In Isaac Sim (3D Model):** The imported KUKA KR210 L150 model accurately reflects the physical robot, where Joint 2 is offset both **horizontally and vertically** from Joint 1 (the base rotation axis). This offset is critical for correct kinematic calculations.
*   **In TIA Portal (Schematic):** The kinematics editor simplified the robot structure, representing Link 1 as a **straight vertical line**. This implied that Joint 1 and Joint 2 lie on the exact same vertical axis, completely ignoring the physical horizontal and vertical offsets (A1 and D1 parameters in KUKA kinematics).

**Consequence:** Because TIA Portal was calculating forward and inverse kinematics based on a simplified 2D schematic, the joint angles and end-effector positions diverged significantly from Isaac Sim’s physics-based calculations. This resulted in **motion desynchronization**, making it impossible to validate the control logic.

---

#### The Solution: Switching to a Custom Minimal URDF [arm4_kr210_like.urdf]
To resolve this mismatch and ensure 1:1 kinematic synchronization, the decision was made to **replace the pre-built, high-fidelity KUKA robot model** with a **custom, minimal URDF (Unified Robot Description Format)** model.

**Why a Custom Minimal URDF?**

1.  **Parameterized Kinematic Equivalence:** Instead of relying on the complex, non-editable mesh of the commercial KUKA model, the custom URDF was built from scratch. This allowed us to explicitly define the **exact same Denavit-Hartenberg (DH) parameters** (link lengths, offsets, and joint limits) that are configured inside TIA Portal. By defining the exact horizontal (`a1`) and vertical (`d1`) offsets between Joint 1 and Joint 2 in the URDF, both platforms now share the same mathematical foundation.
2.  **Simplified Geometry for Performance:** The original KUKA model contained high-polygon meshes and complex collision geometries. The custom URDF uses primitive shapes (cylinders, boxes) to represent the links. This drastically reduces the computational load on the physics engine in Isaac Sim, allowing for faster iteration and real-time synchronization with the PLC.
3.  **Elimination of "Black Box" Configurations:** Pre-built models often come with hidden joint definitions or custom plugins that are difficult to modify. By using a "raw" URDF, we have full control over every joint's origin, axis of rotation, and parent-child relationships. This transparency guarantees that no hidden offsets or transformations interfere with the PLC's commands.
4.  **Direct Mapping to TIA Portal Parameters:** The custom URDF was designed specifically to mirror the "Transformation parameters" (Length L1, L2, L3, etc.) seen in the TIA Portal Geometry editor. This creates a direct, transparent mapping where a change in the URDF directly corresponds to a parameter in the PLC configuration, ensuring that the digital twin and the simulation remain perfectly aligned.

**Result:** By using a lightweight, mathematically precise URDF that explicitly models the offsets between Joint 1 and Joint 2, the kinematic mismatch was eliminated. The joint angles received from TIA Portal now produce identical end-effector poses in Isaac Sim, enabling reliable software-in-the-loop (SIL) testing.
