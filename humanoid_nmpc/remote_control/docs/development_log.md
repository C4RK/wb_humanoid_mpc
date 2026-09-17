# WMET Arm Teleoperation — Development Log

**Project:** Real-time arm teleoperation for Unitree G1 humanoid robot  
**Pipeline:** WMET EM tracker → calibration → retargeting → OCS2 MPC → G1 robot  
**Author:** Kun Ma, OVGU Magdeburg (da.ma@ovgu.de)  
**Thesis context:** Master's thesis — EM-tracker-based human arm teleoperation for a humanoid robot

---

## Hardware and software context

- **EM tracker:** WMET, 6-DOF pose at 50 Hz on `/em/pose` (IP: 10.42.0.144)
  - Transmitter: mounted on the user's shoulder (fixed frame origin)
  - Receiver: strapped to the user's forearm, near the wrist
- **Robot:** Unitree G1 23-DOF, left arm joints at MPC indices 13–16:
  - `left_shoulder_pitch_joint` (index 13)
  - `left_shoulder_roll_joint` (index 14)
  - `left_shoulder_yaw_joint` (index 15)
  - `left_elbow_joint` (index 16)
- **MPC:** OCS2 centroidal MPC, running in Docker (`launch_wb_mpc.bash`)
  - Docker uses `--rm` → container is deleted on restart → files inside `/root/` are lost
- **ROS2 version:** Jazzy
- **Key files:**
  - `humanoid_nmpc/remote_control/remote_control/calibration_node.py`
  - `humanoid_nmpc/remote_control/remote_control/retargeting_node.py`
  - `humanoid_nmpc/remote_control/config/wmet_calibration.yaml`

---

## Phase 1 — Two-sphere approach (failed)

### Idea
Use sphere fitting for both sub-problems:
- **Step 0:** Wrist traces a sphere while pivoting at the shoulder (elbow locked) → sphere center = shoulder joint, sphere radius ≈ total arm length.
- **Step 1:** Wrist traces a sphere while sweeping the forearm (shoulder fixed, elbow pivoting) → sphere center ≈ elbow joint, sphere radius ≈ L2 (forearm length).

### Why Step 0 works
When the elbow is locked and the shoulder pivots, the wrist is at a constant distance from the shoulder joint. The wrist trajectory lies on a sphere centred exactly at the shoulder joint. Algebraic least-squares sphere fitting gives a reliable shoulder position regardless of transmitter tilt.

### Why Step 1 failed
The elbow is a **hinge joint**, not a ball-and-socket joint. When the forearm is swept (shoulder fixed), the wrist moves in a **flat arc** (constrained to a single plane), not on a sphere. A sphere fitted to a planar arc is degenerate: any sphere whose axis passes through the arc is a valid fit, so the radius (which we needed as L2) was arbitrary and noise-dominated. Typical results varied by ±8 cm depending on how much of the arc was covered.

**Lesson for thesis:** The geometry of the joint type matters fundamentally. A hinge joint has 1 DOF; a ball-and-socket has 3. Sphere fitting is only appropriate for ball-and-socket kinematics.

---

## Phase 2 — Law of cosines for L2 (partially worked, unreliable)

### Idea
Given the shoulder joint S (from Phase 1 Step 0), total arm length L_total (from sphere radius), and a single recorded wrist pose W with forearm direction f (from quaternion):

The elbow E lies at W − L2·f. The triangle S–E–W has known sides:
- |SE| = L1 (unknown, but L1 = L_total − L2)
- |EW| = L2 (unknown)
- |SW| = d_sw (measured)

Law of cosines gives a closed-form expression for L2 in terms of the dot product V·f where V = W − S.

### Why it was unreliable
The formula requires only **one pose**. A single measurement is extremely sensitive to:
- Which exact arm position is used
- Measurement noise in position and orientation
- Slight elbow bend (violating the extended-arm assumption)

Across five test sessions with the same person, L2 varied as: 21.2, 24.6, 26.0, 27.4, 27.9 cm — an 8 cm spread. No single value could be trusted.

**Lesson for thesis:** Single-pose solutions are fragile. Overdetermined systems (many poses, least squares) are far more robust to noise.

---

## Phase 3 — Simultaneous 2D linear solve (introduced, then refined)

### Key mathematical insight
For any arm pose, the constraint that the elbow E = W − L2·f lies at distance L1 from the shoulder joint S is:

```
|W − S − L2·f|² = L1²
```

Letting V = W − S and expanding:

```
|V|² − 2(V·f)L2 + L2² = L1²
```

Rearranging with u = L1² − L2² (a new combined unknown):

```
2(V·f)·L2 + u = |V|²
```

This is **linear** in [L2, u]. With N ≈ 750 poses from 15 seconds of free arm motion, stacking these equations gives an overdetermined system solved by least squares. L1 = √(L2² + u).

### Why this is better
- Both L1 and L2 are solved simultaneously — no subtraction dependency
- Many poses average out noise
- Good conditioning requires V·f to vary widely (achieved by bending and extending the elbow across a large range)

### Results within a session
Excellent consistency:
- Elbow residual RMS: 0.6–0.8 cm (across 750 poses)
- L1+L2 vs. sphere radius: within 0.5–0.7 cm

### Remaining problem: between-session variability
L1 ranged from 24.6 to 29.0 cm across sessions, even though L1+L2 was always consistent (~53–55 cm). **Root cause:** In any 2-parameter regression, slope and intercept are negatively correlated. If the sampling distribution of V·f changes between sessions (different motion patterns), the estimated L2 changes while L1²=L2²+u stays roughly constant. The total is an invariant; the individual split is not.

---

## Phase 4 — Constrained 1D linear solve (current approach)

### Key insight
The total arm length L_total = sphere_radius is already a reliable, **independent** measurement from Step 0. Enforcing L1 + L2 = L_total as a hard constraint eliminates the slope/intercept correlation:

Starting from |V − L2·f|² = (L_total − L2)²:

Expand both sides:
```
|V|² − 2(V·f)L2 + L2² = L_total² − 2·L_total·L2 + L2²
```

Cancel L2² from both sides:
```
L2 · 2(V·f − L_total) = |V|² − L_total²
```

This is a **1D least squares** in L2 only. The normal equation is:

```
L2 = Σ aᵢbᵢ / Σ aᵢ²   where aᵢ = 2(Vᵢ·fᵢ − L_total), bᵢ = |Vᵢ|² − L_total²
```

Then L1 = L_total − L2.

### Properties
- The estimator is naturally weighted toward poses where the arm is **bent** (large |aᵢ|), which is where L2 is most directly observable
- No correlation between slope and intercept — the split is stable across sessions
- Both the constrained (1D) and unconstrained (2D) solvers run and are reported side by side as a cross-check

### When the constrained solver is better
Always, as long as the elbow was locked in Step 0 (so sphere_radius ≈ true L_total). The cross-check compares L1 from both methods: disagreement > 3 cm suggests the elbow was slightly bent during Step 0 or the free motion in Step 1 was insufficient.

---

## Frame alignment — three iterations

### Problem discovered
After the arm length calibration was working, the retargeting node produced wrong joint angles. With the arm hanging straight down, the robot received:
- pitch: ~0° ✓
- **roll: +42°** ✗ (should be ~0°)
- yaw: flipping 0° ↔ −150° ✗

### Root cause
The IK code assumed the EM transmitter's −Z axis points in the direction of gravity ("arm hanging down = −Z direction"). But the transmitter is a physical box strapped to the shoulder; its coordinate axes are fixed to the box, not to gravity. When mounted at a tilt, the "arm down" direction in the transmitter frame is a non-trivial mixture of transmitter Y and Z axes.

This is a **frame alignment problem**: the transmitter frame ≠ the robot's expected shoulder frame.

---

### Fix iteration 1 — Vertical tilt correction (Step 2 calibration)

**Added:** 5-second "arm hanging down" calibration pose after arm lengths are solved.

**Method:** Record mean wrist position W and forearm direction f during the arm-down hold. Compute the elbow direction:
```
E_down = mean(Wᵢ − S − L2·fᵢ) / |mean(...)|
```
Save `arm_down_hat` = unit vector pointing from shoulder toward elbow when arm hangs down.

**In retargeting:**
Build R1 = rotation_between(arm_down_hat, [0,0,−1]) using Rodrigues' formula. Apply R1 to all E and forearm_dir vectors before IK angle computation.

**Result:** roll dropped from +42° to −2° for arm-down position. ✓

**Remaining problem:** Roll = +30° when arm raised forward (horizontal rotation not yet corrected).

---

### Fix iteration 2 — Horizontal rotation correction (Step 3 calibration)

**Root cause of remaining roll:** R1 is a rotation in the plane containing arm_down_hat and [0,0,−1]. It corrects the vertical tilt but leaves a rotation around the vertical (Z) axis undetermined. The transmitter was also rotated ~30° in the horizontal plane — its "forward" axis didn't align with the robot's forward (+X) direction.

**Added:** 5-second "arm raised straight forward" calibration pose.

**Method:** Record mean elbow direction `arm_forward_hat` in transmitter frame. Apply R1 to get `fwd_after_R1`. Project onto the horizontal (XY) plane:
```
θ = atan2(fwd_after_R1[1], fwd_after_R1[0])
```
Build R2 = rotation around Z by −θ (makes Y component of forward direction zero).

Full correction: **R_full = R2 @ R1**.

**Verification:**
- R_full @ arm_down_hat = [0, 0, −1] ✓ (arm-down → zero pitch/roll)
- R_full @ arm_forward_hat has Y ≈ 0 ✓ (arm-forward → zero roll)

**Result:** roll dropped from +30° to −2° when arm is raised forward. ✓

---

### Fix iteration 3 — Yaw singularity guard

**Problem:** When arm hangs down (within ~5° of vertical), shoulder yaw was computed as −150° (joint limit) instead of 0°.

**Root cause:** The yaw computation uses a reference vector:
```
ref = world_down − (world_down · E_hat) · E_hat
```
When E_hat ≈ [0,0,−1] (arm pointing down), ref ≈ 0. The old guard threshold (0.05) was too small: with pitch ≈ 3°, ref_norm ≈ sin(3°) ≈ 0.052, which marginally passed the guard and produced garbage atan2 output.

**Fix:** Explicitly check the angle between E_hat and [0,0,−1]. If arm is within 20° of vertical, yaw is geometrically undefined (any axial rotation looks the same from the outside) — return 0.

**Additional insight:** The yaw also shows noise from wrist rotation. The EM sensor is on the forearm; it measures the combined orientation of forearm + wrist. Small wrist movements change the computed shoulder yaw. This is an inherent limitation of the one-sensor placement — a second sensor on the upper arm would separate shoulder yaw from wrist rotation.

---

## Calibration file persistence fix

**Problem:** Calibration was saved to `/root/.ros/wmet_calibration.yaml` inside the Docker container. The container uses `--rm` → deleted on every restart → calibration lost.

**Fix:** Changed `CALIBRATION_FILE` to the mounted workspace volume:
```
/wb_humanoid_mpc_ws/src/wb_humanoid_mpc/humanoid_nmpc/remote_control/config/wmet_calibration.yaml
```

---

## Forearm axis identification

**Problem:** The forearm direction vector (used throughout the IK) is computed as:
```python
forearm_dir = R @ FOREARM_LOCAL_AXIS
```
where R is the rotation matrix from the sensor quaternion, and `FOREARM_LOCAL_AXIS` is a unit vector in the sensor's local frame pointing from elbow toward wrist.

**How it was determined:** The sensor's local Y axis (+Y = [0,1,0]) was confirmed empirically to point from elbow toward wrist based on sensor mounting tests.

**Why it matters:** If the axis is wrong (e.g. −Y or Z), the computed elbow position E = W − L2·forearm_dir points in the wrong direction, and all joint angles are wrong.

---

## Current calibration procedure summary

| Step | Instructions to user | Duration | Mathematical output |
|------|----------------------|----------|---------------------|
| 0 | Sweep arm with locked elbow in many shoulder directions | 8 s | Shoulder joint center S; sphere_radius ≈ L_total |
| 1 | Free arm motion: continuously bend/extend elbow while moving shoulder | 15 s | L1, L2 via 1D constrained + 2D cross-check |
| 2 | Let arm hang straight down, hold still | 5 s | arm_down_hat → R1 (vertical tilt correction) |
| 3 | Raise arm straight forward to shoulder height, hold still | 5 s | arm_forward_hat → R2 (horizontal rotation correction) |

Re-calibration needed every time the transmitter is remounted (tilt and rotation change with each mounting).

---

## Current retargeting IK pipeline

```
/em/pose (PoseStamped, 50 Hz)
  │
  ├─ W = raw_position − shoulder_joint   (wrist in shoulder-centred frame)
  ├─ R = quat_to_matrix(quaternion)
  ├─ forearm_dir = R @ [0, 1, 0]         (forearm direction in transmitter frame)
  │
  ├─ E = W − forearm_dir · L2            (elbow position)
  ├─ E, forearm_dir = R_full @ E, R_full @ forearm_dir   (frame alignment)
  │
  ├─ shoulder_pitch = atan2(−E_hat[0], −E_hat[2])
  ├─ shoulder_roll  = asin(E_hat[1])
  ├─ shoulder_yaw   = (0° if arm within 20° of vertical,
  │                    else angle between gravity-ref and forearm-perp)
  ├─ elbow_joint    = acos((L1²+L2²−d_sw²)/(2·L1·L2)) − π/2
  │
  └─ /arm_joint_target (JointState, 21 joints, 50 Hz)
```

---

## Test results log

### Session — First IK output (before frame alignment)

Arm hanging down:
```
pitch: ~0°  roll: +42°  yaw: 0° or −150°  elbow: ~70°
```
Roll offset of 42° = transmitter vertical tilt. Yaw instability = singularity in near-vertical arm position.

---

### Session — After Step 2 (vertical tilt correction only)

Arm hanging down:
```
pitch: ~−2°  roll: ~−1°  yaw: 0°  elbow: ~70°
```
Roll corrected. Yaw singularity fixed.

Arm raised forward (horizontal):
```
pitch: ~−71°  roll: +30°  yaw: −90° to −150°
```
Roll = +30° = residual horizontal rotation. Yaw large negative = unresolved.

---

### Session — After Steps 2+3 (full frame alignment), constrained solver

Calibration:
- Sphere: 53.0 cm, RMS 0.0 cm
- Constrained: L1 = ?, L2 = ?, total = 53.1 cm (anchored to sphere)
- arm_down tilt: +42.4°; horizontal correction: +29.8°

Arm hanging down:
```
pitch: −6°  roll: −2°  yaw: 0°  elbow: ~71°
```

Arm raised forward (~65° from vertical, not fully horizontal):
```
pitch: −65°  roll: −2°  yaw: −55° to −75°  elbow: ~55°
```
Roll is now correct. Yaw residual under investigation — may be genuine shoulder twist or URDF convention mismatch (to be verified in simulation).

Arm raised sideways (~40° from vertical):
```
pitch: −36°  roll: +39°  yaw: −20°
```
Pitch non-zero = arm was raised diagonally forward-sideways (not pure sideways). Expected for a natural motion.

---

---

## Session — 2026-09-15: First MuJoCo visualization

### What was done
- Wrote `scripts/mujoco_replay.py`: parses retargeting_node log output (pitch/roll/yaw/elbow in degrees + ROS timestamps), loads G1 MuJoCo Menagerie model, replays joint angles in the interactive viewer.
- Replayed trajectory from first calibrated test (arm down → arm forward → arm sideways → return).
- Model used: `/home/kunwang/humanoid-motion-planning/mujoco_menagerie/unitree_g1/scene.xml`, keyframe "stand" as base pose.

### Observations
- MuJoCo visualization confirmed the arm does move in response to human motion.
- **Discrepancies noted between MuJoCo visualization and expected real-world motion** (details to be confirmed in structured tests on 2026-09-16).
- Candidate issues: yaw sign/convention, pitch not reaching full horizontal (−66° not −90°), brief unstable frames during fast transitions (yaw hit −122°, elbow dropped to +19.5° at t=6.1 s).

### Changes made
- Logging rate increased from 1 Hz to 10 Hz (throttle_duration_sec 1.0 → 0.1) for smoother MuJoCo replay.
- Log files saved to `scripts/trajectory_YYYY_MM_DD.txt` for replay and thesis documentation.

### Planned for 2026-09-16
Structured single-joint tests:
1. Pitch only: arm down → forward at 45° / 90° / 135° increments
2. Roll only: arm down → sideways at 45° / 90°
3. Yaw: arm forward, rotate forearm palm-up vs palm-down → check robot direction
4. Elbow: bend/extend with shoulder fixed → check robot range and direction

---

## Open questions / future work

1. **Shoulder yaw convention:** The yaw = −60° when arm is raised forward needs to be verified against the G1 URDF to determine whether it is:
   - Physically correct (genuine shoulder external rotation)
   - A sign flip (should be +60°)
   - A zero-offset issue (should be 0° + some rotation)
   This will be resolved by testing in simulation and observing the robot arm visually.

2. **Single-sensor limitation for yaw:** The EM sensor on the forearm measures the combined orientation of forearm + wrist. Small wrist rotations corrupt the shoulder yaw estimate. For publication-quality teleoperation, a second sensor on the upper arm would allow separating shoulder yaw from wrist orientation.

3. **Elbow angle offset:** The elbow reads ~70° when arm is roughly extended (true elbow ≈ 160°, nearly straight). This is correct: the robot elbow = 90° only for a perfectly locked arm. The slight bend in a relaxed hanging arm corresponds to ~70°. No correction needed.

4. **Sensor placement sensitivity:** Arm lengths L1 and L2 depend on where exactly the receiver is mounted on the forearm. The calibration procedure measures "shoulder joint to receiver position," not "shoulder joint to wrist joint." The sensor should be mounted consistently at the same forearm location each session.

5. **Simulation pipeline test:** Full pipeline test (EM tracker → retargeting → OCS2 MPC → simulated G1) pending.

6. **Real robot test:** Pending approval and hardware access. Requires verifying that the `/arm_joint_target` joint commands are within safe operating ranges before enabling on hardware.

---

## Key design decisions

| Decision | Rationale |
|---|---|
| One EM receiver (forearm only) | Simplicity; sufficient for pitch, roll, elbow; yaw is approximate |
| FOREARM_LOCAL_AXIS = [0,1,0] | Empirically confirmed: sensor Y axis points elbow → wrist |
| Calibration saved to mounted volume | Docker `--rm` deletes container-internal files on restart |
| Sphere fit for shoulder joint | Robust to transmitter tilt; works regardless of arm length assumption |
| Two-step frame alignment (R1 then R2) | R1 and R2 are independent rotations; each corrects one degree of freedom |
| Singularity guard at 20° from vertical | Yaw is geometrically undefined near vertical; guard prevents ±150° clamp artefact |
| Constrained 1D solver as primary | Eliminates regression correlation; uses reliable sphere-radius as anchor |
| 2D unconstrained solver as cross-check | Detects when motion was insufficient or sphere fit was done with bent elbow |
