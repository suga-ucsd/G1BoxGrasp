# Walking Pick-and-Place (Unitree G1 + Inspire hands): how it works

This document explains how the **Walking Pick and Place** LuckyEngine project gets a Unitree G1 humanoid to walk to a can, grab it, turn around, and drop it into the recycle bin behind it. Everything here was reverse-engineered from the scripts, the scene file (`Assets/Scenes/CupCleaner.hscene`) and the policy descriptors. Use it as a blueprint for other LuckyEngine humanoid projects.

---

## 0. TL;DR (the whole trick in 12 lines)

1. **Legs come from learned policies; arms come from IK; fingers are driven by the script.** Three ONNX policies (`walker`, `rotator`, `croucher`) sit in the `RobotControllerComponent`. A motion graph with two `LimbIK` nodes drives the arms. The script writes finger actuator values directly.
2. **Joint masks decide who owns which joint.** `SetDrivenJoints(policy, jointNames[])` removes the IK arm from the policy's control. If you don't do this, the policy and the IK fight over the joint.
3. **Only one locomotion policy is active at a time.** Walker is for walking, Rotator is for turning in place, Croucher is for lowering the pelvis. You swap between them with `SetPolicyActive`.
4. **The behaviour is a queue of small state objects (`Subroutine`s)** ticked by a scene-wide `TaskOrchestrator`. The explicit chain lives in `EnqueueGrabCycle()`. Linear sub-sequences, like the pickup (Settle → PolicyToIK → Reach → Closing → Holding → HoldDwell), chain themselves from `OnComplete` via `EnqueueImmediate` (a **prepend**).
5. **Subroutines don't write to the engine directly for most things.** They set shared state on the robot script (`CurrentPhase`, `IKActive`, `IKTargetPos`, `CloseTarget`, `OrientOffsetOverride`…). Every frame the robot script's output stages (`WriteIKTargets`, `UpdateFingers`, `WriteWalker`, `WriteCroucher`, …) turn that state into engine commands.
6. **All targets are scene entities** (empty or invisible marker meshes) wired into script fields. You tune the behaviour by dragging markers around in the editor, not by editing numbers.
7. **Every position read comes from MuJoCo** (palm *site*, pelvis *body*), converted MuJoCo→Hazel axes. Entity transforms of physics bodies are not used for control.
8. **Carry and hold poses are body-relative:** shoulder world position + pelvis rotation × a body-local offset (MuJoCo axes: X = forward, Y = left, Z = up). That way the cup follows the body while walking and turning.
9. **Stop before you switch policies** (Settle), and **ramp the yaw rate in** when switching to the rotator. The rotator was trained from standstill, so it stumbles if it inherits forward momentum.
10. **Never let IK hold a stale world target.** When IK "isn't in use", write the live palm pose as its target (zero residual). Otherwise the arm contorts as the body moves.
11. **Hand the arm back from IK to the policy gently** (`IKToPolicyHandoffSubroutine`): glide to a rest pose near the policy default, then drop IK.
12. **Loop with `MujocoSceneComponent.ResetToInitialState()`**, a C# state reset, and sync-point-aligned timing for deterministic data recording. A global watchdog force-resets stuck runs.

---

## 1. What actually runs

| Item | Value |
|---|---|
| Scene | `Assets/Scenes/CupCleaner.hscene` (scene name inside file: `fingers`, file/enum name: `CupCleaner`) |
| Robot entity | `g1_29dof_rev_1_0_with_inspire_hand_DFQ` at world (-1.95, 0, -1.86), yaw 0 (facing +X) |
| Robot script | **`WalkerIKExample.GrabUpDownTurnTasksV2`** (inherits `GrabUpDownTurnTasks`, "V1") |
| Orchestrator entity | `TaskOrchestrator` at origin with script `WalkAndGrab.Tasks.TaskOrchestrator` (no fields) |
| MuJoCo model | `Robots/g1_description/g1_29dof_rev_1_0_with_inspire_hand_DFQ.xml` (29 body DoF + 12×2 Inspire finger joints) |
| Motion graph | `Robots/g1_description/mographs/g1_inspire_graph.hmograph` (two `LimbIK` nodes, shoulder_pitch_link → palm_site) |
| Policies | slot 1 `walker` (`policy_descriptor.walker.json` at Assets root), slot 2 `rotator` (`ContentVault/Robots/UnitreeG1/Policies/policy_descriptor.rotator.json`), slot 3 `croucher` (`policies/policy_descriptor.croucher.json`) |

The task you described ("walk to the can, grab it, turn around, throw it in the bin behind") is the **cup A leg** of V2's `BinDropCycleV2`. The full cycle goes on to do a **cup B leg**: it switches to the right hand, walks to a second, lower can on another bench, crouches, grabs it, turns, and places it on the first bench. It then turns and walks to a finish marker, stops, resets MuJoCo to keyframe 0, and loops forever while recording episodes.

### 1.1 Top-down map (Hazel world, Y is up, looking down; +X right, +Z down the page)

```
 z=-2.5 ┤                    pillar(-0.60,-2.53)
        │   ROBOT START                              bench2 (0.73,-2.22)
 z=-2.0 ┤   (-1.95,-1.86) ──walk +X──►  Left_Walk      ● Left_Can (0.27, y0.96, -2.06)  ← cup A (high)
        │                               (-0.04,-1.92)  ○ Bin_Walk_Target_Alt (0.25,-1.68) ← cup B drop stance
        │                                              ◇ Bin_Drop_Target_Alt (0.51, y0.99, -2.03) ← cup B placed here
 z=-1.0 ┤
        │                    pillar(-0.60,-0.65)
 z=-0.5 ┤ BIN (-3.09,-0.43)
        │  Bin_Walk_Target (-2.76,-0.62)   Bin_Drop_Target (-2.98, y0.73, -0.38)
 z= 0.5 ┤                                    ★ Finish_Target (-0.01, 0.51)
        │                                              bench1 (0.77, 0.60)
 z= 1.0 ┤                    pillar(-0.60, 1.22)       ● Right_Can (0.80, y0.56, 0.84) ← cup B (low → crouch)
        │                                              ○ Right_Walk (0.54, 0.88)
        └──────┬──────────┬──────────┬──────────┬──────────┬──
             x=-3       x=-2       x=-1        x=0        x=1
```

Cup A leg: start → Left_Walk → grab Left_Can with the **left** hand → turn about 150° → walk to Bin_Walk_Target → slide palm to Bin_Drop_Target → open fingers so the can falls into the bin → retract.

---

## 2. Project layout and build system

```
Assets/
├── Scenes/CupCleaner.hscene(.auto)          ← scene (autosave differs only in material tables)
├── Scripts/
│   ├── Client/WalkingPickAndPlace.csproj     ← AUTO-GENERATED, never edit
│   ├── Client/Source/                        ← our code
│   │   ├── GrabUpDownTurnTasks.cs   (2145 lines, V1 robot script + ALL shared plumbing)
│   │   ├── GrabUpDownTurnTasksV2.cs (320 lines, V2 bin-drop cycle; the one in use)
│   │   ├── Piper.cs                 (unrelated leftover: Piper-arm Jenga demo, namespace jenga28)
│   │   └── Tasks/
│   │       ├── Task.cs, Subroutine.cs, TaskComponent.cs, TaskOrchestrator.cs  (framework)
│   │       └── Subroutines/*.cs     (40 files, one behaviour each; 2 are dead stubs)
│   ├── Source/Generated/*.cs                 ← AUTO-GENERATED enums (policy ids + commands)
│   └── Binaries/  Intermediates/             ← build output
├── policy_descriptor.walker.json, walker.onnx(.data), walker.honnx, model_config.json
├── policies/  croucher.*  (+ duplicate walker, model_config)
├── ContentVault/Robots/UnitreeG1/Policies/   rotator.*, croucher.*, walker.*, right_reacher.*, walker_Policy.cs
├── Robots/g1_description/  URDF/MJCF variants, meshes, mographs/g1_inspire_graph.hmograph
└── Physics/ConvexHulls/     VHACD hulls for cans / bin / benches
```

**Build facts and gotchas**

- The csproj compiles **every `.cs` under `Assets/`** (`../../**/*.cs`). It excludes only `Scripts/Binaries`, `Scripts/Intermediates` and folders whose name ends in `~`. So:
  - `ContentVault/.../walker_Policy.cs` and `Piper.cs` are compiled into the same DLL. They are harmless unless attached to an entity. **To disable a script without deleting it, move it into a folder named `Something~`.**
  - There is no manual "add file to project" step. Just create the `.cs` file.
- Target: `net9.0`, `Nullable=enable`, references `Hazel-ScriptCore.dll` and `Coral.Managed.dll` from the engine install.
- **Generated enums** (`Scripts/Source/Generated/`):
  - `CupCleanerPolicyIds { walker=1, rotator=2, croucher=3 }` is generated **from the scene named `CupCleaner`** by the SceneSerializer on save. **If you rename the scene or copy the code into a project with a different scene name, the enum name changes and `CupCleanerPolicyIds.croucher` stops compiling.**
  - `WalkerCommands`, `RotatorCommands` (`SetVx=1, SetVy=2, SetYawRate=3`) and `CroucherCommands` (`…, SetTargetHeight=4`) are generated from the policy descriptors. Command IDs are 1-based in the order of the descriptor's `"command"` names list.
  - `PolicyIds.Walker` / `PolicyIds.Rotator` are **engine-provided** (in `Hazel-ScriptCore.dll`), not generated. The code mixes them with the generated `CupCleanerPolicyIds.croucher`. These are assumed to map to slots 1 and 2, and it works in this scene. In a new project, **put walker in slot 1 and rotator in slot 2** or verify what `PolicyIds` resolves to.
- Script binding in the scene is by `ScriptName` (full `Namespace.Class`) plus a `ScriptID`, and field values are stored per field name/ID. **Renaming a class, namespace or public field silently drops its binding or saved value.**
- Enum-typed public fields are **not** exposed by the inspector or serializer. That's why `AltBinTurnDirection` is an `int` with `[Dropdown]`, and why `Mode` (`CycleMode`) can't be set from the scene (see gotcha G1).
- Inspector attributes used (all from Hazel): `[Group]`, `[Header]`, `[Divider]`, `[Space]`, `[Tooltip]`, `[ClampValue]`, `[Slider]`, `[Min]`, `[Units]`, `[DisplayName]`, `[ToggleButton("On","Off")]`, `[Dropdown(...)]`, `[EditCondition("BoolField")]`, `[HideCondition("BoolField", Invert=true)]`, and `[Button("Label")]` on an `internal void` method (runtime-only button).

---

## 3. Scene setup in detail

### 3.1 Scene time settings (important for policies)

- `SceneTimeMode: HighPerformanceDeterministic`.
- Time runners: **Robot 50 Hz**, Capture 30 Hz, Sync 10 Hz.
- `OnUpdate` runs on the Robot runner at 50 Hz with priority -200 (phase 1), before `UpdateRobotControllers` (priority 0, phase 1). So commands written in `OnUpdate` are consumed in the same tick.
- MuJoCo: 50 Hz, `Substeps: 10`, `Mode: Performance_Substeps`, Jolt enabled too.
- Data recorder and camera recorder run on the Capture runner (30 Hz). `ObserverConfig: TotalEpisodes 300, EpisodesPerFile 1, FilesPerChunk 40`.
- `TimeSync.SyncHz` comes from the Sync runner (10 Hz). It's used to align resets and episode starts (§8).

### 3.2 Robot entity components

- `ScriptComponent` → `WalkerIKExample.GrabUpDownTurnTasksV2` with all fields wired (table in §3.4).
- `RobotControllerComponent`:
  - `MotionGraph` = `g1_inspire_graph.hmograph`.
  - `Policies` list: all three slots are serialized with **`Active: false`**. The script activates walker in `OnCreate`. `ClampObservationForUnclaimedJoints: true` on all three: when a joint is masked out (claimed by IK), the policy's observation for it is clamped, so the policy isn't confused by an arm it doesn't control.
  - `GraphInputs` has stale saved values for the IK inputs. They're overwritten every frame.
- `MujocoSceneComponent` → the MJCF asset.

### 3.3 The MJCF has custom palm sites

`g1_29dof_rev_1_0_with_inspire_hand_DFQ.xml` contains:

```xml
<site name="left_palm_site"  pos="0.20 0 -0.01" size="0.015" rgba="0.2 1 0.4 0.6"/>   <!-- inside left wrist_yaw body -->
<site name="right_palm_site" pos="0.20 0 0.01"  size="0.015" rgba="1 0.55 0.25 0.6"/>
```

These are the IK end-effectors (the motion graph `LimbIK` nodes use `left_palm_site` / `right_palm_site` with chain root `*_shoulder_pitch_link`). They're also the "palm position" the script measures every distance with. **A new robot needs equivalent sites added to its MJCF**, placed at the center of the grasp, not at the wrist.

Finger actuators are MJCF `<position>` actuators named after their joints (`L_thumb_proximal_yaw_joint`, …) with a very soft `kp=3.0`, `kv=0.3`. The script looks them up by name with `m_Mjs.GetActuator(name)`.

### 3.4 Wired entity fields (what each marker is and where it sits)

Positions are world coordinates, except `Left_Can_Target`, which is a child of the can.

| Field (V1/V2) | Entity | Pos | Role |
|---|---|---|---|
| `TargetObject` | `Left_Can_Target` (child of `Left_Can`) | local (0,0,-1.102) × parent scale 0.07 ⇒ ≈7.7 cm −Z from can center | **Cup A grab point.** It's a child, so it moves with the can's free joint. |
| `WalkTarget` | `Left_Walk` | (-0.04, 0, -1.92) | Cup A stance. Walk stops at `WalkToStandDistance + WalkToTolerance` = 0.25 m from it. |
| `ApproachAim` / `ReachStartTarget` | — (null) | | Cup A uses the computed parking spot. |
| `BinWalkTarget` | `Bin_Walk_Target` | (-2.76, 0, -0.62) | Stance next to the bin. Also the target for TurnToFaceBin. |
| `BinReleaseTarget` | `Bin_Drop_Target` | (-2.98, 0.73, -0.38) | Palm goes here, then the fingers open (`EnableBinReleaseSlide=true`). |
| `TargetObjectAlt` | `Right_Can_Target` | (0.80, 0.54, 0.87) | Cup B grab point (a sibling marker, **not** a child of the can). |
| `WalkTargetAlt` | `Right_Walk` | (0.54, 0, 0.88) | Cup B stance. |
| `ApproachAimAlt` | `Right_Can_Face_Target` | (0.73, 0.56, 0.71) | Cup B: rotate to face this, then slow-approach it. |
| `ReachStartTargetAlt` | `Right_Can_Start_Target` | (0.62, 0.54, 0.86) | Cup B: explicit IK parking spot before Reach. |
| `BinWalkTargetAlt` | `Bin_Walk_Target_Alt` | (0.25, 0, -1.68) | Cup B drop stance (next to bench 2). |
| `BinPlacedownStartAlt` | `Right_Drop` | (0.35, 1.09, -2.03) | Cup B placedown waypoint 1 (also wired as `DropoffTargetAlt`). |
| `BinPreReleaseTargetAlt` | `Bin_Pre_Drop_Target_Alt` | (0.41, 1.07, -2.03) | Cup B waypoint 2 (route around the rim). |
| `BinReleaseTargetAlt` | `Bin_Drop_Target_Alt` | (0.51, 0.99, -2.03) | Cup B release point. |
| `BinRetractTargetAlt` | `Right_Drop_Retract` | (0.61, 1.04, -1.89) | Cup B lift-away point after release. |
| `FinishTarget` | `Finish_Target` | (-0.01, 0, 0.51) | End-of-cycle walk target. |
| `DropoffTarget`, `DropStart`, `DropoffTargetAlt`, `DropStartAlt` | `Left_Drop`, `Left_Drop_Start`, `Right_Drop`, `Right_Drop_Start` | | V1 dropoff routing. **Not used by V2's chain**; only the telemetry `dropoffDist` reads them. |
| `DebugIKTarget(Alt)` | `Left_Debug` / `Right_Debug` | | A mesh teleported to `IKTargetPos` every frame (visual debug). |

Marker conventions: small, invisible (`Visible: false`) `StaticMeshComponent` spheres or boxes, or plain entities with no components. Floor-level markers (Y≈0) are for walking. Markers at palm height are IK targets.

### 3.5 Physics of the cans, bin and furniture

- Cans (`Left_Can`, `Right_Can`): parent entity scaled (0.07, 0.13, 0.07), `MujocoMeshColliderComponent` with a VHACD convex hull, **`Density: 5`** (very light), **`Friction: [10, 0.1, 0.01]`** (very grippy), and a `MujocoJointComponent` of type `"\x00"` (**free joint**, so it can be picked up). The visible can mesh is a child (`can`) with an inverse-ish scale.
  - Very high friction plus low mass is what lets a **partial** finger close (0.5–0.65 of full) hold the can without torque mode.
- `recycle_bin`: mesh collider, VHACD with a high voxel resolution (16M) so the opening is modelled concavely. Otherwise the can bounces off a "lid".
- `mj_bench` ×2: invisible box meshes with mesh colliders act as tables. `mj_pillar` ×3: cylinder colliders. `Ground`: a box collider at y=-0.5 with scale 10.
- **Reset** (`ResetToInitialState`) restores every free joint (both cans) to keyframe 0. There's no script-side can respawn.

---

## 4. Control architecture: who drives what

```
            ┌──────────────────────── GrabUpDownTurnTasksV2.OnUpdate (50 Hz) ───────────────────────┐
 Subroutine │ sets: CurrentPhase, IKActive, IKTargetPos, OrientOffsetOverride, CloseTarget,          │
 (ticked by │       OffHandCarrying, WalkTargetOverride, CrouchHeightGoal, UseLeftHand, policy on/off│
 Orchestr.) │       + writes rotator yaw directly in TurnToFace* subroutines                         │
            └────────────────────────────────────────────────────────────────────────────────────────┘
                     │ output stages, every frame, in this order:
                     ▼
  UpdateFingers ─────────► MujocoActuator.Value for 12 finger actuators / hand (script-owned, a.IK=false)
  UpdateCrouchHeightRamp ► CommandedCrouchHeight → (WriteCroucher)
  UpdateCarryOffsetProgress
  WriteIKTargets ────────► RCC graph inputs: Left/Right_Target, *_Target_Orientation, *_Pole_Target, *_Pole_Weight
  UpdateDebugMesh / DrawDebugTargetsIfEnabled
  WriteWalker ───────────► policy slot Walker: SetVx / SetVy / SetYawRate  (phase-dependent)
  WriteRotator ──────────► policy slot Rotator (ONLY in Phase.TurnAround; V2 turns write it themselves)
  WriteCroucher ─────────► policy slot croucher: SetVx(anti-drift) / 0 / 0 / SetTargetHeight
```

### 4.1 Policies

All three share the same 29 actuated joints, PD gains and action scales. The walker default pose has arms at shoulder pitch 0.2, roll ±0.2, elbow 0.6.

| Policy | Observations | Commands | Notes |
|---|---|---|---|
| walker | base lin/ang vel, projected gravity, joint pos rel, joint vel, last action, cmd | SetVx, SetVy, SetYawRate | Used for walking, standing, settling. |
| rotator | same | SetVx, SetVy, SetYawRate | Trained for **turning in place from standstill**. The code always sends vx=vy=0. |
| croucher | same + `base_pos_z` | + **SetTargetHeight** | Descriptor has **`freeze_joints`: all 14 arm joints**, so arms snap to `default_pos` on activation. `settle_control_ticks: 0`. |

The API used is `RobotControllerComponent` (as `RobotController m_Robot`):

- `SetPolicyActive(slot, bool)`
- `SetDrivenJoints(slot, string[] jointNames)` (the mask)
- `SetFloat(slot, command, value)`

### 4.2 Joint masks (the most important concept)

`ApplyAppropriateMask(policy)` picks a mask from two flags:

| OffHandCarrying | IKActive | Mask given to the policy | Meaning |
|---|---|---|---|
| true | any | `s_LegsAndWaist` (15 joints) | Both arms under IK (one carrying, one active or empty). |
| false | true | legs + waist + **off-hand** arm (22) | The active arm is IK; the policy keeps the other arm natural. |
| false | false | `s_AllJoints` (29) | Pure policy, whole body. |

- `ApplyOneArmIKMask()` applies this to **both Walker and Croucher**. This fixed a bug: during the cup B pickup, Croucher is active and still had a stale full-body mask, so it fought the IK and the arm sat at `default_pos`. Updating an inactive slot's mask is a harmless no-op.
- **Masks are only re-applied when a subroutine calls one of these functions.** Changing `IKActive` alone does **not** re-mask (for example, Closing flips `IKActive` off without re-masking; that's intentional, see §4.3).

### 4.3 IK targets (`WriteIKTargets`)

- **Active hand, `IKActive=true`:** target = `IKTargetPos`. Orientation = `OrientOffsetOverride ?? GetCurrentOrientationOffset()` (Pickup vs Hold by phase). With `UseBodyRelativeOrientation=true`, pelvis yaw is **added to offset.Y**, so the Euler offset is body-relative.
- **Active hand, `IKActive=false`** has three sub-cases:
  1. `OffHandCarrying` and a locomotion phase → pin the arm at a body-relative empty-hand pose.
  2. `OffHandCarrying` and not locomotion (for example, Closing) → write the **live palm** (hold still).
  3. Neither → write the **live palm** (zero residual; the policy owns the arm).
- **Off hand:** carrying → pinned at a body-relative carry pose. Not carrying → **live palm**.
- **Pole vectors:**
  - Weight = `PoleWeight` for the active hand in PolicyToIK, Reach, Holding, HoldDwell, Rotate, StepToDropoff and Retract. Otherwise 0.
  - The pole *target* uses the **reach** offset (elbow forward and outboard) during PolicyToIK and Reach, and the **carry** offset (elbow back and down) elsewhere.
  - This stops the IK from choosing a "chicken-wing" or through-the-chest elbow solution.
  - Pole targets are written every frame even when the weight is 0 (inert, but never stale).

### 4.4 Fingers (`UpdateFingers` / `StampHand`)

- `CloseCurrent` slews toward `CloseTarget` at `HandCloseSpeed` per second, and the same for the off hand.
- Per joint: `a.Value = amount × baseTarget × multiplier`.
  - Thumb yaw uses `ThumbYawAmount` (open) or `ThumbYawAmountHold` (gripping) × `ThumbYawClosed`.
  - Thumb chain: `ThumbPitchClosed` × {1.0, 1.6, 2.4}. The multipliers emulate the mimic coupling.
  - Fingers: `FingerClosed` × 1.0.
- Optional torque mode: position target = current qpos, plus `SetQfrcApplied(dof, HoldGripTorque × mult)`. It's off in this scene.
- Actuators are claimed in `OnCreate` with `a.IK = false`. Otherwise the motion graph owns them and your values are ignored.

### 4.5 Where positions come from

- Pelvis: `m_Mjs.GetBodyPoseMj(pelvisBodyId)` → `m_AxisMjToHz * posMj + m_SceneTranslation`. Rotation: `m_AxisMjToHz * rotMj`.
- Palm: `m_Mjs.GetSitePositionMj(siteId)` / `GetSitePoseMj`, with the same conversion.
- `m_AxisMjToHz = QuatInverse(m_Mjs.GetAxisHzToMj())`, `m_SceneTranslation = m_Mjs.GetSceneTranslation()`.
- Shoulders: `FindChildEntityByTag("left_shoulder_pitch_link")` entity world transforms. These are only used as anchors for body-relative offsets.
- **Yaw convention (Hazel Y-up):** `forward = pelvisRot * (1,0,0)`, `yaw = atan2(-forward.Z, forward.X)`. Heading to a target is `atan2(-dz, dx)`. Distances are **horizontal XZ only**.
- **Body-local offsets** are applied as `pelvisRot * offset`, where `pelvisRot` maps MuJoCo body axes into the Hazel world. So offsets are in **MuJoCo body axes: X = forward, Y = body-left, Z = up**. For right-hand mirroring, negate Y for positions and Z for Euler orientation offsets.

---

## 5. The task framework (`Tasks/`)

- **`Subroutine`** (abstract): `OnStart()`, `OnTick(ts) → Continue|Succeeded|Failed`, `OnComplete()` (runs on success, failure **and** cancel), `OnCancel(reason)`, `CanRunOn(robot)`.
  - `Status` goes Pending → Running → terminal. It's a plain C# object, not an engine script.
  - It reaches the robot script via `AssignedRobot!.As<GrabUpDownTurnTasks>()` (or `…V2`).
- **`Task`**: a named `Queue<Subroutine>` with a fluent `.Then(...)`.
- **`TaskComponent`** (C#, per robot): a `LinkedList<Task>`.
  - `Tick` starts the next subroutine when idle. **The next subroutine starts in the same frame the previous one finished** (Step → done → AdvanceToNextSubroutine → Start). The new one's first `OnTick` runs next frame.
  - `EnqueueTask` / `EnqueueSubroutine` append. **`EnqueueImmediate` prepends** a one-subroutine Task at the front.
  - It mirrors names into the engine `Hazel.TaskComponent`, which is cosmetic (shows in the inspector).
- **`TaskOrchestrator`** (engine script on its own entity): holds robots, ticks each active robot's `TaskComponent` in its `OnUpdate`. API: `RegisterRobot`, `StartExecution`, `EnqueueTask`, `EnqueueImmediate`, `ClearAll`.

### 5.1 Explicit chain vs auto-chain (and the prepend order trap)

- The cycle Task lists only the **branch points**.
- Linear sequences are auto-chained: each subroutine's `OnComplete` calls `o.EnqueueNext(next)` → `EnqueueImmediate` → prepend. The successor therefore runs **before** the rest of the explicit Task.
- Because it's a prepend, **when one `OnComplete` enqueues several successors, it must enqueue them in reverse run order**. See `WalkToSubroutine.OnComplete`: it pushes `Settle(main)`, `SetCrouch`, `SlowApproach`, `TurnToFaceTarget`, `Settle(pre)`, and they run as `Settle(pre) → TurnToFaceTarget → SlowApproach → SetCrouch → Settle(main)`.
- Auto-chains that would branch into V1 dropoff or turn-around behaviour are **gated** with `if (o.Mode == CycleMode.Bimanual) return;` (HoldDwell, Retract, BackOff, TurnAround). V2 relies on `Mode` being `Bimanual`, which is the **code default**; see gotcha G1.

### 5.2 Startup wiring (and why it's deferred)

1. `OnCreate` (robot):
   - Get the RCC and MJS.
   - Resolve and claim the finger actuators.
   - `SetPolicyActive(Walker, true)` and apply masks.
   - Write the initial walker and croucher commands.
   - Find the pelvis, wrist and shoulder entities by tag, and the palm site and pelvis body IDs.
   - Cache the axis conversion.
2. First `OnUpdate`:
   - `RestTargetPos = IKTargetPos = live palm`, `IKActive=false`, crouch height = standing, `InitialUseLeftHand = UseLeftHand`.
   - Then `WireOrchestratorAndEnqueueIfReady()`: scan `Scene.GetEntities()` for `.As<TaskOrchestrator>()`, `RegisterRobot`, `StartExecution`. It retries every frame until found.
   - **This is not done in `OnCreate`, because entt script-creation order is flat and undefined:** the orchestrator might not exist yet.
3. `HandleTriggers`: `StartGrab` is **saved as `true` in the scene**, so on the first frame (phase Idle) it enqueues `InitialResetBeforeFirstEpisode = [ResetSceneAndRestartSubroutine(forceReset:true)]`. Episode 0 therefore starts from the same post-reset state as every later episode. That subroutine calls `EnqueueGrabCycle()` when it finishes.
4. The `Reset` bool or `[Button("Reset Cycle")]` clears the queue and returns to Idle (it restores Walker if it was mid-turn).

---

## 6. Full execution timeline of `BinDropCycleV2`

Legend: **[E]** = explicit in `EnqueueGrabCycle`, **[A]** = auto-chained (prepended by the previous `OnComplete`).
Values are the ones saved in the scene (they differ from the code defaults; see §9).

### 6.0 Pre-roll: `ResetSceneAndRestartSubroutine(forceReset)`

1. Re-activate Walker, `ApplyOneArmIKMask`, phase Idle, `IKActive=false`.
2. Wait for the first `TimeSync.IsAtSyncPoint`.
3. Wait `ceil(PreResetQuietDuration × SyncHz)` = 5 sync ticks (0.5 s).
4. On a sync tick: `PerformReset`:
   - Disarm the watchdog.
   - `m_Mjs.ResetToInitialState(preserveTime:false)`.
   - Restore `UseLeftHand = InitialUseLeftHand`.
   - Zero all finger, carry, crouch, override and turn state; `IKActive=false`.
   - `ResetTransientControlState()` (also un-swaps Alt targets).
   - Re-snap `RestTargetPos/IKTargetPos` to the live palm.
   - Walker on, masks.
5. Wait 15 sync ticks (1.5 s `PostResetSettleDuration`), then on a sync tick call `EnqueueGrabCycle()`. That arms the watchdog (`CycleRunning=true`, `CycleElapsed=0`) and enqueues the cycle.

### 6.1 Cup A: walk, grab, turn around, bin (the task you described)

| # | Subroutine | Policy active / mask | Phase | What happens | Exit condition |
|---|---|---|---|---|---|
| 1 | **[E] ObserverBeginEpisode** | Walker / all | (Idle) | On the first sync tick **strictly after** OnStart: lazily `Observer.StartRecording()` if not recording, then `Observer.BeginEpisode()`. | Done on that tick. |
| 2 | **[E] WalkTo** (`WalkTarget=Left_Walk`) | Walker / all 29 | WalkTo | `IKActive=false`, `CloseTarget=0`. WriteWalker: `vx=WalkToSpeed 0.55`, yaw rate = clamp(err × 2, ±1) if \|err\|>0.1 rad. Arms swing naturally; IK gets the live palm. | XZ dist to Left_Walk ≤ 0.15+0.10, or 20 s timeout. **OnComplete prepends Settle.** |
| 3 | [A] **Settle** | Walker / all | Settle | vx=0 (WriteWalker writes 0 in any phase without a case). Let the body stop. | 0.6 s → prepends PolicyToIK. |
| 4 | [A] **PolicyToIK** | Walker / legs+waist+right arm | PolicyToIK | `fwd` = horizontal direction pelvis→grab. End = `grab − fwd × LeftApproachOffset(0.11)` (or `ReachStartTarget` if wired). `PolicyToIKTargetSpeed=0.2` → IK target starts at the live palm (Y snapped to the end Y) and moves 0.2 m/s toward the end. `IKActive=true`, `ApplyOneArmIKMask()`. Reach pole on. | Palm within `ArrivalRadius 0.1` of the end, or 0.79 s. → prepends Reach (or SetCrouch+Reach if `CrouchAfterPolicyToIK`). |
| 5 | [A] **Reach** | same | Reach | Start = **inherited `IKTargetPos`** (no backward snap), end = `grab + fwd × LeftReachThrough(0)`, same Y. Lerp at `ReachSpeed 0.1` m/s. `EnableReachPush=true` → walker creeps `vx=0.15` while dist to the can > `EndStandDistance 0.2 + 0.05`. | Lerp reaches 1 → Closing. |
| 6 | [A] **Closing** | same (not re-masked) | Closing | `CloseTarget = HandCloseFinal 0.5`. Each tick `IKActive = CloseCurrent < 0.4 × 0.5`. Once fingers pass 20% closed, IK stops chasing the target: WriteIKTargets writes the live palm, so the arm freezes where it is and doesn't fight the wrap. | CloseCurrent ≥ 0.5 → Holding. |
| 7 | [A] **Holding** | same | Holding | `IKActive=true`, `CarryOffsetProgress=0`, `CloseTarget = HandCloseHold 0.65` (squeeze harder). Slide from the live palm → `ComputeHoldTarget()` (left shoulder + pelvisRot × (0.2, 0.1, -0.1)) at 0.12 m/s. Orient lerps Pickup (-90,10,0) → Hold (-90,-20,40). Carry pole on. | Slide done → HoldDwell. |
| 8 | [A] **HoldDwell** | same | HoldDwell | Pin at the hold target for 1.5 s (lets the can settle in the hand). `RightHoldDwellVx` only applies for the right hand. | 1.5 s. **Bimanual gate → no auto-chain.** |
| 9 | **[E] TurnToFaceBin** | **Walker OFF, Rotator ON** / legs+waist+right | Rotate | Compute a signed sweep to `Bin_Walk_Target` (shortest, about ±150°), **track unwrapped cumulative yaw progress**. yawRate = clamp(remaining × 2, ±`TurnYawRate 0.7`) × ramp(0→1 over 0.5 s). The arm is pinned at the hold pose every tick; fingers stay in grip mode (Phase.Rotate counts as gripping). | \|remaining\| < 0.1 rad or 12 s. OnComplete: rotator yaw 0, Rotator OFF, **Walker ON**, re-mask. |
| 10 | **[E] WalkToBin** | Walker / legs+waist+right | WalkTo | `WalkTargetOverride = Bin_Walk_Target`. `IKActive=true`, IK target = live hold pose each tick, orient override = Hold. WriteWalker steers there at 0.55 m/s. | Dist ≤ 0.25 or 20 s. |
| 11 | **[E] MoveToBinRelease** | same | MoveToDropoff → Release | `EnableBinReleaseSlide=true`: slide the IK target from the live palm → `Bin_Drop_Target` at 0.2 m/s (Hold orient kept). When the slide completes **and** (palm within 0.03 m **or** 5 s), set `CloseTarget=0` (fingers open, can falls). Pin there. | Released ≥ `ReleaseDuration 2 s` **and** `CloseCurrent ≤ 0.05`. → prepends Retract. |
| 12 | [A] **Retract** | same | Retract | Slide palm → hold pose at 0.3 m/s. Orient lerps from the current override (Hold) → Hold. | Done. **Bimanual gate → no auto-chain.** |
| 13 | **[E] IKToPolicyHandoff** (cup A) | same | (stays Retract) | **Doesn't change phase on purpose** (see G12). Glide the IK target from the palm → left shoulder + pelvisRot × `LeftHandoffOffsetFromShoulder (0.1,0.1,-0.1)` over 2.5 s. Then `IKActive=false`. | 2.5 s. |

After step 13 the can is in the bin and the left arm is back near its natural pose. **This is the complete "grab can, turn, bin it" behaviour.**

### 6.2 Cup B (right hand, low can, crouch) and loop

| # | Subroutine | Notes |
|---|---|---|
| 14 | [E] SwitchActiveHand | Swap the Close/OffClose slots, `UseLeftHand=false`, `OffHandCarrying = OffHandCloseTarget>0.1` (false here). IK target = live right palm, `IKActive=true`, **seed orient override = Pickup orient** (avoids the "arm shoots up" snap, G14). Re-mask walker. |
| 15 | [E] SwapAltTargets | Swap every `X`↔`XAlt` pair (Target, Dropoff, Walk, DropStart, Debug, ApproachAim, ReachStart). `m_TargetsSwapped` is toggled. |
| 16 | [E] TurnToFaceWalkTarget | Rotator, `IKActive=false` → full-body mask (arms swing). P-control on the live heading error to `Right_Walk`, ramp-in. Restores Walker. |
| 17 | [E] WalkTo(crouchOnArrival) | Walk to Right_Walk. OnComplete (`ExtendArmBeforeCrouch=false` in the scene) prepends: `Settle(no autochain) → TurnToFaceTarget → SlowApproach → SetCrouch(true) → Settle(autochain)`. |
| 17a | [A] Settle (pre) | Bleed forward velocity before the rotator (rotator hates momentum). |
| 17b | [A] TurnToFaceTarget | Rotator faces `ApproachAim` (=`Right_Can_Face_Target` after the swap). |
| 17c | [A] SlowApproach | Walker at 0.2 m/s with yaw correction until the pelvis is within 0.30+0.05 m of the aim (5 s timeout). |
| 17d | [A] SetCrouch(true) | Walker OFF, **Croucher ON**, mask croucher, `CrouchHeightGoal=0.45`. `CommandedCrouchHeight` ramps down at 0.1 m/s. While heading to crouch, croucher vx=+0.05 (anti-backward-drift). Hold 0.5 s (the ramp continues after). |
| 17e | [A] Settle → PolicyToIK → Reach → Closing → Holding → HoldDwell | Same as cup A but right hand. PolicyToIK uses `ReachStartTarget` (`Right_Can_Start_Target`). `RightApproachOffset 0.15`, `RightReachThrough 0.07`. `ApplyOneArmIKMask` updates **croucher's** mask too. Right hold pose moves Start (0.15,0,0) → End (0.4,-0.1,0) over 5 s of carry (arm extends forward while walking to the placement). |
| 18 | [E] SwapAltTargets | Swap back (restore authored slots). |
| 19 | [E] SetCrouch(false) | Goal = 0.75. **Croucher stays active while ascending** at 0.3 m/s. When the commanded height is within 5 mm of standing: Croucher OFF, Walker ON, re-mask. Then wait for the min hold. |
| 20 | [E] TurnToFaceBin(useAlt, dir=`AltBinTurnDirection`=2 → CounterClockwise) | Forced direction, because the near-180° turn otherwise flip-flops. |
| 21 | [E] WalkToBin(useAlt) | → Bin_Walk_Target_Alt, arm pinned at the (progressing) carry pose. |
| 22 | [E] MoveToBinPlacedown(useAlt) | Phase Placedown (walker vx 0). 0: palm→`Right_Drop`; 1a: →`Bin_Pre_Drop_Target_Alt` (Hold orient); 1b: →`Bin_Drop_Target_Alt` (orient lerps Hold→Release (-90,0,0), shortest-arc per Euler axis); 2: open fingers, pin (≥2 s and fingers open); 3: lift to `Right_Drop_Retract`. All legs at `PlacedownTransitionSpeed 0.1` m/s; each leg needs slide done and (within 0.03 m or 5 s). **No return-to-body lerp** (body-local carry pose is *forward* of the shoulder and would knock things over). |
| 23 | [E] IKToPolicyHandoff(useAlt) | 0.5 s glide to right shoulder + (0.25,-0.1,0.15), then IK off. |
| 24 | [E] TurnToFaceFinish | Rotator, cumulative sweep to Finish_Target, arms policy-owned. |
| 25 | [E] ObserverEndEpisode(success:true) | `Observer.EndCurrentEpisode(true)`. The session stays open. |
| 26 | [E] WalkToFinish | Walker, full body, to Finish_Target. |
| 27 | [E] StopAndIdle | Zero walker commands, Walker and Rotator OFF, phase Idle. |
| 28 | [E] ResetSceneAndRestart | `ResetAndRestartOnFinish=true` → same pre-roll as §6.0, which re-enqueues the cycle. **Infinite loop**, one episode per cycle. |

### 6.3 Watchdog

`CheckCycleWatchdog` (every frame): if `CycleRunning` and `CycleElapsed ≥ CycleTimeout (180 s)`:

1. `ClearAll` (cancels the current subroutine; its `OnComplete` still runs).
2. Enqueue `[ObserverEndEpisode(false), ResetSceneAndRestart(forceReset:true)]`.

Failed episodes are tagged in the dataset.

---

## 7. Subroutine reference (all 40 files)

**Used by V2**

| File | One-liner |
|---|---|
| WalkToSubroutine | Walker drives to `ActiveWalkTarget`; IK off. Auto-chains Settle (+ the crouch pre-sequence when `crouchOnArrival`). Can target `ActiveDropoff`. `interDropoffGated` self-skips. |
| SettleSubroutine | Stand still for `SettleDuration`. Optionally auto-chains PolicyToIK. |
| PolicyToIKSubroutine | Policy→IK arm handoff to the parking spot. Chains Reach (or SetCrouch→Reach). |
| ReachSubroutine | Linear IK slide parking→grab (+reach-through). Chains Closing. |
| ClosingSubroutine | Close fingers; drop IK once past the cutoff. Chains Holding. |
| HoldingSubroutine | Slide the grasped object to the body-relative chest carry pose; squeeze. Chains HoldDwell. |
| HoldDwellSubroutine | Wait at the carry pose. SingleHand mode: branch to dropoff. Bimanual: stop. |
| TurnToFaceBinSubroutine | Rotator turn toward a bin stance using cumulative unwrapped progress, optional forced direction. Arm pinned at hold. |
| WalkToBinSubroutine | Walker to a bin stance with IK holding the carry pose. |
| MoveToBinReleaseSubroutine | Slide to the release point (or not), open, wait. Chains Retract. |
| MoveToBinPlacedownSubroutine | Multi-waypoint gentle placement + release + lift-away. |
| RetractSubroutine | Slide back to the carry pose. SingleHand: branch BackOff/TurnAround/Idle. |
| IKToPolicyHandoffSubroutine | Glide to a body-local rest pose near the policy default, then IK off. |
| SwitchActiveHandSubroutine | Swap active/off hand incl. finger state; set OffHandCarrying; seed orient. |
| SwapAltTargetsSubroutine | Swap all primary/Alt target slots. |
| TurnToFaceWalkTargetSubroutine | Rotator P-control on the live heading error, arms free. |
| TurnToFaceTargetSubroutine | Same, aimed at `ActiveApproachAim`. |
| SlowApproachSubroutine | Slow walker push to within a distance of the aim. |
| SetCrouchSubroutine | Walker↔Croucher swap with a height ramp; stand-up swaps back only after the ramp finishes. |
| TurnToFaceFinishSubroutine / WalkToFinishSubroutine / StopAndIdleSubroutine | End-of-run choreography. |
| ResetSceneAndRestartSubroutine | Sync-aligned MuJoCo reset + C# state reset + re-enqueue. |
| ObserverBeginEpisode / ObserverEndEpisode | Dataset episode markers (sync-gated begin, lazy session start). |

**V1 only (bimanual carry-both / single-hand modes):** TurnAroundSubroutine (fixed-degree rotator turn using `WriteRotator` with a 0.2 floor on the slowdown factor), BackOffSubroutine (walk backwards), BimanualPostHoldSettleSubroutine (move the soon-to-be off hand from hold to side carry pose), StepToDropoffSubroutine, MoveToDropStartSubroutine, MoveToDropoffSubroutine, ReleaseSubroutine, RestartBimanualCycleSubroutine, ObserverStart/StopRecordingSubroutine.

**Dead:** `ApproachSubroutine.cs` (renamed to PolicyToIK), `StepForwardSubroutine.cs` (removed). Both are comment-only files and safe to delete.

---

## 8. Determinism and data recording

- `Observer` (namespace `Hazel.Data`) API: `StartRecording()`, `IsRecording`, `BeginEpisode()`, `EndCurrentEpisode(bool success)`, `StopRecording()`.
- The session is opened lazily at the first `BeginEpisode` and **never closed**. Every cycle appends one episode, and the watchdog tags failures.
- `TimeSync.IsAtSyncPoint`, `TimeSync.CurrentSyncIndex`, `TimeSync.SyncHz`:
  - The reset fires on a sync tick.
  - Settle counts whole sync ticks.
  - `BeginEpisode` fires on the first sync tick *strictly after* its OnStart.
  - Without this alignment, episode frame 0 lands in one of several phase "attractors" of the 30 Hz capture vs 50 Hz control clocks, and episodes aren't bit-identical.
- `ResetTransientControlState()` resets the phase-edge tracker, the phase timer, `LastVx`, **and un-swaps Alt targets**. Episodes always start with the authored hand↔target pairing.

---

## 9. Tuned values that made it work (scene vs code default)

These are the values saved in `CupCleaner.hscene`. **The code defaults are not the working values**, so copy these.

| Field | Code default | **Scene (working)** | Why it matters |
|---|---|---|---|
| WalkToStandDistance | 0.70 | **0.15** | Stance marker is placed exactly where the pelvis should stop. |
| WalkToSpeed | 0.50 | **0.55** | |
| LeftApproachOffset / Right | 0.30 / 0.30 | **0.11 / 0.15** | Park the palm close to the can. |
| LeftReachThrough / Right | 0.05 / 0.05 | **0 / 0.07** | |
| Left/RightGrabYOffset | -0.05 | **0 / 0** | Grab height is set by the marker instead. |
| ReachSpeed | 0.20 | **0.10** | A slow final approach avoids knocking the can. |
| ArrivalRadius | 0.05 | **0.10** | |
| PolicyToIKDuration | 0.8 | **0.79** | |
| PolicyToIKTargetSpeed | 0 (static) | **0.2** | A moving target gives a gentler arm extension. |
| EndStandDistance / EnableReachPush | 0.40 / false | **0.20 / true** | Walker creeps forward during Reach (vx 0.15). |
| HandCloseFinal / HandCloseHold | 0.85 / 0.95 | **0.5 / 0.65** | Partial close plus can friction 10 holds reliably without crushing or ejecting. |
| HandCloseSpeed | 1.5 | **0.8** | |
| Left/RightHoldOffsetFromShoulder | (0.25,±0.10,0.15) | **L (0.2,0.1,-0.1), R (0.15,0,0)** | |
| RightHoldOffsetFromShoulderEnd | (0.25,-0.10,0.15) | **(0.4,-0.1,0)** | Cup B carry extends forward over time. |
| Left/RightPalmOrientationOffset | (-90,±20,0) | **(-90,±10,0)** | |
| Left/RightHoldOrientationOffset | (-90,±20,0) | **(-90,-20,40) both** | |
| ThumbPitchClosed / ThumbYawClosed | 0.7 / 1.6 | **0.8 / 1.7** | |
| EnableDropoffStaging / EnableStepToDropoff | true / true | **false / false** | V1 features, off. |
| ReleaseDuration | 0.5 | **2.0** | Give the can time to fall clear before retracting. |
| DropoffArrivalTimeout | 2 | **5** | |
| TurnYawRate / TurnTimeout | 1.0 / 8 | **0.7 / 12** | Slower rotator turns are more stable carrying a can. |
| CrouchTargetHeight / CrouchTransitionSpeed / CrouchStandUpSpeed | 0.55 / 0.30 / 0.15 | **0.45 / 0.10 / 0.30** | |
| ExtendArmBeforeCrouch | true | **false** | Crouch first, then extend (croucher `freeze_joints` would erase an early arm extension). |
| EmptyHandPosOffset | (0.3,0.05,-0.2) | (0.3,-0.05,0.2) | |
| EnableBinReleaseSlide | false | **true** | Cup A slides to Bin_Drop_Target before release. |
| AltBinTurnDirection | 2 | 2 (CCW) | |
| StartGrab | false | **true** | Auto-start on Play. |
| LogEveryNFrames | 5 | 0 (off) | |
| DrawDebugTargets | true | false | Turn on while tuning (§11). |
| CycleTimeout | 180 | 180 | |

---

## 10. Gotchas and hard-won lessons

The code comments record the bugs each of these fixed.

**Framework and engine**

- **G1 – `Mode` is not serializable.** `public CycleMode Mode = CycleMode.Bimanual;` is an enum, and the scene serializer can't store enums. V2 **depends** on `Mode == Bimanual` to stop HoldDwell and Retract from auto-chaining into V1's dropoff/back-off/turn-around. If you change the default to `SingleHand`, V2 breaks. (For inspector-exposed choices, use `int` + `[Dropdown]` like `AltBinTurnDirection`.)
- **G2 – Don't find other scripts in `OnCreate`.** Script creation order is flat and undefined. Do cross-entity wiring on the first `OnUpdate` and retry until found.
- **G3 – `EnqueueImmediate` prepends.** When pushing several successors from one `OnComplete`, push them in reverse order. `OnComplete` also runs on **cancel and failure**, so check `Status == Succeeded` before chaining, but *always* restore policy state (Walker on) regardless of status.
- **G4 – The next subroutine's `OnStart` runs in the same frame the previous one finished.** State you set in OnComplete is immediately overwritten by the next OnStart. Design each OnStart to fully specify phase, IK, mask and policy.
- **G5 – Generated enum names depend on the scene name** (`CupCleanerPolicyIds`). Slot IDs must match the `Policies` list order and IDs in the RCC. Engine `PolicyIds.Walker/Rotator` are assumed to be slots 1 and 2.
- **G6 – Finger actuators must be claimed** (`actuator.IK = false`), or the motion graph overrides your values.
- **G7 – Renaming a public field loses its saved scene value.** Renaming a class or namespace unbinds the script. The comments in `GrabUpDownTurnTasks.cs` mention `GrabUpDownTurn.cs` and `f_grab_up_down_turn_tasks*.hscene`. Those are from the project this was ported from and **don't exist here**; the real scene is `CupCleaner.hscene`.

**Policies and locomotion**

- **G8 – Mask every IK-owned joint out of *every* policy that might be active** (`ApplyOneArmIKMask` does Walker **and** Croucher). A stale full-body mask on the active policy makes it fight the IK, and the arm sticks at the policy default.
- **G9 – Stop before switching Walker→Rotator.** The rotator was trained from standstill. Put a `Settle` in between, **ramp the yaw command in** over `TurnRampInDuration`, and keep vx=vy=0 on the rotator. Always set rotator yaw to 0 before deactivating it.
- **G10 – For near-180° turns, track unwrapped cumulative progress** against a fixed signed sweep chosen at OnStart. Don't use the live wrapped heading error, which flip-flops at ±π. Optionally force CW/CCW.
- **G11 – Crouching:**
  - Croucher's `freeze_joints` snaps the arms to `default_pos` at activation, which erases any IK reach done before it. So crouch **before** PolicyToIK (`ExtendArmBeforeCrouch=false`), or accept a double motion.
  - **Ramp `SetTargetHeight`** instead of stepping it.
  - Use **asymmetric speeds** (the policy "rockets up" when standing).
  - Add a small forward vx (0.05) while crouched to cancel backward COM drift.
  - **When standing up, keep Croucher active until the ramp reaches standing height, then swap to Walker.** Swapping immediately makes Walker's PD slam the legs straight ("springy stand-up").
- **G12 – Walking steers with yaw correction** (`yawRate = clamp(err × gain, ±max)` with a dead zone). Before long walks from a bad heading, **turn in place first** (TurnToFaceBin/WalkTarget). Otherwise the walker arcs into furniture (the pillars here).

**IK and arm**

- **G13 – Never leave a stale world-space IK target.** Whenever IK isn't doing work, write the **live palm** pose as the target. Otherwise, as the body walks, the IK drags the arm toward an old world point ("arm contorts during WalkTo" / "arm flies out the back after a turn").
- **G14 – Seed the orientation when an arm transitions policy→IK.** SwitchActiveHand sets `OrientOffsetOverride = Pickup orient`. Otherwise the phase picker returns Hold (chest-tuck) orient for an arm hanging at the hip, which is geometrically impossible, and "the arm shoots up in front".
- **G15 – Reach inherits `IKTargetPos` from PolicyToIK instead of reading the live palm.** If the palm lagged, reading the live palm would teleport the target backward ~20–30 cm.
- **G16 – Use pole vectors** during reach and carry (reach pole: elbow forward and outboard; carry pole: elbow back and down) to prevent chicken-wing or through-chest elbow solutions.
- **G17 – Stop IK during finger closure.** In Closing, IK turns off after 40% of the close, so the IK doesn't fight the contact wrap and push the can away.
- **G18 – Hand the arm back to the policy gently** (`IKToPolicyHandoff`): glide to a body-local rest pose close to the policy default, then set `IKActive=false`. Otherwise the policy PD snaps the arm. **Run it while the arm is still the *active* hand** (before SwitchActiveHand), because it reads `GetPalmWorldPosition()` for the active side. **Don't change `CurrentPhase` in it.** Changing Retract→Release would flip the orientation picker from Hold to Pickup mid-glide (a wrist snap).
- **G19 – Keep orientation continuous across phases.** Subroutines that change pose lerp `OrientOffsetOverride` (Pickup↔Hold↔Release) and clear it in OnComplete. Retract starts from whatever override is live (`?? Pickup`) so there's no Hold→Pickup snap. Placedown lerps Euler angles with a **shortest-arc per axis**.
- **G20 – Body-local offsets are in MuJoCo body axes** (X fwd, Y left, Z up). World is Hazel (Y up). Mirror right-hand positions by negating Y, and orientation Euler offsets by negating Z (where auto-mirrored). `Left/Right*HoldOffset` fields are **not** auto-mirrored.
- **G21 – The placedown has no return-to-body lerp.** A body-local carry pose is *in front of* the shoulder, so lerping back to it while still facing the bin pushes the palm forward over the drop point and knocks things over.
- **G22 – Read poses from MuJoCo** (`GetBodyPoseMj`, `GetSitePositionMj`) with the axis and translation conversion, not from entity transforms of simulated links.

**Grasping and physics**

- **G23 – Put the grab marker as a child of the object** (`Left_Can_Target` under `Left_Can`) so it moves if the can is nudged. **A child's local position is scaled by the parent's scale** (local -1.10 × 0.07 = -7.7 cm).
- **G24 – Light, high-friction objects** (density 5, friction 10) plus a partial close (0.5→0.65) grasp reliably without torque mode. Increase `ReleaseDuration` so the object clears the fingers before retracting.
- **G25 – Concave receptacles need fine VHACD** (bin voxel resolution 16M) or the opening is sealed by the convex hull.

**Running and looping**

- **G26 – `StartGrab=true` is saved in the scene** so Play auto-starts. The first episode is preceded by the same reset+settle as the others.
- **G27 – `ResetToInitialState` resets MuJoCo only.** You must reset every piece of C# state yourself (hand, carry, finger values, overrides, crouch, swapped slots, rest target) and re-arm Walker, or the next cycle starts corrupted (for example, alternating starting hands).
- **G28 – Keep a policy active during quiet and reset windows.** With all policies off, the robot collapses.
- **G29 – Every waiting subroutine has a timeout**, and the whole cycle has a 180 s watchdog that logs the episode as failed and resets.
- **G30 – Slide-type subroutines use `progress += speed × ts / distance`** (constant m/s regardless of distance) and require both *slide complete* **and** (*palm arrived* **or** *timeout*). The IK target might be reached before the palm is.

---

## 11. Debugging playbook

- Enable **`DrawDebugTargets`**. Colors:
  - yellow = active IK target, red = active palm
  - green = off-hand carry target, pink = off-hand palm
  - light blue = empty-hand pose, magenta = hold target
  - blue = pole targets (with lines from the shoulders), gray = shoulders
  - orange = pelvis + 0.4 m forward arrow
  - purple ring/post = TargetObject + effective grab point
  - red-orange = dropoff, cyan = DropStart
  - light-yellow ring = WalkTarget with radius = `WalkToStandDistance`
- Set `LogEveryNFrames` to 5 to get `phase / t / palmDist / robotDist / dropoffDist / vx / close / ikActive` lines. Every phase change logs `PHASE: A → B`. Every subroutine logs OnStart parameters and its exit reason (`reached / timedOut / arrived`). **`timedOut=true` in a log is your first clue** that a target is unreachable.
- MoveToBinPlacedown logs a 10 Hz heartbeat with the sub-phase, progress, palmDist, IK target and palm.
- Wire `DebugIKTarget` to a visible small mesh to see `IKTargetPos` in the viewport.
- The engine `TaskComponent` on the robot shows the pending task and subroutine names in the inspector.
- Use the runtime buttons **Start Grab Now** and **Reset Cycle**.
- If the arm won't move to the target, check in order:
  1. Is the mask excluding that arm on the **active** policy?
  2. Is `IKActive` true?
  3. Is the pole weight or pole target pulling it somewhere odd?
  4. Is the target outside the reach envelope (body too far away → adjust the stance marker)?

---

## 12. Recipe: building a new "walk → grab → carry → place" project

1. **Robot model.** Use a G1 MJCF with Inspire hands (or your robot). **Add palm sites** at the grasp center of each hand. Make sure finger `<position>` actuators exist and are named like the joints.
2. **Scene.**
   - Robot entity: `RobotControllerComponent` + `MujocoSceneComponent`.
   - Add the policies in this slot order: **1 walker, 2 rotator, 3 croucher (optional)**. Leave them inactive; the script activates them. Enable `ClampObservationForUnclaimedJoints`.
   - Assign a motion graph with two `LimbIK` nodes (root = `*_shoulder_pitch_link`, end effector = `*_palm_site`), exposing inputs named exactly `Left_Target`, `Left_Target_Orientation`, `Left_Pole_Target`, `Left_Pole_Weight` and the same for `Right_*`. Copying `g1_inspire_graph.hmograph` is easiest.
3. **Time settings.** Robot runner 50 Hz, MuJoCo 50 Hz with 10 substeps, OnUpdate priority before robot controller update. Add a Sync runner (10 Hz) and Capture runner (30 Hz) if recording.
4. **Objects.**
   - Give graspable objects a free `MujocoJointComponent`, a VHACD mesh collider, low density and high friction.
   - Receptacles: fine VHACD.
   - Tables: invisible box colliders.
5. **Markers.** Create invisible marker entities:
   - Stance markers on the floor where the pelvis should stop.
   - A grab marker as a **child** of each object at the palm contact point.
   - Optional aim or parking markers.
   - A bin stance marker and a release point above the opening.
   - A finish marker.
6. **Code.**
   - Copy `Tasks/` (framework + subroutines), `GrabUpDownTurnTasks.cs` and `GrabUpDownTurnTasksV2.cs`.
   - Put the `TaskOrchestrator` script on an empty entity and the V2 script on the robot.
   - **Fix `CupCleanerPolicyIds`** to your scene's generated enum name. Save the scene first so the enum is generated.
   - Consider renaming namespaces *before* wiring the scene (G7).
7. **Wire fields.** Copy the working values from §9 as a starting point.
8. **Compose the chain.** For just "grab one can and bin it", override `EnqueueGrabCycle()` in a V3 subclass:
   ```csharp
   Task cycle = new Task("GrabAndBin")
       .Then(new ObserverBeginEpisodeSubroutine())
       .Then(new WalkToSubroutine())            // auto: Settle→PolicyToIK→Reach→Closing→Holding→HoldDwell
       .Then(new TurnToFaceBinSubroutine())     // rotator, arm pinned at carry pose
       .Then(new WalkToBinSubroutine())
       .Then(new MoveToBinReleaseSubroutine())  // auto: Retract
       .Then(new IKToPolicyHandoffSubroutine())
       .Then(new TurnToFaceFinishSubroutine())
       .Then(new ObserverEndEpisodeSubroutine(success: true))
       .Then(new WalkToFinishSubroutine())
       .Then(new StopAndIdleSubroutine())
       .Then(new ResetSceneAndRestartSubroutine());
   CycleRunning = true; CycleElapsed = 0f; DroppingAltCup = false;   // arm the watchdog like the base does
   m_Orchestrator.EnqueueTask(this, cycle);
   ```
   Keep `Mode = Bimanual` (G1), or the auto-chains branch into V1 dropoff logic.
9. **Tune in this order:**
   1. Stance marker (where the pelvis stops).
   2. Approach offset or parking marker.
   3. Grab marker height and position.
   4. Reach speed.
   5. Close amounts.
   6. Hold offsets and orientations (use the magenta hold marker).
   7. Bin stance and release point.
   8. Handoff offset (make it close to the policy default arm pose: shoulder pitch 0.2, roll ±0.2, elbow 0.6).
   9. Turn speed.
10. **New behaviours** are new `Subroutine` subclasses. Follow the pattern:
    - OnStart: set `CurrentPhase`, `IKActive`, policy on/off and **mask**, seed `IKTargetPos` and orient.
    - OnTick: update the target, return Continue/Succeeded, with a timeout.
    - OnComplete: restore policies (always), clear overrides, chain on success only.
    - If a new phase needs walker commands, add a case to `WriteWalker`. If it should keep the grip, add it to `activeGripping` in `UpdateFingers`. For the pole or orient pick, add it to the lists in `WriteIKTargets` / `GetCurrentOrientationOffset`.

---

## 13. Known cruft and cleanup ideas

- `ApproachSubroutine.cs` and `StepForwardSubroutine.cs` are empty stubs; delete them.
- `Piper.cs` (Jenga arm demo) and `ContentVault/.../walker_Policy.cs` are compiled but unused. Move them into a `~` folder.
- Comments reference the non-existent `GrabUpDownTurn.cs` and `f_grab_up_down_turn_tasks*.hscene`.
- `StepForwardTolerance` is used as a generic arrival tolerance (misnamed).
- The `IKToPolicyHandoffSubroutine` OnStart log prints the offset chosen by `useAlt`. The actual end pose uses `UseLeftHand`, and these agree in V2's chain.
- `PolicyToIKSubroutine`'s class comment says a static target is required, but the working scene uses `PolicyToIKTargetSpeed=0.2` (moving target). The field comment is the accurate one.
- `CupCleanerPolicyIds` is hard-coded in the shared V1 code. Abstracting the croucher slot ID would make the scripts portable across scenes.
- Walker/rotator/croucher descriptors and ONNX files are duplicated in three places (Assets root, `policies/`, `ContentVault/`). The scene references one of each; keep track of which (§1).
