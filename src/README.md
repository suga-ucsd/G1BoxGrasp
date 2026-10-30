# G1BoxGrasp controller

One class, `G1Pipe.G1BoxGrasp`, split into partial-class files by topic. The engine finds it by that name, so files can move
freely between folders. Start with `G1BoxGrasp.cs`: `OnUpdate` runs one phase step per frame, and each phase lives in `Phases/`.

| Folder / file | What is in it |
|---|---|
| `G1BoxGrasp.cs` | Class overview, the `Phase` enum, `OnCreate`, and `OnUpdate`, which dispatches to the current phase |
| `Config/Settings.cs` | Every inspector field, grouped as in the editor (`[Group]`) |
| `Core/Arm.cs` | One arm's handles and per-frame state (`Side` = +1 left, −1 right) |
| `Core/Constants.cs` | Fixed constants: joint lists, policy command IDs, rates, gains, margins |
| `Core/State.cs` | Runtime state (all of it is reset between recorded episodes) |
| `Core/PhaseMachine.cs` | `Advance`, sub-steps, the progress watchdog (`Stalled`), `Fail`, `Recover` |
| `Phases/Setup.cs` | `Begin`: finds the robot, arms, boxes, conveyor and table; start pose |
| `Phases/Catch.cs` | Catch: the downhill hand waits in the sliding box's path, gives with it, centres it |
| `Phases/Approach.cs` | Walking up to a resting box, the reach check (Approach, Ready) |
| `Phases/Grasp.cs` | Grip planning, PreGrasp (hands in front, slide in), Squeeze |
| `Phases/Lift.cs` | DragIn and Lift |
| `Phases/Carry.cs` | Back off, turn, walk to the table, station hold, carry frame |
| `Phases/Place.cs` | Place, Release, Retract, the table spot |
| `Hands/Grip.cs` | Squeeze control, contact forces, taring, required force |
| `Hands/HandTargets.cs` | Grip frame, hand targets, wrist steering, finger poses, IK glides |
| `Body/Locomotion.cs` | Walker/rotator commands, walking and turning, the waist lock |
| `World/Boxes.cs` | Finding boxes, choosing one, box pose and speed, the conveyor |
| `World/Geometry.cs` | MuJoCo ↔ editor frames, pelvis frame, surfaces, small vector helpers |
| `Diagnostics/Logging.cs` | Status, grip and pre-grasp log lines |
| `Diagnostics/ArmTest.cs`, `WalkerTest.cs` | Test modes (inspector `ArmTest`, `WalkerTest`) |
| `Recording/Episodes.cs` | ACT data recording: episodes, box pool, reset, sidecar (`RecordEpisodes`) |

**Phase order** (catch path): Start → FindBox → Catch → PreGrasp → Squeeze → (DragIn) → Lift → PullIn → BackOff →
TurnToTable → WalkToTable → FaceTable → Place → Release → Retract. Any failure goes to Recover.

**Edit, check, run:** edit here, then run `./sync_engine.sh` from the repo root. It builds against the engine's DLLs and, only if
the build is clean, copies these files to `g1_sort/Assets/Scripts/Client/Source/G1BoxGrasp/`. The engine copy differs only
in `ArmReach` (0.65 there, 0.50 here). Then open the editor or press Play.

**Function map:** `g1_grasp_map.html` in the repo root shows every function with its calls, callers, engine API, settings,
state and failure checks. It is generated from this folder; refresh it with `python3 tools/make_map.py` after code changes.
