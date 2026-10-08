using System;
using System.Collections.Generic;
using Hazel;

namespace G1Pipe
{
    // Picks up each box resting on the conveyor with both hands, places it on the table to the left, and comes back for the next.
    // FindBox -> FaceBox -> WalkToBox -> FaceChute -> PreGrasp -> Squeeze -> Lift -> PullIn -> TurnToTable -> WalkToTable -> FaceTable -> Place -> Release -> Retract -> FindBox ...
    // The "Phases" checkboxes switch groups of phases off for testing. Every phase has a progress watchdog: a stall triggers a corrective
    // retry (step closer, re-squeeze, re-turn); when those run out the attempt fails (Recover) and the next box is tried; a box that fails
    // MaxBoxAttempts times is skipped until it moves.
    public class G1BoxGrasp : Entity
    {
        private enum Phase { Start, FindBox, FaceBox, WalkToBox, FaceChute, Ready, PreGrasp, Squeeze, Reposition, Holding, Lift, PullIn, TurnToTable, WalkToTable, FaceTable, Place, Release, Retract, Recover, Done, Failed }

        /// <summary>One arm. Side = +1 left, -1 right: mirroring the two arms is just this sign.</summary>
        private sealed class Arm
        {
            public readonly float Side;
            public readonly string Name, Input;        // joint/body prefix ("left") and IK input prefix ("LeftArm_")
            public readonly Identifier Target, Pole;   // cached IK input ids
            public uint Site = uint.MaxValue;          // grasp site = the LimbIK end effector
            public uint Shoulder = uint.MaxValue;      // shoulder pitch link = the LimbIK chain root
            public uint[] Geoms = new uint[0];         // hand collision geoms
            public float[] Base = new float[0];        // their contact force at tare time
            public float Gap, Raise, Force;            // air between palm and side face (m, - = squeezing), height above contact (m), tared force (N)
            public MujocoActuator[] Joints = new MujocoActuator[0];  // shoulder pitch, roll, yaw, elbow (IK), wrist roll, pitch, yaw (script)
            public float[] BendFrom = new float[0];    // joint ctrl when the ready-pose bend started (rad)
            public uint[] WristLink = new uint[0];     // the links those joints turn
            public float[] WristQ = new float[0];      // wrist ctrl targets (rad)
            public float PalmErr;                      // deg between the palm normal and the squeeze axis
            public MujocoActuator[] Hand = new MujocoActuator[0];    // thumb 1, thumb 2, index 0, middle 0 (script-owned)
            public bool[] IsThumb = new bool[0];       // which hand geoms are thumb links
            public float ThumbForce;                   // tared thumb contact force (N), on the box's front face
            public float Thumb, Curl;                  // thumb swing back from straight, index + middle curl (rad; right hand mirrored)
            public Vector3 Cmd, From, To;              // IK target actually sent; ends of the path segment being run (SendHands)
            public float T, Tau;                       // segment duration and time into it (s); Tau >= T: no segment running
            public bool Rising;                        // the segment climbs straight up out of a low spot (wrist held so the fingers don't swing low)

            public Arm(float side)
            {
                Side = side;
                Name = side > 0f ? "left" : "right";
                Input = side > 0f ? "LeftArm_" : "RightArm_";
                Target = new Identifier(Input + "Target_Position");
                Pole = new Identifier(Input + "Pole_Target");
            }
        }

        private static readonly string[] s_LegsAndWaistOnly = {
            "left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint",
            "left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
            "right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint",
            "right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
            "waist_yaw_joint","waist_roll_joint","waist_pitch_joint",
        };

        // Motion graph: Start stores the current pose and blends to the IK solution over Duration; after that targets are tracked every frame.
        private static readonly Identifier s_Duration = new Identifier("Duration"), s_Start = new Identifier("Start");

        // Walker/rotator command IDs: SetFloat(slot, id, value) writes one command channel.
        private const uint k_SetVx = 1u;       // forward speed (m/s)
        private const uint k_SetVy = 2u;       // sideways speed (m/s)
        private const uint k_SetYawRate = 3u;  // turn rate (rad/s)

        private const float k_YawGain = 2f, k_MaxYawRate = 1f;                                    // heading P-gain (rad/s per rad) and cap
        private const float k_NearRadius = 0.4f;                                                   // m, inside this a walk goal is stepped onto without turning
        private const float k_StepGain = 2f, k_MinStepSpeed = 0.1f;                                // stepping speed = clamp(distance * gain, min, WalkSpeed)
        private const float k_Settle = 0.6f;                                                       // s at zero velocity before a policy swap or a reach (G9)
        private const float k_DecelTime = 0.2f;                                                    // s of zero commands to an outgoing policy while still active (G9)
        private const float k_TurnRate = 0.7f, k_TurnRampIn = 0.5f;                                // rotator: max rad/s, ramp-in s
        private const float k_WalkProgress = 0.02f, k_TurnProgress = 0.02f, k_ReachProgress = 0.005f;   // m, rad, m: smallest gain that counts as progress
        private const float k_RestSpeed = 0.02f;                                                   // m/s, box counts as resting
        private const float k_DepthGain = 0.002f, k_MaxSqueezeRate = 0.02f, k_MaxSqueezeDepth = 0.03f;   // squeeze admittance: m/(N*s), m/s, m
        private const float k_ForceTolFrac = 0.35f, k_GripConfirm = 0.5f;                         // grip OK: both hands >= 65% of target for 0.5 s
        private const float k_TouchForce = 2f;                                                     // N, a hand this loaded is already at the box
        private const float k_LiftSpeed = 0.05f, k_SlipTol = 0.01f, k_DropTol = 0.05f;            // m/s, m, m
        private const float k_CarrySpeed = 0.1f;                                                   // m/s, box moves in the hands
        private const float k_PlaceHover = 0.05f, k_PlaceLowerSpeed = 0.03f;                       // m, m/s
        private const float k_OpenSpeed = 0.05f, k_ReleaseGap = 0.05f, k_ReleaseTime = 1.5f, k_RetractUp = 0.10f;   // m/s, m, s, m
        private const float k_Widen = 0.05f;                                                       // m, a hand landing on the box slides out at most this far
        private const float k_WristGain = 2f, k_WristRate = 1f;                                    // wrist servo: rad/s per rad of palm error, cap
        private const float k_ThumbFlare = 0.72f, k_ThumbRate = 1.5f;                              // rad thumb swung back (open); rad/s it swings
        private const float k_MaxCurl = 0.8f, k_CurlRate = 0.5f;                                   // rad index + middle curl at most into the side face; rad/s they curl while the grip is short
        private const float k_ThumbStagger = 0.018f;                                               // m, left hand raised 2x this over the right so the straight thumbs pass
        private const float k_Snap = 0.05f, k_LocalMove = 0.10f, k_SafeMargin = 0.10f;              // m: target tracked directly within this; shorter moves across go straight; travel height over belt and boxes
        private const float k_MinSegment = 0.5f, k_SeedBlend = 0.5f;                               // s shortest path segment; s IK re-seed blend (eases any change of arm pose)
        private const float k_PalmHalf = 0.044f, k_SiteOffset = 0.01f;                             // m (MJCF): palm half-height about the wrist axis; grasp site below (left) / above (right) it
        private const float k_StillSpeed = 0.05f, k_StillWait = 1f;                                // a reach starts once the pelvis is under this m/s (or after this many s)
        private const float k_ArmLength = 0.517f, k_Comfort = 0.85f;                               // m shoulder -> grasp site at full stretch (MJCF); share of it the grip is planned for
        private static readonly float[] s_Mirror = { 1f, -1f, -1f, 1f, -1f, 1f, -1f };             // right arm joint = left x this (pitch, roll, yaw, elbow, wrist roll, pitch, yaw; MJCF)
        private const float k_TopMargin = 0.02f, k_PlacedWait = 0.5f;                              // m grasp site stays under the box top; s a placed hand must stop improving
        private const float k_Disturb = 0.02f, k_LostDist = 0.25f;                                 // m box moves logged as a disturbance; m that count as the box gone
        private static readonly Vector3[] s_Axes = { new Vector3(1f, 0f, 0f), new Vector3(0f, 1f, 0f), new Vector3(0f, 0f, 1f) };   // MuJoCo body X, Y, Z

        [Group("Phases")]
        public bool WalkToBox = true;           // walk up to the box and square up to the conveyor (off: grasp from where the robot stands)
        [Group("Phases")]
        public bool LiftBox = true;             // lift after the grip (off: open the hands again right away)
        [Group("Phases")]
        public bool CarryToTable = true;        // turn and walk to the table (off: put the box back down where it was picked)
        [Group("Phases")]
        public bool PlaceBox = true;            // set the box down and let go (off: keep holding it)
        [Group("Phases")]
        public bool RepeatLoop = true;          // after a box or a failure, go for the next one (off: stop)

        [Group("Walk")]
        public float WalkSpeed = 0.3f;          // m/s
        [Group("Walk")]
        public float StandDistance = 0.33f;     // m, pelvis -> box centre when grasping (arm reach limit)
        [Group("Walk")]
        public float EdgeClearance = 0.15f;     // m, pelvis never closer than this to the conveyor's or table's near edge
        [Group("Walk")]
        public float ArriveTolerance = 0.10f;   // m, this close to a walk goal counts as arrived
        [Group("Walk")]
        public float WalkTimeout = 60f;         // s
        [Group("Walk")]
        public float TurnTolerance = 0.1f;      // rad, a turn is done within this
        [Group("Walk")]
        public float MinTurnRate = 0.2f;        // rad/s, smallest yaw rate sent while turning (rotator deadband)

        [Group("Box")]
        public Entity? ChuteEntity;                // conveyor the boxes arrive on (unit-cube mesh, local +Y up)
        [Group("Box")]
        public float ApproachGap = 0.03f;          // m of air between each palm and its side face while the hands come down (0 = touching)
        [Group("Box")]
        public float PalmOffset = 0.02f;           // m, grasp site -> palm face along the palm normal (MJCF: 0.017), so sites touch at width + 2 x this
        [Group("Box")]
        public float BeltClearance = 0.03f;        // m of air under the lower palm edge: hands grip this low on the box, and never lower over the conveyor
        [Group("Box")]
        public float FrontInset = 0.051f;          // m, grasp site this far behind the front face: the straight thumb's front surface lies on it (MJCF: 0.051)
        [Group("Box")]
        public float BoxFriction = 1f;             // hand-box mu (MuJoCo uses the larger of the two geoms' friction)

        [Group("Arms")]
        public Vector3 FoldOffset = new Vector3(0.15f, 0.15f, 0.20f);    // hand target while walking, from the pelvis: (forward, up, outward)
        [Group("Arms")]
        public Vector3 PoleOffset = new Vector3(-0.30f, 0.10f, 0.15f);   // elbow hint from the pelvis: (forward, up, outward)
        [Group("Arms")]
        public Vector3 CarryOffset = new Vector3(0.30f, 0.10f, 0f);      // box centre while carrying, from the pelvis: (forward, up, left)
        [Group("Ready Pose")]
        public float ShoulderPitch = 0f;           // deg, left arm (right mirrored); - swings the arm forward and up
        [Group("Ready Pose")]
        public float ShoulderRoll = 0f;            // deg, + swings the arm out to the side
        [Group("Ready Pose")]
        public float ShoulderYaw = 0f;             // deg, + turns the forearm outward
        [Group("Ready Pose")]
        public float Elbow = 0f;                   // deg, 0 = bent 90 deg (forearm level, forward); + straightens it down, - bends it up
        [Group("Ready Pose")]
        public float WristRoll = 0f;               // deg, 0 = palms facing each other; + turns the palm down
        [Group("Ready Pose")]
        public float WristPitch = 0f;              // deg, + tips the fingers down
        [Group("Ready Pose")]
        public float WristYaw = 0f;                // deg, + swings the fingers outward
        [Group("Ready Pose")]
        public float BendDuration = 1.5f;          // s the arms take to bend into this pose (min-jerk); PreGrasp starts the moment it's reached

        [Group("Arms")]
        public float HandMoveSpeed = 0.2f;         // m/s average along each hand path segment (min-jerk: eases in and out, peaks at ~1.9x this)
        [Group("Arms")]
        public float ReachTolerance = 0.05f;       // m, grasp site this close to its target = in position
        [Group("Arms")]
        public float ApproachHeight = 0.10f;       // m, hands come down beside the box from this far above its top
        [Group("Arms")]
        public float LiftHeight = 0.08f;           // m
        [Group("Arms")]
        public float GripSafetyFactor = 2f;        // squeeze per hand = factor * m*g / (2*mu)
        [Group("Arms")]
        public float ArmReach = 0.50f;             // m, shoulder -> grasp site at full stretch (MJCF: 0.517); a grip lower than this below the shoulders is unreachable

        [Group("Place")]
        public Entity? PlaceTarget;                // the table top slab (unit-cube mesh, local +Y up); boxes go on its top face
        [Group("Place")]
        public float PlaceSpacing = 0.25f;         // m between slots along the table's near edge
        [Group("Place")]
        public float PlaceEdgeInset = 0.12f;       // m, slot line this far in from the table's near edge (and ends)
        [Group("Place")]
        public float PlaceStandDistance = 0.30f;   // m, pelvis -> slot when placing (arm reach limit)

        [Group("Recovery")]
        public float StallTime = 3f;               // s without progress (hand error, distance, turn error, grip) before a phase counts as stalled
        [Group("Recovery")]
        public float PhaseTimeout = 15f;           // s, hard cap on one try of a reach, squeeze, turn or step (walks use WalkTimeout)
        [Group("Recovery")]
        public int MaxBoxAttempts = 2;             // failed attempts before a box is skipped (until it moves)
        [Group("Recovery")]
        public int MaxRepositions = 3;             // closer stand points tried per grasp and per place
        [Group("Recovery")]
        public float RepositionStep = 0.04f;       // m closer per failed reach
        [Group("Recovery")]
        public float RepositionTolerance = 0.03f;  // m, stepping goal tolerance (and the least move worth stepping for)
        [Group("Recovery")]
        public float MinStandDistance = 0.20f;     // m, pelvis never steps closer than this to the box / slot
        [Group("Recovery")]
        public int MaxSqueezeTries = 3;            // squeezes (reopen + re-centre between them) before the grip fails
        [Group("Recovery")]
        public int MaxWalkRetries = 2;             // re-turns toward a walk goal after a stalled walk
        [Group("Recovery")]
        public int MaxTurnTries = 2;               // a stalled turn is retried (with twice the minimum rate) up to this many tries

        [Group("Policy")]
        public uint WalkerSlotId = 1u;
        [Group("Policy")]
        public uint RotatorSlotId = 2u;


        [Group("Debug")]
        public float LogInterval = 1f;             // s between status lines (0 = off)

        private RobotControllerComponent? m_Robot;
        private MujocoSceneComponent? m_Mujoco;
        private readonly Arm[] m_Arms = { new Arm(1f), new Arm(-1f) };
        private readonly List<(uint Body, uint Geom, Vector3 Half)> m_Boxes = new List<(uint Body, uint Geom, Vector3 Half)>();   // every free box
        private int[] m_Fails = new int[0];                  // failed attempts per box
        private int m_Box = -1;                              // index of the box being handled
        private uint m_PelvisId = uint.MaxValue;
        private Quaternion m_MjToHz = Quaternion.Identity;   // MuJoCo -> Hazel axes
        private Vector3 m_MjOffset;                          // MuJoCo -> Hazel translation
        private Phase m_Phase = Phase.Start;
        private int m_Step;                                  // sub-step inside a phase
        private float m_PhaseTime, m_StepTime;               // s since the phase / sub-step started
        private float m_Best, m_SinceBest, m_WatchTime;      // progress watchdog: best metric, s since it improved, s since the watch started
        private uint m_Active;                               // locomotion policy slot driving the legs + waist
        private float m_GripLift;                            // m the grip is raised up the box so the arms can reach it
        private float m_TurnYaw;                             // heading the current turn aims at (fixed when it starts)
        private int m_TurnTries, m_WalkRetries, m_Repositions, m_SqueezeTries;   // retries used in the current turn / walk goal / reach / grip
        private float m_Reach;                               // m, current pelvis -> box (or slot) stand distance; shrinks on failed reaches
        private bool m_Retry;                                // Recover goes back for the next box (else stops)
        private Vector3 m_ChuteN, m_ChuteTop;                // conveyor surface normal and the centre of its top face
        private Vector3 m_Stand, m_Slot;                     // where the pelvis walks to (fixed per goal); where the box goes down
        private Vector3 m_Cmd;                               // last walk command (vx, vy, yaw rate), for the log
        private float m_ToGo, m_LogTimer;                    // distance left to the walk goal (m); time since the last status line

        // Grip frame: box centre, squeeze axis (robot left), forward axis. World coords, or pelvis coords (forward, up, left) while carrying.
        private Vector3 m_Center, m_Lat, m_Fwd;
        private bool m_Carrying;
        private float m_HalfW, m_HalfD, m_HalfH;             // box half extents along m_Lat, m_Fwd and up
        private float m_Lift, m_Boost = 1f, m_Y0;            // lift (m), force boost while slipping, palm-over-box height at grip
        private Vector3 m_PickAt, m_BoxAt;                   // box centre when the reach started; when last re-aligned to (disturbance reference)
        private float m_SafeY, m_Dt = 0.02f;                 // height (world) hands travel at, clear of the belt and boxes; frame time (s)

        // Components, and the walker on legs + waist from the first step so the robot stands (the rest waits for MuJoCo in Begin).
        protected override void OnCreate()
        {
            m_Robot = GetComponent<RobotControllerComponent>();
            m_Mujoco = GetComponent<MujocoSceneComponent>();
            if (m_Robot == null || m_Mujoco == null) Log.Error($"[G1] '{Tag}' needs a RobotControllerComponent and a MujocoSceneComponent - script does nothing");
            else SetLocomotion(WalkerSlotId);
        }

        // Runs the current phase; the robot stands still unless the phase walks or turns.
        protected override void OnUpdate(float ts)
        {
            if (m_Robot == null || m_Mujoco == null) return;
            m_Dt = ts;
            m_PhaseTime += ts;
            m_StepTime += ts;
            m_SinceBest += ts;
            m_WatchTime += ts;
            if (LogInterval > 0f && m_Phase != Phase.Start && m_PelvisId != uint.MaxValue && (m_LogTimer += ts) >= LogInterval) { m_LogTimer = 0f; LogStatus(); }
            Drive(0f, 0f, 0f);

            switch (m_Phase)
            {
                case Phase.Start: Begin(); break;
                case Phase.FindBox:
                    SendHands(Fold, false);
                    m_Box = RestingBox();
                    if (m_Box < 0) break;   // nothing to pick yet: the status line lists every box and why
                    m_Reach = StandDistance;
                    m_Repositions = m_SqueezeTries = m_WalkRetries = 0;
                    m_Stand = StandFor(ChuteEntity!, BoxCenter(m_Box), m_Reach, FaceOf(ChuteEntity!));
                    Log.Info($"[G1] picking box {m_Box} at {BoxCenter(m_Box)}, {Flat(BoxCenter(m_Box) - Pelvis()).Length():F2} m from the pelvis{(WalkToBox ? $", standing at {m_Stand}" : "")}");
                    if (WalkToBox) Advance(Phase.FaceBox);
                    else EnterReady();
                    break;
                case Phase.FaceBox:
                    SendHands(Fold, false);
                    if ((m_Step == 0 && Flat(m_Stand - Pelvis()).Length() < k_NearRadius) || Turn(YawOf(m_Stand - Pelvis()))) Advance(Phase.WalkToBox);
                    break;
                case Phase.WalkToBox:
                    SendHands(Fold, false);
                    if (WalkTo(m_Stand, ArriveTolerance, true)) Advance(Phase.FaceChute);
                    else if (Stalled(m_ToGo, k_WalkProgress, WalkTimeout)) WalkStalled(Phase.FaceBox);
                    break;
                case Phase.FaceChute:
                    SendHands(Fold, false);
                    if (Turn(YawOf(FaceOf(ChuteEntity!)))) EnterReady();
                    break;
                case Phase.Ready: StepReady(); break;
                case Phase.PreGrasp: StepPreGrasp(ts); break;
                case Phase.Squeeze: StepSqueeze(ts); break;
                case Phase.Reposition: StepReposition(ts); break;
                case Phase.Holding: Hold(ts); break;   // test end state: keep the grip
                case Phase.Lift: StepLift(ts); break;
                case Phase.PullIn:
                    if (Hold(ts) && MoveTowards(ref m_Center, CarryOffset, k_CarrySpeed * ts)) EnterTurnToTable();
                    break;
                case Phase.TurnToTable:
                    if (Hold(ts) && ((m_Step == 0 && Flat(m_Stand - Pelvis()).Length() < k_NearRadius) || Turn(YawOf(m_Stand - Pelvis())))) Advance(Phase.WalkToTable);
                    break;
                case Phase.WalkToTable:
                    if (!Hold(ts)) break;
                    if (WalkTo(m_Stand, ArriveTolerance, true)) Advance(Phase.FaceTable);
                    else if (Stalled(m_ToGo, k_WalkProgress, WalkTimeout)) WalkStalled(Phase.TurnToTable);
                    break;
                case Phase.FaceTable:
                    if (Hold(ts) && Turn(YawOf(FaceOf(PlaceTarget!)))) Advance(PlaceBox ? Phase.Place : Phase.Holding);
                    break;
                case Phase.Place: StepPlace(ts); break;
                case Phase.Release: StepRelease(ts); break;
                case Phase.Retract: StepRetract(); break;
                case Phase.Recover: StepRecover(); break;
                default: SendHands(Fold, false); break;   // Done / Failed: stand with the arms folded
            }
            if (m_Phase == Phase.Start) return;
            OrientWrists(ts);
            PoseHands(ts);
        }

        // First frames (MuJoCo isn't ready in OnCreate): find the pelvis and boxes, set up the arm IK (failing loudly on a missing part), read the conveyor, fold the arms.
        private void Begin()
        {
            m_MjToHz = m_Mujoco!.GetAxisHzToMj().Conjugate;
            m_MjOffset = m_Mujoco.GetSceneTranslation();
            m_PelvisId = m_Mujoco.GetBodyID("pelvis");
            if (ChuteEntity == null || PlaceTarget == null) { Fail("assign ChuteEntity (conveyor) and PlaceTarget (table top)", false); return; }
            FindBoxes();
            if (m_PelvisId == uint.MaxValue || m_Boxes.Count == 0)
            {
                if (m_PhaseTime > 2f) Fail(m_PelvisId == uint.MaxValue ? "no 'pelvis' body in the MuJoCo model" : "no free-moving MuJoCo boxes in the scene", false);
                return;   // scene bodies can join the model a few frames after start: retry
            }
            foreach (var b in m_Boxes) m_Mujoco.UnweldBody(b.Body);   // clear welds left by earlier versions of this script (welds default to persistent)
            m_Fails = new int[m_Boxes.Count];
            foreach (Arm a in m_Arms)
            {
                // The IK owns the shoulders and the elbow; the script owns the wrists (OrientWrists) and the thumb and finger bases (PoseHands).
                var acts = new List<MujocoActuator>();
                foreach (string j in new[] { "_shoulder_pitch_joint", "_shoulder_roll_joint", "_shoulder_yaw_joint", "_elbow_joint", "_wrist_roll_joint", "_wrist_pitch_joint", "_wrist_yaw_joint", "_hand_thumb_1_joint", "_hand_thumb_2_joint", "_hand_index_0_joint", "_hand_middle_0_joint" })
                {
                    MujocoActuator? act = m_Mujoco.GetActuator(a.Name + j);
                    if (act == null) { Fail($"no actuator '{a.Name + j}'", false); return; }
                    act.IK = !j.StartsWith("_wrist") && !j.StartsWith("_hand");
                    acts.Add(act);
                }
                a.Joints = acts.GetRange(0, 7).ToArray();
                a.Hand = acts.GetRange(7, 4).ToArray();
                a.WristQ = new[] { (float)a.Joints[4].Value, (float)a.Joints[5].Value, (float)a.Joints[6].Value };
                a.WristLink = Array.ConvertAll(new[] { "_wrist_roll_link", "_wrist_pitch_link", "_wrist_yaw_link" }, l => m_Mujoco.GetBodyID(a.Name + l));
                a.Site = m_Mujoco.GetSiteID(a.Name + "_grasp_site");
                a.Shoulder = m_Mujoco.GetBodyID(a.Name + "_shoulder_pitch_link");
                if (a.Site == uint.MaxValue || a.Shoulder == uint.MaxValue || Array.IndexOf(a.WristLink, uint.MaxValue) >= 0) { Fail($"no site '{a.Name}_grasp_site' or body '{a.Name}_shoulder_pitch_link' / '{a.Name}_wrist_*_link'", false); return; }
                m_Robot!.SetInputBool(new Identifier(a.Input + "Solve_Orientation"), false);   // position only
                m_Robot.SetInputBool(new Identifier(a.Input + "High_Accuracy"), true);
                m_Robot.SetInputFloat(new Identifier(a.Input + "Pole_Weight"), 1f);
                HandGeoms(a);
                a.Base = new float[a.Geoms.Length];
            }
            m_ChuteN = ChuteEntity.Transform.WorldRotationQuat * Vector3.Up;
            m_ChuteTop = TopOf(ChuteEntity);
            SetSafeY();
            if (ApproachGap > 0.1f) Log.Warn($"[G1] ApproachGap is {ApproachGap:F2} m: each palm aims that far outside the box (arms flung sideways) - expected 0.02-0.05");
            Log.Info($"[G1] ready: {m_Boxes.Count} boxes, hand geoms L={m_Arms[0].Geoms.Length} R={m_Arms[1].Geoms.Length}, pelvis at {Pelvis()}");
            SendHands(Fold, true);
            Advance(Phase.FindBox);
        }

        // Grip frame squared to the box, grip as high up it as the arms need (PlanGrip), then reach (Ready -> PreGrasp -> Squeeze).
        private void EnterReady()
        {
            SetGripFrame(Heading());
            m_Carrying = false;
            m_Lift = 0f;
            m_Boost = 1f;
            m_PickAt = m_BoxAt = BoxCenter(m_Box);
            SetSafeY();
            PlanGrip();
            foreach (Arm a in m_Arms)
            {
                a.Gap = ApproachGap;
                a.Raise = 2f * m_HalfH - GripHeight(a) + ApproachHeight;
            }
            Log.Info($"[G1] reach: {GripInfo()} | front-bottom corners L {Corner(1f)} R {Corner(-1f)} | palms touch at width + 2 x PalmOffset {2f * (m_HalfW + PalmOffset):F3} m | hands travel at {m_SafeY:F2} m");
            Advance(Phase.Ready);
        }

        // Grip height from the live shoulders: raised up the box until each grasp site is within k_Comfort of the arm's reach, up to k_TopMargin
        // under its top; if even that isn't enough, says by how much the box sits too low. Logs the plan.
        private void PlanGrip()
        {
            m_GripLift = 0f;
            float reach = k_Comfort * k_ArmLength, need = 0f, maxLift = MaxGripLift();
            foreach (Arm a in m_Arms)
            {
                Vector3 d = GripPoint(a) - BodyPos(a.Shoulder);
                float across = Flat(d).Length();
                need = Math.Max(need, -d.Y - (float)Math.Sqrt(Math.Max(0f, reach * reach - across * across)));   // shoulder height over the grip beyond what the arm covers
            }
            m_GripLift = Math.Min(need, maxLift);
            Log.Info($"[G1] reach plan: shoulders {BodyPos(m_Arms[0].Shoulder).Y:F2} m, grip L {GripPoint(m_Arms[0]).Y:F2} R {GripPoint(m_Arms[1]).Y:F2} m (box {m_Center.Y - m_HalfH:F2}-{m_Center.Y + m_HalfH:F2}), grip raised {m_GripLift:F3} m");
            if (need > maxLift + 0.005f)
                Log.Warn($"[G1] box sits {need - maxLift:F2} m too low for the arms even gripping just under its top - raise the belt by that much (reaching anyway)");
        }

        // Grasp-site target beside the box at grip height (palm ApproachGap clear, not raised, not lifted).
        private Vector3 GripPoint(Arm a) => m_Center + m_Lat * (a.Side * (m_HalfW + PalmOffset + ApproachGap)) + m_Fwd * (FrontInset - m_HalfD) + Vector3.Up * (GripHeight(a) - m_HalfH);

        // How far the grip can still rise before a grasp site comes within k_TopMargin of the box top.
        private float MaxGripLift() => 2f * m_HalfH - k_TopMargin - Math.Max(GripHeight(m_Arms[0]), GripHeight(m_Arms[1])) + m_GripLift;

        // A hand is beside the box, judged from its live grasp site in the box frame: palm outside its side face by no more than the closing
        // range, thumb within 3 cm of the front face, palm overlapping the side face above the belt. Outputs those three numbers for the log.
        private bool Placed(Arm a, out float side, out float front, out float up)
        {
            Vector3 p = SiteWorld(a.Site) - m_Center;
            side = a.Side * Vector3.Dot(p, m_Lat) - m_HalfW - PalmOffset;
            front = Vector3.Dot(p, m_Fwd) + m_HalfD - FrontInset;
            up = p.Y + m_HalfH;
            return side > -0.015f && side < ApproachGap + k_Widen && Math.Abs(front) < 0.03f && up > PalmBelow(a) && up < 2f * m_HalfH - 0.01f;
        }
        private bool BothPlaced() => Placed(m_Arms[0], out _, out _, out _) && Placed(m_Arms[1], out _, out _, out _);

        // Inner loop, every frame of the reach: re-reads the box so the hands follow it (nudged, sliding) while follow is set, logs each
        // disturbance, and fails the attempt (next box) once the box has gone.
        private bool Track(bool follow)
        {
            Vector3 c = BoxCenter(m_Box);
            float gone = Flat(c - m_PickAt).Length(), moved = (c - m_BoxAt).Length();
            if (gone > k_LostDist || ChuteGap(m_Box) > 0.05f) { Fail($"box left the grasp: {gone:F2} m from where the reach started, {ChuteGap(m_Box):F2} m off the conveyor"); return false; }
            if (moved > k_Disturb)
            {
                Log.Warn($"[G1] box moved {moved:F3} m (now at {c}){(follow ? " -> hands follow it" : " - hands already touching, holding the frame")}");
                m_BoxAt = c;
            }
            if (follow) SetGripFrame(m_Fwd);
            return true;
        }

        // Ready: once the body is still (the walker keeps balancing), the shoulders and elbows come off the IK and all seven arm joints bend into the
        // inspector's ready pose (right arm mirrored) over BendDuration, min-jerk - elbows bent, hands up clear of the belt. The moment it's
        // reached the IK takes over again and PreGrasp starts.
        private void StepReady()
        {
            if (!Track(true)) return;
            if (m_Step == 0)
            {
                if (PelvisSpeed() > k_StillSpeed && m_StepTime < k_StillWait) return;   // arm moves on a swaying body make the walker step
                foreach (Arm a in m_Arms) a.BendFrom = Array.ConvertAll(a.Joints, j => (float)j.Value);
                ArmIK(false);
                NextStep();
                return;
            }
            float u = Math.Min(1f, m_StepTime / Math.Max(BendDuration, 0.01f)), k = u * u * u * (10f + u * (6f * u - 15f));   // min-jerk progress
            float[] pose = { ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, WristRoll, WristPitch, WristYaw };
            foreach (Arm a in m_Arms)
                for (int i = 0; i < 7; i++)
                {
                    float goal = pose[i] * Mathf.Deg2Rad * (a.Side > 0f ? 1f : s_Mirror[i]);
                    a.Joints[i].Value = a.BendFrom[i] + (goal - a.BendFrom[i]) * k;
                }
            if (u < 1f) return;
            ArmIK(true);
            foreach (Arm a in m_Arms)
                for (int i = 0; i < 3; i++) a.WristQ[i] = (float)a.Joints[4 + i].Value;   // the wrist servo carries on from the pose
            SendHands(HandTarget, true);
            Advance(Phase.PreGrasp);
        }

        // Shoulders and elbows to the IK (true), or to the script for the ready-pose bend (false).
        private void ArmIK(bool on)
        {
            foreach (Arm a in m_Arms)
                for (int i = 0; i < Math.Min(4, a.Joints.Length); i++) a.Joints[i].IK = on;
        }

        // Over the corners the contact sensing is tared and the hands come straight down beside the box, ApproachGap clear of it (a hand that lands
        // on it slides outward). Grasp (Squeeze) once both are within ReachTolerance - or beside the box faces (Placed) and no longer improving.
        // Hands that stall short adapt (GraspOutOfReach).
        private void StepPreGrasp(float ts)
        {
            if (!Track(true)) return;
            SendHands(HandTarget, false);
            if (Gliding()) { ResetWatch(); return; }   // on its way along the planned path: that's progress
            if (m_Step == 1)
            {
                ReadForces();
                foreach (Arm a in m_Arms)
                    if (a.Force > k_TouchForce) a.Gap = Math.Min(a.Gap + k_OpenSpeed * ts, ApproachGap + k_Widen);
            }
            float err = Math.Max(HandError(m_Arms[0]), HandError(m_Arms[1]));
            bool stalled = err > ReachTolerance && Stalled(err, k_ReachProgress, PhaseTimeout);
            if (m_Step == 1 && (err <= ReachTolerance || (BothPlaced() && m_SinceBest > k_PlacedWait)))
            {
                if (err > ReachTolerance) Log.Info($"[G1] hands {err:F3} m from their targets but beside the box faces - grasping");
                Advance(Phase.Squeeze);
                return;
            }
            if (err > ReachTolerance)
            {
                if (stalled) GraspOutOfReach();
                return;
            }
            Tare();
            foreach (Arm a in m_Arms) a.Raise = 0f;
            SendHands(HandTarget, true);
            NextStep();
        }

        // Grasp: the thumbs swing onto the front face and index + middle curl into the side faces while the arms close in, each hand pushing
        // toward the other, until both hold the required force for GripConfirm s (the hands follow the box until the first touch). A grip that
        // stops building with a hand at its squeeze limit (palm gap and finger curl both maxed) is reopened and re-centred (MaxSqueezeTries).
        private void StepSqueeze(float ts)
        {
            if (!Track(Contact() < k_TouchForce)) return;
            SendHands(HandTarget, false);
            float req = RequiredForce(), min = (1f - k_ForceTolFrac) * req;
            Squeeze(ts, req);
            bool stuck = false;
            foreach (Arm a in m_Arms)
            {
                if (a.Force < min) m_StepTime = 0f;
                stuck |= a.Force < min && a.Gap <= 0.001f - k_MaxSqueezeDepth && a.Curl >= k_MaxCurl;
            }
            if (!stuck) m_SinceBest = 0f;   // still closing or holding: that's progress
            if (m_StepTime >= k_GripConfirm)
            {
                Log.Info($"[G1] grip: L={m_Arms[0].Force:F1} R={m_Arms[1].Force:F1} N (need {req:F1} N), curl L={m_Arms[0].Curl:F2} R={m_Arms[1].Curl:F2} rad");
                m_Y0 = PalmMidY() - BoxCenter(m_Box).Y;
                m_Slot = m_Center - Vector3.Up * m_HalfH;   // pick spot (bottom centre): where the box goes back when not carried to the table
                m_Repositions = 0;
                Advance(LiftBox ? Phase.Lift : PlaceBox ? Phase.Release : Phase.Holding);
            }
            else if (m_SinceBest > StallTime || m_WatchTime > PhaseTimeout) RetrySqueeze(req);
        }

        // Grip didn't build: reopen (fingers ease straight, thumbs back), re-measure the box and re-centre the hands beside it, then grasp again;
        // fail after MaxSqueezeTries.
        private void RetrySqueeze(float req)
        {
            string why = $"grip L={m_Arms[0].Force:F1} R={m_Arms[1].Force:F1} N of {req:F1} N, gap L={m_Arms[0].Gap:F3} R={m_Arms[1].Gap:F3} m";
            if (++m_SqueezeTries >= MaxSqueezeTries) { Fail($"grip too weak after {MaxSqueezeTries} squeezes: {why}"); return; }
            Log.Warn($"[G1] {why} -> reopening and re-centring on the box ({m_SqueezeTries}/{MaxSqueezeTries - 1})");
            SetGripFrame(m_Fwd);
            foreach (Arm a in m_Arms) a.Gap = ApproachGap;
            Advance(Phase.PreGrasp);
            NextStep();                    // straight to "beside the box": the hands just open and re-centre at grip height
            SendHands(HandTarget, true);   // after the phase change, so the reopen eases along a path instead of jumping like a grip target
        }

        // Hands stopped short: grip higher up the box if the worse hand is below its target and there's box left; else fail, stating the shortfall.
        private void GraspOutOfReach()
        {
            Arm w = HandError(m_Arms[0]) >= HandError(m_Arms[1]) ? m_Arms[0] : m_Arms[1];
            Vector3 miss = HandTarget(w) - SiteWorld(w.Site);
            float along = Vector3.Dot(Flat(miss), m_Fwd), below = -miss.Y, room = MaxGripLift() - m_GripLift;
            string why = $"{w.Name} hand short by {miss.Length():F3} m ({along:F3} along the grip axis, {below:F3} below)";
            ResetWatch();
            if (below > 0.01f && room > 0.005f)
            {
                float lift = Math.Min(below, room);
                m_GripLift += lift;
                foreach (Arm a in m_Arms) if (a.Raise > 0f) a.Raise -= lift;   // over-the-corner targets stay put
                Log.Warn($"[G1] {why} -> gripping {m_GripLift:F3} m higher up the box");
                return;
            }
            Fail($"{why}: no higher grip left (grip raised {m_GripLift:F3} m) - raise the belt or move the robot");
        }

        // Next stand point m_Reach in front of p along the grip axis, a little closer each failed try (>= MinStandDistance, EdgeClearance
        // kept from the surface edge). False if the repositions are used up or the new point isn't a real move.
        private bool StepCloser(Entity surface, Vector3 p, string why)
        {
            float dist = Vector3.Dot(Flat(p - Pelvis()), m_Fwd);
            if (dist <= m_Reach + RepositionTolerance) m_Reach = Math.Max(MinStandDistance, dist - RepositionStep);   // already this close: a few cm closer
            Vector3 stand = StandFor(surface, p, m_Reach, m_Fwd);
            float move = Flat(stand - Pelvis()).Length();
            if (m_Repositions >= MaxRepositions || move < RepositionTolerance) return false;
            m_Stand = stand;
            Log.Warn($"[G1] {why} -> stepping {move:F2} m to stand {m_Reach:F2} m from it ({++m_Repositions}/{MaxRepositions})");
            return true;
        }

        // Steps onto the closer stand point without turning, holding the box, settles, then places again.
        private void StepReposition(float ts)
        {
            if (!Hold(ts)) return;
            if (m_Step == 0)
            {
                bool there = WalkTo(m_Stand, RepositionTolerance, false);
                if (!there && !Stalled(m_ToGo, k_WalkProgress, PhaseTimeout)) return;
                if (!there) Log.Warn($"[G1] stepping stalled {m_ToGo:F2} m short -> placing from here");
                NextStep();
            }
            else if (m_StepTime >= k_Settle) Advance(Phase.Place);
        }

        // Lift straight up, then hold the box in pelvis coords so it follows the body; next carry it, or put it back down where it was picked.
        private void StepLift(float ts)
        {
            if (!Hold(ts)) return;
            m_Lift = Math.Min(m_Lift + k_LiftSpeed * ts, LiftHeight);
            if (m_Lift < LiftHeight) return;
            m_Center = ToBody(m_Center + Vector3.Up * m_Lift - Pelvis());
            m_Lat = ToBody(m_Lat);
            m_Fwd = ToBody(m_Fwd);
            m_Lift = 0f;
            m_Carrying = true;
            Advance(CarryToTable ? Phase.PullIn : PlaceBox ? Phase.Place : Phase.Holding);
        }

        // Picks the nearest free table slot and where to stand for it, then turns toward that spot.
        private void EnterTurnToTable()
        {
            if (!FreeSlot(out m_Slot)) { Fail("the table is full", false); return; }
            m_Reach = PlaceStandDistance;
            m_WalkRetries = 0;
            m_Stand = StandFor(PlaceTarget!, m_Slot, m_Reach, FaceOf(PlaceTarget!));
            Log.Info($"[G1] placing at {m_Slot}, standing at {m_Stand}, {Vector3.Dot(m_Slot - m_Stand, FaceOf(PlaceTarget!)):F2} m from the slot");
            Advance(Phase.TurnToTable);
        }

        // Settle, switch the grip to world coords, move the box over its spot, then lower it until the surface holds it. If the hands
        // can't get it down, step closer and place again (MaxRepositions); else release where it is.
        private void StepPlace(float ts)
        {
            if (!Hold(ts)) return;
            Vector3 rest = m_Slot + Vector3.Up * m_HalfH;   // box centre resting on the surface
            if (m_Step == 0 && m_StepTime >= k_Settle)
            {
                m_Center = BodyPoint(m_Center);
                m_Lat = BodyDir(m_Lat);
                m_Fwd = BodyDir(m_Fwd);
                m_Carrying = false;
                NextStep();
            }
            else if (m_Step == 1 && MoveTowards(ref m_Center, rest + Vector3.Up * k_PlaceHover, k_CarrySpeed * ts)) NextStep();
            else if (m_Step == 2)
            {
                m_Center = m_Center - Vector3.Up * (k_PlaceLowerSpeed * ts);
                float above = BoxCenter(m_Box).Y - rest.Y;   // how far the box still is above its resting height
                if (above > 0.02f && m_Center.Y > BoxCenter(m_Box).Y - 0.02f) return;   // still coming down
                bool low = LowBy(rest.Y) > 0f;
                string why = $"box {above:F3} m above its spot, the hands can't go lower";
                if (above > 0.02f && !low && StepCloser(CarryToTable ? PlaceTarget! : ChuteEntity!, m_Slot, why))
                {
                    // Carry it in pelvis coords while stepping.
                    m_Center = ToBody(m_Center - Pelvis());
                    m_Lat = ToBody(m_Lat);
                    m_Fwd = ToBody(m_Fwd);
                    m_Carrying = true;
                    Advance(Phase.Reposition);
                    return;
                }
                Log.Info($"[G1] releasing {above:F3} m above the spot{(above <= 0.02f ? "" : low ? " - the spot is too low for the arms" : " - no closer stand point helps")}");
                Advance(Phase.Release);
            }
        }

        // Open both palms away from the box, then start lifting the hands clear.
        private void StepRelease(float ts)
        {
            foreach (Arm a in m_Arms) a.Gap = Math.Min(a.Gap + k_OpenSpeed * ts, k_ReleaseGap);
            SendHands(HandTarget, false);
            if (m_PhaseTime < k_ReleaseTime) return;
            m_Lift = k_RetractUp;
            SendHands(HandTarget, true);
            Advance(Phase.Retract);
        }

        // Hands up first, then back to the fold pose - never forward over the placed box (G21) - then look for the next box.
        private void StepRetract()
        {
            if (m_Step == 0)
            {
                SendHands(HandTarget, false);
                if (Gliding()) return;
                SendHands(Fold, true);
                NextStep();
            }
            else
            {
                SendHands(Fold, false);
                if (Gliding()) return;
                Log.Info($"[G1] placed {Flat(BoxCenter(m_Box) - m_Slot).Length():F3} m from its spot");
                Advance(RepeatLoop ? Phase.FindBox : Phase.Done);
            }
        }

        // After a failure: zero commands while the rotator is still active, hand back to the walker (G9), fold the arms, then go again or stop.
        private void StepRecover()
        {
            SendHands(Fold, false);
            if (m_Active == RotatorSlotId)
            {
                if (m_StepTime < k_DecelTime) return;
                SetLocomotion(WalkerSlotId);
                NextStep();
            }
            else if (!Gliding() && m_StepTime >= k_Settle) Advance(m_Retry && RepeatLoop ? Phase.FindBox : Phase.Failed);
        }

        // While holding: keep squeezing, track the hand targets, boost the force if the box slips, fail if it slides out.
        private bool Hold(float ts)
        {
            Squeeze(ts, RequiredForce());
            SendHands(HandTarget, false);
            float slip = PalmMidY() - BoxCenter(m_Box).Y - m_Y0;   // + = box sliding down between the palms
            if (slip > k_DropTol) { Fail($"box slipped out of the hands ({slip:F3} m)"); return false; }
            if (slip > k_SlipTol) m_Boost = Math.Min(m_Boost + ts, 2f);
            return true;
        }

        // Squeeze per hand: the palm gap closes (admittance) and index + middle curl further in while the tared force is short of the target.
        private void Squeeze(float ts, float target)
        {
            ReadForces();
            foreach (Arm a in m_Arms)
            {
                a.Gap = Mathf.Clamp(a.Gap - Mathf.Clamp(k_DepthGain * (target - a.Force), -k_MaxSqueezeRate, k_MaxSqueezeRate) * ts, -k_MaxSqueezeDepth, ApproachGap + k_Widen);
                if (a.Force < target) a.Curl = Math.Min(a.Curl + k_CurlRate * ts, k_MaxCurl);
            }
        }

        // Tared contact force of each hand: palm and fingers (the squeeze on the side face) and thumb (on the front face) apart.
        private void ReadForces()
        {
            foreach (Arm a in m_Arms)
            {
                a.Force = a.ThumbForce = 0f;
                for (int i = 0; i < a.Geoms.Length; i++)
                {
                    float f = Math.Max(0f, m_Mujoco!.GetGeomContactForce(a.Geoms[i]) - a.Base[i]);
                    if (a.IsThumb[i]) a.ThumbForce += f;
                    else a.Force += f;
                }
            }
        }

        // Total tared contact of both hands, thumbs included.
        private float Contact() => m_Arms[0].Force + m_Arms[1].Force + m_Arms[0].ThumbForce + m_Arms[1].ThumbForce;

        // Stores each hand geom's resting contact force so constant self-contact isn't read as grip.
        private void Tare()
        {
            foreach (Arm a in m_Arms)
                for (int i = 0; i < a.Geoms.Length; i++)
                    a.Base[i] = m_Mujoco!.GetGeomContactForce(a.Geoms[i]);
        }

        // Nearest box that rests on the conveyor, can be reached from outside its edge, and hasn't failed MaxBoxAttempts times; -1 if none.
        private int RestingBox()
        {
            int best = -1;
            float bestDist = float.MaxValue;
            for (int i = 0; i < m_Boxes.Count; i++)
            {
                float speed = BoxSpeed(i), dist = Flat(BoxCenter(i) - Pelvis()).Length();
                if (speed > 0.2f) m_Fails[i] = 0;   // a box on the move (dropped, respawned) gets fresh attempts
                if (Math.Abs(ChuteGap(i)) < 0.02f && speed < k_RestSpeed && (!WalkToBox || ReachShort(ChuteEntity!, BoxCenter(i), StandDistance) <= 0.05f) && m_Fails[i] < MaxBoxAttempts && dist < bestDist)
                {
                    best = i;
                    bestDist = dist;
                }
            }
            return best;
        }

        // Free-moving boxes: scene bodies (named lucky_body_<entity id>) with a box geom and a free joint.
        private void FindBoxes()
        {
            m_Boxes.Clear();
            for (uint g = 0; g < m_Mujoco!.ModelNgeom; g++)
            {
                if (!m_Mujoco.GetGeomDescriptor(g, out int body, out int type, out _, out _, out _, out Vector3 half) || type != 6) continue;   // 6 = mjGEOM_BOX
                uint b = (uint)body;
                if ((m_Mujoco.GetBodyNameByIndex(b) ?? "").StartsWith("lucky_body_") && m_Mujoco.GetBodyFreeJointIndex(b) < m_Mujoco.ModelNjnt)
                    m_Boxes.Add((b, g, half));
            }
        }

        // Nearest free slot on the table top: along its near edge, PlaceEdgeInset in from the edge and ends, PlaceSpacing apart. False if all taken.
        private bool FreeSlot(out Vector3 slot)
        {
            TransformComponent t = PlaceTarget!.Transform;
            Vector3 face = FaceOf(PlaceTarget), along = LeftOf(face);
            float halfLen = 0.5f * Math.Max(t.WorldScale.X, t.WorldScale.Z), halfWidth = 0.5f * Math.Min(t.WorldScale.X, t.WorldScale.Z);
            Vector3 line = TopOf(PlaceTarget) - face * (halfWidth - PlaceEdgeInset);
            float best = float.MaxValue;
            slot = line;
            for (float s = -halfLen + PlaceEdgeInset; s <= halfLen - PlaceEdgeInset; s += PlaceSpacing)
            {
                Vector3 p = line + along * s;
                bool taken = false;
                for (int i = 0; i < m_Boxes.Count; i++)
                    taken |= i != m_Box && Flat(BoxCenter(i) - p).Length() < 0.5f * PlaceSpacing && Math.Abs(BoxCenter(i).Y - p.Y) < 0.3f;
                if (!taken && Flat(p - Pelvis()).Length() < best)
                {
                    best = Flat(p - Pelvis()).Length();
                    slot = p;
                }
            }
            return best < float.MaxValue;
        }

        // Collision geoms of one hand (wrist_yaw_link + hand links) for contact force sensing, and which of them are thumb links.
        private void HandGeoms(Arm a)
        {
            var geoms = new List<uint>();
            var thumb = new List<bool>();
            for (uint g = 0; g < m_Mujoco!.ModelNgeom; g++)
            {
                if (!m_Mujoco.GetGeomDescriptor(g, out int body, out _, out _, out int contype, out int conaffinity, out _) || (contype | conaffinity) == 0) continue;
                string name = m_Mujoco.GetBodyNameByIndex((uint)body) ?? "";
                if (name != a.Name + "_wrist_yaw_link" && !name.StartsWith(a.Name + "_hand_")) continue;
                geoms.Add(g);
                thumb.Add(name.StartsWith(a.Name + "_hand_thumb"));
            }
            a.Geoms = geoms.ToArray();
            a.IsThumb = thumb.ToArray();
        }

        // Grip frame from the box as it is now: centre, its own horizontal axis closest to near (the heading, then the last forward so it
        // never flips) as forward, squeeze axis to its left, half extents.
        private void SetGripFrame(Vector3 near)
        {
            m_Mujoco!.GetGeomPoseMj(m_Boxes[m_Box].Geom, out _, out Quaternion q);
            Quaternion r = m_MjToHz * q;
            float best = 0f;
            m_Fwd = near;
            foreach (Vector3 axis in new[] { r * Vector3.Right, r * Vector3.Up, r * Vector3.Forward })
            {
                float d = Vector3.Dot(Flat(axis), near);   // unnormalised, so a near-vertical axis never wins
                if (Math.Abs(d) <= best) continue;
                best = Math.Abs(d);
                m_Fwd = FlatDir(axis) * (d > 0f ? 1f : -1f);
            }
            m_Lat = LeftOf(m_Fwd);
            m_Center = BoxCenter(m_Box);
            m_HalfW = HalfExtent(m_Box, m_Lat);
            m_HalfD = HalfExtent(m_Box, m_Fwd);
            m_HalfH = HalfExtent(m_Box, Vector3.Up);
        }

        // Hand target at its front-bottom box corner: palm PalmOffset + Gap outside the side face, grasp site FrontInset behind the front face
        // (straight thumb across the front face, fingers along the side), GripHeight up, + m_Lift and Raise; while reaching for the box, never
        // lower than GripHeight over the conveyor surface under it.
        private Vector3 HandTarget(Arm a)
        {
            Vector3 c = m_Carrying ? BodyPoint(m_Center) : m_Center;
            Vector3 lat = m_Carrying ? BodyDir(m_Lat) : m_Lat, fwd = m_Carrying ? BodyDir(m_Fwd) : m_Fwd;
            Vector3 t = c + lat * (a.Side * (m_HalfW + PalmOffset + a.Gap)) + fwd * (FrontInset - m_HalfD) + Vector3.Up * (GripHeight(a) - m_HalfH + m_Lift + a.Raise);
            if (!m_Carrying && m_Phase is Phase.PreGrasp or Phase.Squeeze) t.Y = Math.Max(t.Y, SurfaceY(t) + GripHeight(a));
            return t;
        }

        // Grasp-site height over the box bottom that leaves BeltClearance under the palm's lower edge (the left hand 2 x k_ThumbStagger higher),
        // plus any raise from the reach plan.
        private float GripHeight(Arm a) => BeltClearance + PalmBelow(a) + (1f + a.Side) * k_ThumbStagger + m_GripLift;

        // How far the palm's lower edge hangs below the grasp site (hand upright): left 0.034 m, right 0.054 m.
        private static float PalmBelow(Arm a) => k_PalmHalf - a.Side * k_SiteOffset;

        // Front-bottom corner of the box on one side (world, from the grip frame).
        private Vector3 Corner(float side) => m_Center + m_Lat * (side * m_HalfW) - m_Fwd * m_HalfD - Vector3.Up * m_HalfH;

        // Thumbs flared back (open) except while gripping, when they swing straight across the front face; index + middle follow a.Curl while
        // gripping (Squeeze curls them in) and ease straight otherwise. Everything moves at a set rate - nothing snaps into the box.
        private void PoseHands(float ts)
        {
            bool grip = Gripping();
            foreach (Arm a in m_Arms)
            {
                if (a.Hand.Length < 4) continue;
                a.Thumb += Mathf.Clamp((grip ? 0f : k_ThumbFlare) - a.Thumb, -k_ThumbRate * ts, k_ThumbRate * ts);
                if (!grip) a.Curl = Math.Max(0f, a.Curl - 2f * k_CurlRate * ts);
                a.Hand[0].Value = -a.Side * a.Thumb;
                a.Hand[1].Value = 0.0;
                a.Hand[2].Value = a.Hand[3].Value = -a.Side * a.Curl;
            }
        }

        // Height of the conveyor's top plane under point p.
        private float SurfaceY(Vector3 p) => m_ChuteTop.Y - (m_ChuteN.X * (p.X - m_ChuteTop.X) + m_ChuteN.Z * (p.Z - m_ChuteTop.Z)) / m_ChuteN.Y;

        // Wrist servo: turns each hand toward its zero-pose frame on the grip axes - fingers (X) along forward, Y along the squeeze axis, so the
        // left palm (-Y) and right palm (+Y) face the box; heading axes when not gripping. Closed on the measured hand frame, whatever the IK does.
        // A hand still climbing out from low down keeps its wrist still (turning there would swing the fingers into the belt).
        private void OrientWrists(float ts)
        {
            if (m_Phase == Phase.Ready && m_Step == 1) return;   // the ready-pose bend drives the wrists
            bool grip = m_Phase >= Phase.Ready && m_Phase <= Phase.Release;
            Vector3 x = grip ? (m_Carrying ? BodyDir(m_Fwd) : m_Fwd) : Heading(), y = LeftOf(x), z = Vector3.Cross(x, y);
            foreach (Arm a in m_Arms)
            {
                if (a.Joints.Length < 7 || a.Rising || SiteWorld(a.Site).Y < a.Cmd.Y - k_Snap) continue;
                Quaternion r = BodyRot(a.WristLink[2]);
                Vector3 cy = r * s_Axes[1], w = (Vector3.Cross(r * s_Axes[0], x) + Vector3.Cross(cy, y) + Vector3.Cross(r * s_Axes[2], z)) * 0.5f;   // rotation error
                a.PalmErr = Mathf.Acos(Mathf.Clamp(Vector3.Dot(cy, y), -1f, 1f)) * Mathf.Rad2Deg;
                for (int i = 0; i < 3; i++)
                {
                    m_Mujoco!.GetActuatorCtrlRange((uint)a.Joints[4 + i].ActuatorID, out float lo, out float hi);
                    a.WristQ[i] = Mathf.Clamp(a.WristQ[i] + Mathf.Clamp(k_WristGain * Vector3.Dot(BodyRot(a.WristLink[i]) * s_Axes[i], w), -k_WristRate, k_WristRate) * ts, lo, hi);
                    a.Joints[4 + i].Value = a.WristQ[i];
                }
            }
        }

        // Folded hand target in front of the body.
        private Vector3 Fold(Arm a) => BodyPoint(Mirror(FoldOffset, a.Side));

        // Writes both hand targets and elbow poles. While gripping, or within k_Snap, a hand target is tracked directly. A farther one is reached
        // along a safe path of straight segments, each a min-jerk move (eases in and out) at HandMoveSpeed on average: a short move across goes
        // straight; a longer one first climbs straight up to m_SafeY if low, then crosses above the belt and boxes, then comes straight down.
        // newMove re-plans from the live hands and restarts the graph's blend (Start) from the current pose. No-op until the pelvis is known.
        private void SendHands(Func<Arm, Vector3> target, bool newMove)
        {
            if (m_PelvisId == uint.MaxValue) return;
            bool grip = Gripping();
            foreach (Arm a in m_Arms)
            {
                Vector3 goal = target(a);
                if (newMove) { a.Cmd = SiteWorld(a.Site); a.T = a.Tau = 0f; }
                if (grip || (a.Tau >= a.T && (goal - a.Cmd).Length() <= k_Snap))
                {
                    a.Cmd = goal;
                    a.T = a.Tau = 0f;
                    a.Rising = false;
                }
                else
                {
                    if (a.Tau >= a.T) PlanSegment(a, goal);
                    a.Tau = Math.Min(a.Tau + m_Dt, a.T);
                    float u = a.Tau / a.T;
                    a.Cmd = a.From + (a.To - a.From) * (u * u * u * (10f + u * (6f * u - 15f)));   // min-jerk position profile
                }
                m_Robot!.SetInputVector3(a.Target, a.Cmd);
                m_Robot.SetInputVector3(a.Pole, BodyPoint(Mirror(PoleOffset, a.Side)));
            }
            if (!newMove) return;
            m_Robot!.SetInputFloat(s_Duration, k_SeedBlend);
            m_Robot.SetInputTrigger(s_Start);
        }

        // Next straight segment toward goal. Pure vertical moves go straight. From below m_SafeY, a long move across or a big climb first rises
        // straight up to m_SafeY (wrist held). A move that ends lower first crosses at the current height, then the next segment comes straight
        // down. Anything else (short local moves, climbs above m_SafeY) goes straight.
        private void PlanSegment(Arm a, Vector3 goal)
        {
            float across = Flat(goal - a.Cmd).Length();
            a.Rising = across > 0.02f && a.Cmd.Y < m_SafeY - 0.005f && (across > k_LocalMove || goal.Y > a.Cmd.Y + k_LocalMove);
            a.From = a.Cmd;
            a.To = across <= 0.02f ? goal : a.Rising ? new Vector3(a.Cmd.X, m_SafeY, a.Cmd.Z) : goal.Y < a.Cmd.Y - 0.02f ? new Vector3(goal.X, a.Cmd.Y, goal.Z) : goal;
            a.T = Math.Max(k_MinSegment, (a.To - a.From).Length() / HandMoveSpeed);
            a.Tau = 0f;
        }

        // A hand is still running a path segment.
        private bool Gliding() => m_Arms[0].Tau < m_Arms[0].T || m_Arms[1].Tau < m_Arms[1].T;

        // The hands hold (or are closing on) the box: targets tracked directly, thumbs and fingers in their grip pose.
        private bool Gripping() => m_Phase is Phase.Squeeze or Phase.Holding or Phase.Lift or Phase.PullIn or Phase.TurnToTable or Phase.WalkToTable or Phase.FaceTable or Phase.Place or Phase.Reposition;

        // Safe travel height for the hands: k_SafeMargin over the conveyor top and the boxes on it.
        private void SetSafeY()
        {
            m_SafeY = m_ChuteTop.Y;
            for (int i = 0; i < m_Boxes.Count; i++)
                if (Math.Abs(ChuteGap(i)) < 0.05f) m_SafeY = Math.Max(m_SafeY, BoxCenter(i).Y + HalfExtent(i, Vector3.Up));
            m_SafeY += k_SafeMargin;
        }

        // Closed-loop walk to a point: steer at it while far (if allowed); otherwise step straight onto it (forward + sideways, no turning,
        // slowing near it) so the robot never circles it. True within tol.
        private bool WalkTo(Vector3 goal, float tol, bool steer)
        {
            Vector3 to = Flat(goal - Pelvis()), fwd = Heading();
            float err = Mathf.WrapToPi(YawOf(to) - YawOf(fwd));
            m_ToGo = to.Length();
            if (m_ToGo <= tol) return true;
            if (steer && m_ToGo > k_NearRadius) Drive(Math.Abs(err) < 0.6f ? WalkSpeed : 0f, 0f, Mathf.Clamp(err * k_YawGain, -k_MaxYawRate, k_MaxYawRate));
            else
            {
                float v = Mathf.Clamp(m_ToGo * k_StepGain, k_MinStepSpeed, WalkSpeed) / m_ToGo;
                Drive(v * Vector3.Dot(to, fwd), v * Vector3.Dot(to, LeftOf(fwd)), 0f);
            }
            return false;
        }

        // Walk made no progress: turn toward the goal again and resume (MaxWalkRetries), else fail the attempt.
        private void WalkStalled(Phase turn)
        {
            if (++m_WalkRetries > MaxWalkRetries) { Fail($"walk stalled {m_ToGo:F2} m from the goal after {MaxWalkRetries} re-turns"); return; }
            Log.Warn($"[G1] walk stalled {m_ToGo:F2} m from the goal -> turning toward it again ({m_WalkRetries}/{MaxWalkRetries})");
            Advance(turn);
        }

        // Turn in place with the rotator to the heading given when the turn starts: settle, swap in, ramped P-turn (at least MinTurnRate,
        // doubled on a retry after a stall), stop, swap back, settle (G9). True when done; a turn still stalled after MaxTurnTries fails.
        private bool Turn(float yaw)
        {
            if (m_Step == 0) m_TurnYaw = yaw;   // aim fixed once the turn starts, so body sway doesn't move it
            float err = Mathf.WrapToPi(m_TurnYaw - YawOf(Heading()));
            switch (m_Step)
            {
                case 0:   // settle on the walker; no turn needed if already facing
                    if (m_StepTime < k_Settle) return false;
                    if (Math.Abs(err) < TurnTolerance) return true;
                    SetLocomotion(RotatorSlotId);
                    m_TurnTries = 1;
                    break;
                case 1:   // turn, yaw rate ramped in from standstill
                    Drive(0f, 0f, Math.Sign(err) * Mathf.Clamp(Math.Abs(err) * k_YawGain, Math.Min(MinTurnRate * m_TurnTries, k_TurnRate), k_TurnRate) * Math.Min(1f, m_StepTime / k_TurnRampIn));
                    if (Math.Abs(err) <= TurnTolerance) break;
                    if (!Stalled(Math.Abs(err), k_TurnProgress, PhaseTimeout)) return false;
                    if (m_TurnTries >= MaxTurnTries) { Fail($"turn stalled {err * Mathf.Rad2Deg:F0} deg short after {MaxTurnTries} tries"); return false; }
                    Log.Warn($"[G1] turn stalled {err * Mathf.Rad2Deg:F0} deg short -> turning again harder ({++m_TurnTries}/{MaxTurnTries})");
                    m_StepTime = 0f;
                    ResetWatch();
                    return false;
                case 2:   // zero yaw while the rotator is still active, then hand back to the walker
                    if (m_StepTime < k_DecelTime) return false;
                    SetLocomotion(WalkerSlotId);
                    break;
                default:  // settle on the walker
                    return m_StepTime >= k_Settle;
            }
            NextStep();
            return false;
        }

        // Hands the legs + waist to the walker or the rotator, switching the other off; the arms stay with the IK (G8).
        private void SetLocomotion(uint slot)
        {
            m_Robot!.SetDrivenJoints(slot, s_LegsAndWaistOnly);
            m_Robot.SetPolicyActive(slot == WalkerSlotId ? RotatorSlotId : WalkerSlotId, false);
            m_Robot.SetPolicyActive(slot, true);
            m_Active = slot;
        }

        // Sends (vx, vy, yaw rate) to the active locomotion policy.
        private void Drive(float vx, float vy, float yawRate)
        {
            m_Robot!.SetFloat(m_Active, k_SetVx, vx);
            m_Robot.SetFloat(m_Active, k_SetVy, vy);
            m_Robot.SetFloat(m_Active, k_SetYawRate, yawRate);
            m_Cmd = new Vector3(vx, vy, yawRate);
        }

        // The only place the phase changes: resets the phase timers and the watchdog, and logs the transition.
        private void Advance(Phase next)
        {
            Log.Info($"[G1] {m_Phase} -> {next}");
            m_Phase = next;
            m_Step = 0;
            m_PhaseTime = m_StepTime = m_ToGo = 0f;
            ResetWatch();
        }

        // Next sub-step of the current phase (fresh watchdog).
        private void NextStep()
        {
            m_Step++;
            m_StepTime = 0f;
            ResetWatch();
        }

        // Progress watchdog: true once metric hasn't dropped by minGain for StallTime, or this watch has run longer than cap.
        private bool Stalled(float metric, float minGain, float cap)
        {
            if (metric < m_Best - minGain) { m_Best = metric; m_SinceBest = 0f; }
            return m_SinceBest > StallTime || m_WatchTime > cap;
        }
        private void ResetWatch()
        {
            m_Best = float.MaxValue;
            m_SinceBest = m_WatchTime = 0f;
        }

        // Status line: phase, pelvis, last walk command; the walk goal while moving; every box (and why it isn't picked) while
        // looking for one; the handled box; and the hands while they work.
        private void LogStatus()
        {
            Vector3 p = Pelvis();
            string s = $"[G1] {m_Phase}: pelvis ({p.X:F2}, {p.Y:F2}, {p.Z:F2}) yaw {YawOf(Heading()) * Mathf.Rad2Deg:F0} deg | cmd vx={m_Cmd.X:F2} vy={m_Cmd.Y:F2} yaw={m_Cmd.Z:F2}";
            if (m_Phase is Phase.FaceBox or Phase.WalkToBox or Phase.Reposition or Phase.TurnToTable or Phase.WalkToTable)
                s += $" | goal ({m_Stand.X:F2}, {m_Stand.Z:F2}), to go {m_ToGo:F2} m";
            if (m_Phase == Phase.FindBox)
                for (int i = 0; i < m_Boxes.Count; i++)
                    s += $" | box {i}: {Flat(BoxCenter(i) - p).Length():F2} m away, gap {ChuteGap(i):F3} m, speed {BoxSpeed(i):F2}"
                        + (WalkToBox ? $", out of reach by {Math.Max(0f, ReachShort(ChuteEntity!, BoxCenter(i), StandDistance)):F2} m" : "")
                        + (m_Fails[i] >= MaxBoxAttempts ? ", skipped until it moves" : "");
            else if (m_Box >= 0)
                s += $" | box {m_Box} at {BoxCenter(m_Box)}";
            if (m_Phase >= Phase.Ready && m_Phase <= Phase.Release)
            {
                s += $" | {GripInfo()}";
                foreach (Arm a in m_Arms)
                {
                    bool placed = Placed(a, out float side, out float front, out float up);
                    s += $" | {a.Name} err {HandError(a):F3} m, force {a.Force:F1} N, thumb {a.ThumbForce:F1} N, gap {a.Gap:F3} m, curl {a.Curl:F2} rad, palm {a.PalmErr:F0} deg off"
                        + $", box frame: palm {side:F3} out, thumb {front:F3} fwd, site {up:F3} up{(placed ? " (placed)" : "")}";
                }
            }
            Log.Info(s);
        }

        // Grip geometry: box CoG (a uniform box geom, so its centre), its size (width = across the squeeze axis), target and actual grasp-site separation
        // (palms touch the box at width + 2 x PalmOffset).
        private string GripInfo()
        {
            Vector3 c = BoxCenter(m_Box);
            float target = (HandTarget(m_Arms[0]) - HandTarget(m_Arms[1])).Length(), actual = (SiteWorld(m_Arms[0].Site) - SiteWorld(m_Arms[1].Site)).Length();
            float Under(Arm a) => SiteWorld(a.Site).Y - PalmBelow(a) - (c.Y - m_HalfH);   // palm's lower edge over the box bottom
            return $"box CoG ({c.X:F3}, {c.Y:F3}, {c.Z:F3}), {2f * m_HalfW:F3} wide x {2f * m_HalfD:F3} deep x {2f * m_HalfH:F3} high | hand gap target {target:F3} m, actual {actual:F3} m"
                + $" | palm edge over box bottom L {Under(m_Arms[0]):F3} R {Under(m_Arms[1]):F3} m";
        }

        // Logs why, counts the attempt against the box (skip = give up on it now), folds the arms (letting go of any box) and recovers;
        // retry = go back for the next box afterwards, else stop for good.
        private void Fail(string reason, bool retry = true, bool skip = false)
        {
            Log.Error($"[G1] failed in {m_Phase}: {reason}");
            if (retry && m_Box >= 0 && (skip || ++m_Fails[m_Box] >= MaxBoxAttempts))
            {
                m_Fails[m_Box] = MaxBoxAttempts;
                Log.Warn($"[G1] skipping box {m_Box} until it moves");
            }
            m_Retry = retry;
            ArmIK(true);   // in case it failed mid-bend
            Advance(Phase.Recover);
            SendHands(Fold, true);   // after the phase change, so hands that were gripping still leave along the safe path
        }

        // Surfaces (conveyor, table) are unit-cube meshes: centre of the top face, and the direction that faces one square
        // (across its long horizontal axis) from the robot's side.
        private static Vector3 TopOf(Entity e) => e.Transform.WorldTranslation + e.Transform.WorldRotationQuat * Vector3.Up * (0.5f * e.Transform.WorldScale.Y);
        private Vector3 FaceOf(Entity e)
        {
            TransformComponent t = e.Transform;
            Vector3 across = FlatDir(t.WorldRotationQuat * (t.WorldScale.X > t.WorldScale.Z ? Vector3.Forward : Vector3.Right));
            return Vector3.Dot(across, t.WorldTranslation - Pelvis()) >= 0f ? across : across * -1f;
        }

        // Pelvis spot to work on point p of a surface: reach behind p along dir (horizontal), pushed back so the pelvis stays at least
        // EdgeClearance outside the surface's near edge and the legs never hit it.
        private Vector3 StandFor(Entity surface, Vector3 p, float reach, Vector3 dir)
        {
            Vector3 face = FaceOf(surface), stand = p - dir * reach;
            float clear = Vector3.Dot(surface.Transform.WorldTranslation - stand, face) - 0.5f * Math.Min(surface.Transform.WorldScale.X, surface.Transform.WorldScale.Z);
            return clear >= EdgeClearance ? stand : stand - face * (EdgeClearance - clear);
        }

        // How much farther than reach the surface edge forces the robot to stand from p (> 0: p is out of reach by that much).
        private float ReachShort(Entity surface, Vector3 p, float reach) => Vector3.Dot(p - StandFor(surface, p, reach, FaceOf(surface)), FaceOf(surface)) - reach;

        // How far height y lies beyond ArmReach below the higher shoulder (> 0: out of reach from any stand point).
        private float LowBy(float y) => Math.Max(BodyPos(m_Arms[0].Shoulder).Y, BodyPos(m_Arms[1].Shoulder).Y) - y - ArmReach;

        // Box i from MuJoCo: centre (world), speed, clearance under it on the conveyor, and half extent along a world direction (support function of its box geom).
        private Vector3 BoxCenter(int i)
        {
            m_Mujoco!.GetGeomPoseMj(m_Boxes[i].Geom, out Vector3 p, out _);
            return m_MjToHz * p + m_MjOffset;
        }
        private float BoxSpeed(int i)
        {
            m_Mujoco!.GetBodyPoseMj(m_Boxes[i].Body, out _, out _, out Vector3 v, out _);
            return v.Length();
        }
        private float ChuteGap(int i) => Vector3.Dot(BoxCenter(i) - m_ChuteTop, m_ChuteN) - HalfExtent(i, m_ChuteN);
        private float HalfExtent(int i, Vector3 dir)
        {
            m_Mujoco!.GetGeomPoseMj(m_Boxes[i].Geom, out _, out Quaternion q);
            Quaternion r = m_MjToHz * q;
            Vector3 h = m_Boxes[i].Half;
            return Math.Abs(Vector3.Dot(dir, r * Vector3.Right)) * h.X + Math.Abs(Vector3.Dot(dir, r * Vector3.Up)) * h.Y + Math.Abs(Vector3.Dot(dir, r * Vector3.Forward)) * h.Z;
        }

        // Squeeze force per hand that holds the box: factor * m*g / (2*mu), boosted while slipping.
        private float RequiredForce() => GripSafetyFactor * m_Mujoco!.GetBodyMass(m_Boxes[m_Box].Body) * 9.81f / (2f * BoxFriction) * m_Boost;

        // Distance from a hand's grasp site to its target.
        private float HandError(Arm a) => (SiteWorld(a.Site) - HandTarget(a)).Length();

        // Average height of the two grasp sites.
        private float PalmMidY() => 0.5f * (SiteWorld(m_Arms[0].Site).Y + SiteWorld(m_Arms[1].Site).Y);

        // MuJoCo poses in Hazel world coordinates: axis swap + scene translation, as the reference project did (PLAN.md G22).
        private Vector3 SiteWorld(uint site) => m_MjToHz * m_Mujoco!.GetSitePositionMj(site) + m_MjOffset;
        private Vector3 BodyPos(uint body) { m_Mujoco!.GetBodyPoseMj(body, out Vector3 p, out _, out _, out _); return m_MjToHz * p + m_MjOffset; }
        private Quaternion BodyRot(uint body) { m_Mujoco!.GetBodyPoseMj(body, out _, out Quaternion q, out _, out _); return m_MjToHz * q; }

        // Pelvis position (world) and horizontal heading, from MuJoCo.
        private Vector3 Pelvis() => BodyPos(m_PelvisId);
        private float PelvisSpeed() { m_Mujoco!.GetBodyPoseMj(m_PelvisId, out _, out _, out Vector3 v, out _); return v.Length(); }
        private Vector3 Heading() => FlatDir(BodyRot(m_PelvisId) * Vector3.Right);

        // Pelvis coords (forward, up, left) <-> world, using the pelvis heading only so the grip ignores body sway.
        private Vector3 BodyDir(Vector3 b) => Heading() * b.X + Vector3.Up * b.Y + LeftOf(Heading()) * b.Z;
        private Vector3 BodyPoint(Vector3 b) => Pelvis() + BodyDir(b);
        private Vector3 ToBody(Vector3 v) => new Vector3(Vector3.Dot(v, Heading()), v.Y, Vector3.Dot(v, LeftOf(Heading())));

        // Engine is Y-up: yaw = atan2(-z, x), left of a heading = (z, 0, -x); "flat" drops the height.
        private static float YawOf(Vector3 d) => Mathf.Atan2(-d.Z, d.X);
        private static Vector3 LeftOf(Vector3 f) => new Vector3(f.Z, 0f, -f.X);
        private static Vector3 Flat(Vector3 v) => new Vector3(v.X, 0f, v.Z);
        private static Vector3 FlatDir(Vector3 v) => Flat(v).Normalized();

        // (forward, up, outward) offset of one arm: the right arm mirrors the outward part.
        private static Vector3 Mirror(Vector3 o, float side) => new Vector3(o.X, o.Y, side * o.Z);

        // Moves p toward goal by at most step; true once there.
        private static bool MoveTowards(ref Vector3 p, Vector3 goal, float step)
        {
            Vector3 d = goal - p;
            float len = d.Length();
            p = len <= step ? goal : p + d * (step / len);
            return len <= step;
        }
    }
}
