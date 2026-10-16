using System;
using System.Collections.Generic;
using Hazel;

namespace G1Pipe
{
    // Picks up each box resting on the conveyor with both hands, places it on the table to the left, and comes back for the next.
    // FindBox -> FaceBox -> WalkToBox -> FaceChute -> Approach -> Ready -> PreGrasp -> Squeeze -> Lift -> PullIn -> TurnToTable -> WalkToTable -> FaceTable -> Place -> Release -> Retract -> FindBox ...
    // The "Phases" checkboxes switch groups of phases off for testing. Every phase has a progress watchdog: a stall triggers a corrective
    // retry (step closer, re-squeeze, re-turn); when those run out the attempt fails (Recover) and the next box is tried; a box that fails
    // MaxBoxAttempts times is skipped until it moves.
    public class G1BoxGrasp : Entity
    {
        private enum Phase
        {
            Start,
            FindBox,
            FaceBox,
            WalkToBox,
            FaceChute,
            Approach,
            Ready,
            PreGrasp,
            Squeeze,
            Reposition,
            Holding,
            Lift,
            PullIn,
            TurnToTable,
            WalkToTable,
            FaceTable,
            Place,
            Release,
            Retract,
            Recover,
            WalkerTest,
            Done,
            Failed,
        }

        /// <summary>One arm. Side = +1 left, -1 right: mirroring the two arms is just this sign.</summary>
        private sealed class Arm
        {
            public readonly float Side;
            public readonly string Name,
                Input; // joint/body prefix ("left") and IK input prefix ("LeftArm_")
            public readonly Identifier Target,
                Pole; // cached IK input ids
            public uint Site = uint.MaxValue; // grasp site = the LimbIK end effector
            public uint Shoulder = uint.MaxValue; // shoulder pitch link = the LimbIK chain root
            public uint[] Geoms = new uint[0]; // hand collision geoms
            public float[] Base = new float[0]; // their contact force at tare time
            public float Gap,
                Raise,
                Force; // air between palm and side face (m, - = squeezing), height above contact (m), tared force (N)
            public MujocoActuator[] Joints = new MujocoActuator[0]; // shoulder pitch, roll, yaw, elbow (IK), wrist roll, pitch, yaw (script)
            public float[] BendFrom = new float[0]; // joint ctrl when the ready-pose bend started (rad)
            public uint[] WristLink = new uint[0]; // the links those joints turn
            public float[] WristQ = new float[0]; // wrist ctrl targets (rad)
            public float PalmErr; // deg between the palm normal and the squeeze axis
            public MujocoActuator[] Hand = new MujocoActuator[0]; // thumb 1, thumb 2, index 0, middle 0 (script-owned)
            public bool[] IsThumb = new bool[0]; // which hand geoms are thumb links
            public bool[] IsPalm = new bool[0]; // which hand geoms are the palm (wrist_yaw_link)
            public float ThumbForce,
                PalmForce; // tared thumb contact force on the front face; palm-only share of Force (N)
            public bool PalmOn; // the palm has reached its face in this grip (fingers may curl)
            public bool Capped; // its IK target is being pulled in to k_MaxReach of the shoulder
            public float Thumb,
                Curl; // thumb swing back from straight, index + middle curl (rad; right hand mirrored)
            public Vector3 Cmd,
                From,
                To; // IK target actually sent; ends of the path segment being run (SendHands)
            public float T,
                Tau; // segment duration and time into it (s); Tau >= T: no segment running
            public bool Rising; // the segment climbs straight up out of a low spot (wrist held so the fingers don't swing low)
            public bool Direct; // target tracked directly (within k_Snap / gripping) instead of along a path segment
            public string[] GeomName = new string[0]; // body of each hand geom, for the contact log
            public bool[] Touch = new bool[0]; // hand geom in contact at the last PreGrasp frame
            public float[] LoggedQ = new float[0]; // arm joint ctrl at the last PreGrasp line (rad)

            public Arm(float side)
            {
                Side = side;
                Name = side > 0f ? "left" : "right";
                Input = side > 0f ? "LeftArm_" : "RightArm_";
                Target = new Identifier(Input + "Target_Position");
                Pole = new Identifier(Input + "Pole_Target");
            }
        }

        private static readonly string[] s_LegsAndWaistOnly =
        {
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
        };

        // Motion graph: Start stores the current pose and blends to the IK solution over Duration; after that targets are tracked every frame.
        private static readonly Identifier s_Duration = new Identifier("Duration"),
            s_Start = new Identifier("Start");

        // Walker/rotator command IDs: SetFloat(slot, id, value) writes one command channel.
        private const uint k_SetVx = 1u; // forward speed (m/s)
        private const uint k_SetVy = 2u; // sideways speed (m/s)
        private const uint k_SetYawRate = 3u; // turn rate (rad/s)

        private const float k_YawGain = 2f,
            k_MaxYawRate = 1f; // heading P-gain (rad/s per rad) and cap
        private const float k_NearRadius = 0.4f; // m, inside this a walk goal is stepped onto without turning
        private const float k_StepGain = 2f,
            k_MinStepSpeed = 0.1f; // stepping speed = clamp(distance * gain, min, WalkSpeed)
        private const float k_Settle = 0.6f; // s at zero velocity before a policy swap or a reach (G9)
        private const float k_DecelTime = 0.2f; // s of zero commands to an outgoing policy while still active (G9)
        private const float k_TurnRate = 0.7f,
            k_TurnRampIn = 0.5f; // rotator: max rad/s, ramp-in s
        private const float k_WalkProgress = 0.02f,
            k_TurnProgress = 0.02f,
            k_ReachProgress = 0.005f; // m, rad, m: smallest gain that counts as progress
        private const float k_RestSpeed = 0.02f; // m/s, box counts as resting
        private const float k_DepthGain = 0.002f,
            k_MaxSqueezeRate = 0.02f,
            k_MaxSqueezeDepth = 0.03f; // squeeze admittance: m/(N*s), m/s, m
        private const float k_SqueezeMargin = 1.25f,
            k_GripConfirm = 0.5f; // palms press toward this x the required force; grip OK: both palms on their faces and each hand (palm + fingers) >= the required force for 0.5 s
        private const float k_TouchForce = 2f; // N, a hand this loaded is already at the box
        private const float k_LiftSpeed = 0.05f,
            k_SlipTol = 0.01f,
            k_DropTol = 0.05f; // m/s, m, m
        private const float k_CarrySpeed = 0.1f; // m/s, box moves in the hands
        private const float k_PlaceHover = 0.05f,
            k_PlaceLowerSpeed = 0.03f; // m, m/s
        private const float k_OpenSpeed = 0.05f,
            k_ReleaseGap = 0.05f,
            k_ReleaseTime = 1.5f,
            k_RetractUp = 0.10f; // m/s, m, s, m
        private const float k_Widen = 0.05f; // m, a hand landing on the box slides out at most this far
        private const float k_WristGain = 2f,
            k_WristRate = 1f; // wrist servo: rad/s per rad of palm error, cap
        private const float k_ThumbFlare = 0.72f,
            k_ThumbRate = 1.5f; // rad thumb swung back (open); rad/s it swings
        private const float k_MaxCurl = 0.8f,
            k_CurlRate = 0.5f; // rad index + middle curl at most into the side face; rad/s they curl while the grip is short
        private const float k_ThumbStagger = 0.018f; // m, left hand raised 2x this over the right so the straight thumbs pass
        private const float k_Snap = 0.05f,
            k_LocalMove = 0.10f,
            k_SafeMargin = 0.10f; // m: target tracked directly within this; shorter moves across go straight; travel height over belt and boxes
        private const float k_FoldSmooth = 0.8f,
            k_BoxNear = 0.35f; // s lag of a folded hand behind its (swaying, body-relative) fold point; m, hands this close to a box take the safe climb
        private const float k_MinSegment = 0.5f,
            k_SeedBlend = 0.5f; // s shortest path segment; s IK re-seed blend (eases any change of arm pose)
        private const float k_PalmHalf = 0.044f,
            k_SiteOffset = 0.01f; // m (MJCF): palm half-height about the wrist axis; grasp site below (left) / above (right) it
        private const float k_StillSpeed = 0.05f,
            k_StillTime = 0.5f,
            k_StillLog = 4f,
            k_HandLever = 0.45f; // body still: hands carried under this m/s for this many s straight; s between "waiting" lines; m pelvis -> hands

        // Approach (lining the body up with the box), from the walker test: smallest commands that step (side 0.4, forward 0.6, back 0.4 m/s)
        // move ~10-20 cm per 0.25 s burst after coasting ~1 s; a 2 rad/s turn command turns in place (~57 deg/s, ~2 deg coast).
        private const float k_AlignYaw = 0.14f,
            k_TurnStop = 0.1f,
            k_TurnCmd = 2f,
            k_TurnMax = 3f; // rad facing tolerance; rad short of the aim a turn stops (coast); rad/s turn command; s longest turn
        private const float k_AlignSide = 0.06f,
            k_AlignFwd = 0.05f; // m: box centre off the straight-ahead line; front face off FaceDistance
        private const float k_SideCmd = 0.4f,
            k_FwdCmd = 0.6f,
            k_BackCmd = 0.4f,
            k_Burst = 0.25f,
            k_LongBurst = 0.5f,
            k_ApCoast = 1.5f; // m/s burst commands; s burst (long one when far off); s to coast to a stop before measuring again
        private const int k_MaxMoves = 12; // approach corrections per attempt before reaching from where the body is
        private const float k_PlanReach = 0.93f,
            k_MaxReach = 0.95f; // share of the arm every hand target of a reach must lie within (else the attempt fails); hard cap on any IK target
        private const float k_ArmLength = 0.517f,
            k_Comfort = 0.85f; // m shoulder -> grasp site at full stretch (MJCF); share of it the grip is planned for
        private static readonly float[] s_Mirror = { 1f, -1f, -1f, 1f, -1f, 1f, -1f }; // right arm joint = left x this (pitch, roll, yaw, elbow, wrist roll, pitch, yaw; MJCF)
        private const float k_TopMargin = 0.02f,
            k_PlacedWait = 0.5f; // m grasp site stays under the box top; s a placed hand must stop improving
        private const float k_Disturb = 0.02f,
            k_LostDist = 0.25f; // m box moves logged as a disturbance; m that count as the box gone
        private static readonly Vector3[] s_Axes =
        {
            new Vector3(1f, 0f, 0f),
            new Vector3(0f, 1f, 0f),
            new Vector3(0f, 0f, 1f),
        }; // MuJoCo body X, Y, Z

        [Group("Phases")]
        public bool WalkToBox = true; // walk up to the box and square up to the conveyor (off: grasp from where the robot stands)

        [Group("Phases")]
        public bool LiftBox = true; // lift after the grip (off: open the hands again right away)

        [Group("Phases")]
        public bool CarryToTable = true; // turn and walk to the table (off: put the box back down where it was picked)

        [Group("Phases")]
        public bool PlaceBox = true; // set the box down and let go (off: keep holding it)

        [Group("Phases")]
        public bool RepeatLoop = true; // after a box or a failure, go for the next one (off: stop)

        [Group("Phases")]
        public bool WalkerTest = false; // no grasp: measure the walker and rotator - a fixed list of commands, 2 s each, logging how far the body really moved and turned

        [Group("Walk")]
        public float WalkSpeed = 0.3f; // m/s

        [Group("Walk")]
        public float StandDistance = 0.33f; // m, pelvis -> box centre when grasping (arm reach limit)

        [Group("Walk")]
        public float EdgeClearance = 0.15f; // m, pelvis never closer than this to the conveyor's or table's near edge

        [Group("Walk")]
        public float ArriveTolerance = 0.10f; // m, this close to a walk goal counts as arrived

        [Group("Walk")]
        public float WalkTimeout = 60f; // s

        [Group("Walk")]
        public float TurnTolerance = 0.1f; // rad, a turn is done within this

        [Group("Walk")]
        public float FaceDistance = 0.27f; // m, pelvis -> box front face the body lines up to (centred on the box, facing it square) before reaching

        [Group("Walk")]
        public float MinTurnRate = 0.2f; // rad/s, smallest yaw rate sent while turning (rotator deadband)

        [Group("Box")]
        public Entity? ChuteEntity; // conveyor the boxes arrive on (unit-cube mesh, local +Y up)

        [Group("Box")]
        public float ApproachGap = 0.03f; // m of air between each palm and its side face while the hands come down (0 = touching)

        [Group("Box")]
        public float PalmOffset = 0.02f; // m, grasp site -> palm face along the palm normal (MJCF: 0.017), so sites touch at width + 2 x this

        [Group("Box")]
        public float BeltClearance = 0.03f; // m of air under the lower palm edge: hands grip this low on the box, and never lower over the conveyor

        [Group("Box")]
        public float FrontInset = 0.051f; // m, grasp site this far behind the front face: the straight thumb's front surface lies on it (MJCF: 0.051)

        [Group("Box")]
        public float BoxFriction = 1f; // hand-box mu (MuJoCo uses the larger of the two geoms' friction)

        [Group("Arms")]
        public Vector3 FoldOffset = new Vector3(0.15f, 0.15f, 0.20f); // hand target while walking, from the pelvis: (forward, up, outward)

        [Group("Arms")]
        public Vector3 PoleOffset = new Vector3(-0.30f, 0.10f, 0.15f); // elbow hint from the pelvis: (forward, up, outward)

        [Group("Arms")]
        public Vector3 CarryOffset = new Vector3(0.30f, 0.10f, 0f); // box centre while carrying, from the pelvis: (forward, up, left)

        [Group("Ready Pose")]
        public float ShoulderPitch = -30f; // deg, left arm (right mirrored); - swings the arm forward and up (-30 with Elbow -20: whole hand >= 0.19 m over the pelvis, clear of the box top)

        [Group("Ready Pose")]
        public float ShoulderRoll = 0f; // deg, + swings the arm out to the side

        [Group("Ready Pose")]
        public float ShoulderYaw = 0f; // deg, + turns the forearm outward

        [Group("Ready Pose")]
        public float Elbow = -20f; // deg, 0 = bent 90 deg (forearm level, forward); + straightens it down, - bends it up

        [Group("Ready Pose")]
        public float WristRoll = 0f; // deg, 0 = palms facing each other; + turns the palm down

        [Group("Ready Pose")]
        public float WristPitch = 0f; // deg, + tips the fingers down

        [Group("Ready Pose")]
        public float WristYaw = 0f; // deg, + swings the fingers outward

        [Group("Ready Pose")]
        public float BendDuration = 1.5f; // s the arms take to bend into this pose (min-jerk); PreGrasp starts the moment it's reached

        [Group("Arms")]
        public float HandMoveSpeed = 0.2f; // m/s average along each hand path segment (min-jerk: eases in and out, peaks at ~1.9x this)

        [Group("Arms")]
        public float ReachTolerance = 0.05f; // m, grasp site this close to its target = in position

        [Group("Arms")]
        public float ApproachHeight = 0.10f; // m, hands come down beside the box from this far above its top

        [Group("Arms")]
        public float LiftHeight = 0.08f; // m

        [Group("Arms")]
        public float GripSafetyFactor = 2f; // squeeze per hand = factor * m*g / (2*mu)

        [Group("Arms")]
        public float ArmReach = 0.50f; // m, shoulder -> grasp site at full stretch (MJCF: 0.517); a grip lower than this below the shoulders is unreachable

        [Group("Place")]
        public Entity? PlaceTarget; // the table top slab (unit-cube mesh, local +Y up); boxes go on its top face

        [Group("Place")]
        public float PlaceSpacing = 0.25f; // m between slots along the table's near edge

        [Group("Place")]
        public float PlaceEdgeInset = 0.12f; // m, slot line this far in from the table's near edge (and ends)

        [Group("Place")]
        public float PlaceStandDistance = 0.30f; // m, pelvis -> slot when placing (arm reach limit)

        [Group("Recovery")]
        public float StallTime = 3f; // s without progress (hand error, distance, turn error, grip) before a phase counts as stalled

        [Group("Recovery")]
        public float PhaseTimeout = 15f; // s, hard cap on one try of a reach, squeeze, turn or step (walks use WalkTimeout)

        [Group("Recovery")]
        public int MaxBoxAttempts = 2; // failed attempts before a box is skipped (until it moves)

        [Group("Recovery")]
        public int MaxRepositions = 3; // closer stand points tried per grasp and per place

        [Group("Recovery")]
        public float RepositionStep = 0.04f; // m closer per failed reach

        [Group("Recovery")]
        public float RepositionTolerance = 0.03f; // m, stepping goal tolerance (and the least move worth stepping for)

        [Group("Recovery")]
        public float MinStandDistance = 0.20f; // m, pelvis never steps closer than this to the box / slot

        [Group("Recovery")]
        public int MaxSqueezeTries = 3; // squeezes (reopen + re-centre between them) before the grip fails

        [Group("Recovery")]
        public int MaxWalkRetries = 2; // re-turns toward a walk goal after a stalled walk

        [Group("Recovery")]
        public int MaxTurnTries = 2; // a stalled turn is retried (with twice the minimum rate) up to this many tries

        [Group("Policy")]
        public uint WalkerSlotId = 1u;

        [Group("Policy")]
        public uint RotatorSlotId = 2u;

        [Group("Debug")]
        public float LogInterval = 1f; // s between status lines (0 = off)

        [Group("Debug")]
        public float PreGraspLogInterval = 0.1f; // s (sim) between detailed PreGrasp and grip (Squeeze, Lift, carry) lines (0 = off); events are logged as they happen

        private RobotControllerComponent? m_Robot;
        private MujocoSceneComponent? m_Mujoco;
        private readonly Arm[] m_Arms = { new Arm(1f), new Arm(-1f) };
        private readonly List<(uint Body, uint Geom, Vector3 Half)> m_Boxes =
            new List<(uint Body, uint Geom, Vector3 Half)>(); // every free box
        private int[] m_Fails = new int[0]; // failed attempts per box
        private int m_Box = -1; // index of the box being handled
        private uint m_PelvisId = uint.MaxValue;
        private Quaternion m_MjToHz = Quaternion.Identity; // MuJoCo -> Hazel axes
        private Vector3 m_MjOffset; // MuJoCo -> Hazel translation
        private Phase m_Phase = Phase.Start;
        private int m_Step; // sub-step inside a phase
        private float m_PhaseTime,
            m_StepTime; // s since the phase / sub-step started
        private float m_Best,
            m_SinceBest,
            m_WatchTime; // progress watchdog: best metric, s since it improved, s since the watch started
        private uint m_Active; // locomotion policy slot driving the legs + waist
        private float m_GripLift; // m the grip is raised up the box so the arms can reach it
        private float m_TurnYaw; // heading the current turn aims at (fixed when it starts)
        private int m_TurnTries,
            m_WalkRetries,
            m_Repositions,
            m_SqueezeTries; // retries used in the current turn / walk goal / reach / grip
        private float m_Reach; // m, current pelvis -> box (or slot) stand distance; shrinks on failed reaches
        private bool m_Retry; // Recover goes back for the next box (else stops)
        private float m_Still; // s the body has been still without a break (BodyMotion under k_StillSpeed)
        private int m_Moves; // approach corrections made for the current attempt
        private Vector3 m_ApCmd; // approach: command of the current correction (vx, vy, yaw rate)
        private float m_ApTime,
            m_ApTurnTo; // approach: s the command runs; heading a turn aims at
        private int m_TestIdx; // walker test: command being measured
        private Vector3 m_TestFrom,
            m_TestFootL,
            m_TestFootR; // walker test: pelvis and feet (ankle roll links) when the command started
        private float m_TestYaw; // walker test: heading when the command started

        // Walker test commands: (rotator?, vx, vy, yaw rate, s sent). Short bursts at the smallest commands that step (each followed by its
        // reverse), right turns with the walker (alone and with a side step), then left turns in place at three rates.
        private static readonly (bool Rot, float Vx, float Vy, float Yaw, float T)[] s_WalkerTests =
        {
            (false, -0.4f, 0f, 0f, 0.25f),
            (false, 0.6f, 0f, 0f, 0.25f),
            (false, -0.4f, 0f, 0f, 0.5f),
            (false, 0.6f, 0f, 0f, 0.5f),
            (false, 0f, -0.4f, 0f, 0.25f),
            (false, 0f, 0.4f, 0f, 0.25f),
            (false, 0f, -0.4f, 0f, 0.5f),
            (false, 0f, 0.4f, 0f, 0.5f),
            (false, 0f, 0f, -1.5f, 2f),
            (false, 0f, 0f, -2f, 2f),
            (false, 0f, -0.4f, -1f, 1f),
            (false, 0f, 0.4f, -1f, 1f),
            (false, 0f, 0f, 0.6f, 2f),
            (false, 0f, 0f, 1f, 2f),
            (false, 0f, 0f, 1.5f, 2f),
        };
        private const float k_TestSettle = 2f,
            k_TestCoast = 1.5f,
            k_TestWait = 10f; // walker test: s still before each command (waited for at most k_TestWait), s to coast to a stop after it
        private Vector3 m_TestEnd; // walker test: pelvis when the command stopped
        private float m_TestEndYaw; // walker test: heading when the command stopped
        private Vector3 m_ChuteN,
            m_ChuteTop; // conveyor surface normal and the centre of its top face
        private Vector3 m_Stand,
            m_Slot; // where the pelvis walks to (fixed per goal); where the box goes down
        private Vector3 m_Cmd; // last walk command (vx, vy, yaw rate), for the log
        private float m_ToGo,
            m_LogTimer; // distance left to the walk goal (m); time since the last status line

        // Grip frame: box centre, squeeze axis (robot left), forward axis. World coords, or pelvis coords (forward, up, left) while carrying.
        private Vector3 m_Center,
            m_Lat,
            m_Fwd;
        private bool m_Carrying;
        private float m_HalfW,
            m_HalfD,
            m_HalfH; // box half extents along m_Lat, m_Fwd and up
        private float m_Lift,
            m_Boost = 1f,
            m_Y0; // lift (m), force boost while slipping, palm-over-box height at grip
        private Vector3 m_PickAt,
            m_BoxAt; // box centre when the reach started; when last re-aligned to (disturbance reference)
        private float m_SafeY,
            m_Dt = 0.02f; // height (world) hands travel at, clear of the belt and boxes; frame time (s)
        private float m_DbgTimer,
            m_DbgSim; // PreGrasp log: sim s since the last line, sim s in that window
        private readonly System.Diagnostics.Stopwatch m_Wall =
            System.Diagnostics.Stopwatch.StartNew(); // real time, for the sim speed

        // Components, and the walker on legs + waist from the first step so the robot stands (the rest waits for MuJoCo in Begin).
        protected override void OnCreate()
        {
            m_Robot = GetComponent<RobotControllerComponent>();
            m_Mujoco = GetComponent<MujocoSceneComponent>();
            if (m_Robot == null || m_Mujoco == null)
                Log.Error(
                    $"[G1] '{Tag}' needs a RobotControllerComponent and a MujocoSceneComponent - script does nothing"
                );
            else
                SetLocomotion(WalkerSlotId);
        }

        // Runs the current phase; the robot stands still unless the phase walks or turns.
        protected override void OnUpdate(float ts)
        {
            if (m_Robot == null || m_Mujoco == null)
                return;
            m_Dt = ts;
            if (m_PelvisId != uint.MaxValue)
                m_Still = BodyMotion() < k_StillSpeed ? m_Still + ts : 0f;
            m_PhaseTime += ts;
            m_StepTime += ts;
            m_SinceBest += ts;
            m_WatchTime += ts;
            if (
                LogInterval > 0f
                && m_Phase != Phase.Start
                && m_PelvisId != uint.MaxValue
                && (m_LogTimer += ts) >= LogInterval
            )
            {
                m_LogTimer = 0f;
                LogStatus();
            }
            Drive(0f, 0f, 0f);

            switch (m_Phase)
            {
                case Phase.Start:
                    Begin();
                    break;
                case Phase.FindBox:
                    SendHands(Fold, false);
                    if (WalkerTest)
                    {
                        m_TestIdx = 0;
                        Log.Info($"[G1] walker test: {s_WalkerTests.Length} commands, no grasp");
                        Advance(Phase.WalkerTest);
                        break;
                    }
                    m_Box = RestingBox();
                    if (m_Box < 0)
                        break; // nothing to pick yet: the status line lists every box and why
                    m_Reach = StandDistance;
                    m_Repositions = m_SqueezeTries = m_WalkRetries = 0;
                    m_Stand = StandFor(
                        ChuteEntity!,
                        BoxCenter(m_Box),
                        m_Reach,
                        FaceOf(ChuteEntity!)
                    );
                    Log.Info(
                        $"[G1] picking box {m_Box} at {BoxCenter(m_Box)}, {Flat(BoxCenter(m_Box) - Pelvis()).Length():F2} m from the pelvis{(WalkToBox ? $", standing at {m_Stand}" : "")}"
                    );
                    if (WalkToBox)
                        Advance(Phase.FaceBox);
                    else
                        EnterApproach();
                    break;
                case Phase.FaceBox:
                    SendHands(Fold, false);
                    if (
                        (m_Step == 0 && Flat(m_Stand - Pelvis()).Length() < k_NearRadius)
                        || Turn(YawOf(m_Stand - Pelvis()))
                    )
                        Advance(Phase.WalkToBox);
                    break;
                case Phase.WalkToBox:
                    SendHands(Fold, false);
                    if (WalkTo(m_Stand, ArriveTolerance, true))
                        Advance(Phase.FaceChute);
                    else if (Stalled(m_ToGo, k_WalkProgress, WalkTimeout))
                        WalkStalled(Phase.FaceBox);
                    break;
                case Phase.FaceChute:
                    SendHands(Fold, false);
                    if (Turn(YawOf(FaceOf(ChuteEntity!))))
                        EnterApproach();
                    break;
                case Phase.Approach:
                    StepApproach();
                    break;
                case Phase.Ready:
                    StepReady();
                    break;
                case Phase.PreGrasp:
                    StepPreGrasp(ts);
                    break;
                case Phase.Squeeze:
                    StepSqueeze(ts);
                    break;
                case Phase.Reposition:
                    StepReposition(ts);
                    break;
                case Phase.Holding:
                    Hold(ts);
                    break; // test end state: keep the grip
                case Phase.Lift:
                    StepLift(ts);
                    break;
                case Phase.PullIn:
                    if (Hold(ts) && MoveTowards(ref m_Center, CarryOffset, k_CarrySpeed * ts))
                        EnterTurnToTable();
                    break;
                case Phase.TurnToTable:
                    if (
                        Hold(ts)
                        && (
                            (m_Step == 0 && Flat(m_Stand - Pelvis()).Length() < k_NearRadius)
                            || Turn(YawOf(m_Stand - Pelvis()))
                        )
                    )
                        Advance(Phase.WalkToTable);
                    break;
                case Phase.WalkToTable:
                    if (!Hold(ts))
                        break;
                    if (WalkTo(m_Stand, ArriveTolerance, true))
                        Advance(Phase.FaceTable);
                    else if (Stalled(m_ToGo, k_WalkProgress, WalkTimeout))
                        WalkStalled(Phase.TurnToTable);
                    break;
                case Phase.FaceTable:
                    if (Hold(ts) && Turn(YawOf(FaceOf(PlaceTarget!))))
                        Advance(PlaceBox ? Phase.Place : Phase.Holding);
                    break;
                case Phase.Place:
                    StepPlace(ts);
                    break;
                case Phase.Release:
                    StepRelease(ts);
                    break;
                case Phase.Retract:
                    StepRetract();
                    break;
                case Phase.Recover:
                    StepRecover();
                    break;
                case Phase.WalkerTest:
                    StepWalkerTest();
                    break;
                default:
                    SendHands(Fold, false);
                    break; // Done / Failed: stand with the arms folded
            }
            if (m_Phase == Phase.Start)
                return;
            OrientWrists(ts);
            PoseHands(ts);
        }

        // First frames (MuJoCo isn't ready in OnCreate): find the pelvis and boxes, set up the arm IK (failing loudly on a missing part), read the conveyor, fold the arms.
        private void Begin()
        {
            m_MjToHz = m_Mujoco!.GetAxisHzToMj().Conjugate;
            m_MjOffset = m_Mujoco.GetSceneTranslation();
            m_PelvisId = m_Mujoco.GetBodyID("pelvis");
            if (ChuteEntity == null || PlaceTarget == null)
            {
                Fail("assign ChuteEntity (conveyor) and PlaceTarget (table top)", false);
                return;
            }
            FindBoxes();
            if (m_PelvisId == uint.MaxValue || m_Boxes.Count == 0)
            {
                if (m_PhaseTime > 2f)
                    Fail(
                        m_PelvisId == uint.MaxValue
                            ? "no 'pelvis' body in the MuJoCo model"
                            : "no free-moving MuJoCo boxes in the scene",
                        false
                    );
                return; // scene bodies can join the model a few frames after start: retry
            }
            foreach (var b in m_Boxes)
                m_Mujoco.UnweldBody(b.Body); // clear welds left by earlier versions of this script (welds default to persistent)
            m_Fails = new int[m_Boxes.Count];
            foreach (Arm a in m_Arms)
            {
                // The IK owns the shoulders and the elbow; the script owns the wrists (OrientWrists) and the thumb and finger bases (PoseHands).
                var acts = new List<MujocoActuator>();
                foreach (
                    string j in new[]
                    {
                        "_shoulder_pitch_joint",
                        "_shoulder_roll_joint",
                        "_shoulder_yaw_joint",
                        "_elbow_joint",
                        "_wrist_roll_joint",
                        "_wrist_pitch_joint",
                        "_wrist_yaw_joint",
                        "_hand_thumb_1_joint",
                        "_hand_thumb_2_joint",
                        "_hand_index_0_joint",
                        "_hand_middle_0_joint",
                    }
                )
                {
                    MujocoActuator? act = m_Mujoco.GetActuator(a.Name + j);
                    if (act == null)
                    {
                        Fail($"no actuator '{a.Name + j}'", false);
                        return;
                    }
                    act.IK = !j.StartsWith("_wrist") && !j.StartsWith("_hand");
                    acts.Add(act);
                }
                a.Joints = acts.GetRange(0, 7).ToArray();
                a.Hand = acts.GetRange(7, 4).ToArray();
                a.WristQ = new[]
                {
                    (float)a.Joints[4].Value,
                    (float)a.Joints[5].Value,
                    (float)a.Joints[6].Value,
                };
                a.WristLink = Array.ConvertAll(
                    new[] { "_wrist_roll_link", "_wrist_pitch_link", "_wrist_yaw_link" },
                    l => m_Mujoco.GetBodyID(a.Name + l)
                );
                a.Site = m_Mujoco.GetSiteID(a.Name + "_grasp_site");
                a.Shoulder = m_Mujoco.GetBodyID(a.Name + "_shoulder_pitch_link");
                if (
                    a.Site == uint.MaxValue
                    || a.Shoulder == uint.MaxValue
                    || Array.IndexOf(a.WristLink, uint.MaxValue) >= 0
                )
                {
                    Fail(
                        $"no site '{a.Name}_grasp_site' or body '{a.Name}_shoulder_pitch_link' / '{a.Name}_wrist_*_link'",
                        false
                    );
                    return;
                }
                m_Robot!.SetInputBool(new Identifier(a.Input + "Solve_Orientation"), false); // position only
                m_Robot.SetInputBool(new Identifier(a.Input + "High_Accuracy"), true);
                m_Robot.SetInputFloat(new Identifier(a.Input + "Pole_Weight"), 1f);
                HandGeoms(a);
                a.Base = new float[a.Geoms.Length];
                a.Touch = new bool[a.Geoms.Length];
            }
            m_ChuteN = ChuteEntity.Transform.WorldRotationQuat * Vector3.Up;
            m_ChuteTop = TopOf(ChuteEntity);
            SetSafeY();
            if (ApproachGap > 0.1f)
                Log.Warn(
                    $"[G1] ApproachGap is {ApproachGap:F2} m: each palm aims that far outside the box (arms flung sideways) - expected 0.02-0.05"
                );
            Log.Info(
                $"[G1] ready: {m_Boxes.Count} boxes, hand geoms L={m_Arms[0].Geoms.Length} R={m_Arms[1].Geoms.Length}, pelvis at {Pelvis()}"
            );
            SendHands(Fold, true);
            Advance(Phase.FindBox);
        }

        // Approach: line the body up with the box (Ready follows).
        private void EnterApproach()
        {
            m_Moves = 0;
            Advance(Phase.Approach);
        }

        // Approach, one correction per cycle: wait for a still body, measure the box in its own frame, then make the first correction needed -
        // turn in place to face its front face square, side-step until the pelvis-to-box line points straight ahead, step forward / back
        // until the front face is FaceDistance ahead (never closer than EdgeClearance to the conveyor edge) - then coast to a stop and
        // measure again. Lined up (or out of corrections): Ready, whose reach check decides.
        private void StepApproach()
        {
            SendHands(Fold, false);
            if (m_Step == 0)
            {
                if (AwaitStill(m_StepTime, "before lining up with the box"))
                    return;
                SetGripFrame(Heading());
                Vector3 rel = Flat(m_Center - Pelvis());
                float off = Mathf.WrapToPi(YawOf(m_Fwd) - YawOf(Heading())),
                    side = Vector3.Dot(rel, m_Lat),
                    face = Vector3.Dot(rel, m_Fwd) - m_HalfD,
                    room = ChuteRoom();
                string state =
                    $"facing {off * Mathf.Rad2Deg:F0} deg off the box, its centre {side:F2} m left of straight ahead, front face {face:F2} m ahead (aim {FaceDistance:F2}), {room:F2} m of room before the conveyor edge";
                string why;
                if (m_Moves >= k_MaxMoves)
                {
                    Log.Warn($"[G1] approach: {state} - out of corrections, reaching from here");
                    EnterReady();
                    return;
                }
                if (Math.Abs(off) > k_AlignYaw)
                {
                    m_ApTurnTo = YawOf(m_Fwd);
                    m_ApCmd = new Vector3(0f, 0f, Math.Sign(off) * k_TurnCmd);
                    m_ApTime = k_TurnMax;
                    why = $"turning in place {(off > 0f ? "left" : "right")}";
                }
                else if (Math.Abs(side) > k_AlignSide)
                {
                    m_ApCmd = new Vector3(0f, Math.Sign(side) * k_SideCmd, 0f);
                    m_ApTime = Math.Abs(side) > 0.2f ? k_LongBurst : k_Burst;
                    why = $"side-stepping {(side > 0f ? "left" : "right")} ({m_ApTime:F2} s burst)";
                }
                else if (face > FaceDistance + k_AlignFwd && room > 0.1f)
                {
                    m_ApCmd = new Vector3(k_FwdCmd, 0f, 0f);
                    m_ApTime = k_Burst;
                    why = $"stepping forward ({m_ApTime:F2} s burst)";
                }
                else if (face < FaceDistance - k_AlignFwd)
                {
                    m_ApCmd = new Vector3(-k_BackCmd, 0f, 0f);
                    m_ApTime = face < FaceDistance - 0.15f ? k_LongBurst : k_Burst;
                    why = $"stepping back ({m_ApTime:F2} s burst)";
                }
                else
                {
                    Log.Info($"[G1] lined up with the box after {m_Moves} corrections: {state}");
                    EnterReady();
                    return;
                }
                m_Moves++;
                Log.Info($"[G1] approach {m_Moves}/{k_MaxMoves}: {state} -> {why}");
                NextStep();
            }
            else if (m_Step == 1)
            {
                Drive(m_ApCmd.X, m_ApCmd.Y, m_ApCmd.Z);
                bool turned =
                    m_ApCmd.Z != 0f
                    && Math.Abs(Mathf.WrapToPi(m_ApTurnTo - YawOf(Heading()))) < k_TurnStop; // stop early: it coasts on
                if (m_StepTime >= m_ApTime || turned)
                    NextStep();
            }
            else if (m_StepTime >= k_ApCoast)
            {
                m_Step = 0; // coasted to a stop: measure again
                m_StepTime = 0f;
                ResetWatch();
            }
        }

        // How far the pelvis may still step toward the conveyor before it comes within EdgeClearance of its near edge.
        private float ChuteRoom() =>
            Vector3.Dot(ChuteEntity!.Transform.WorldTranslation - Pelvis(), FaceOf(ChuteEntity!))
            - 0.5f
                * Math.Min(ChuteEntity!.Transform.WorldScale.X, ChuteEntity!.Transform.WorldScale.Z)
            - EdgeClearance;

        // Grip frame squared to the box, then Ready: settle, plan the grip and check the reach, then reach (PreGrasp -> Squeeze).
        private void EnterReady()
        {
            SetGripFrame(Heading());
            m_Carrying = false;
            m_Lift = 0f;
            m_Boost = 1f;
            m_PickAt = m_BoxAt = BoxCenter(m_Box);
            SetSafeY();
            foreach (Arm a in m_Arms)
                a.Gap = ApproachGap;
            Advance(Phase.Ready);
        }

        // Farthest hand target of the reach (over the corners and beside the box) from its shoulder, as a share of the arm.
        private float ReachShare(out string which)
        {
            float worst = 0f;
            which = "";
            foreach (Arm a in m_Arms)
                for (int i = 0; i < 2; i++)
                {
                    float r =
                        ((i == 0 ? HandTarget(a) : GripPoint(a)) - BodyPos(a.Shoulder)).Length()
                        / k_ArmLength;
                    if (r <= worst)
                        continue;
                    worst = r;
                    which = $"{a.Name} hand {(i == 0 ? "over its corner" : "beside the box")}";
                }
            return worst;
        }

        // Grip planned from the live shoulders, then the reach checked from where the robot stands: every hand target (over the corners and beside
        // the box) within k_PlanReach of its shoulder (Approach has lined the body up as close as the conveyor edge allows), else the attempt
        // fails here (retried; the box is skipped after MaxBoxAttempts). Logs how the body stands to the box.
        private bool Reachable()
        {
            PlanGrip();
            foreach (Arm a in m_Arms)
                a.Raise = 2f * m_HalfH - GripHeight(a) + ApproachHeight;
            float off = Mathf.WrapToPi(YawOf(m_Fwd) - YawOf(Heading())),
                face = Vector3.Dot(Flat(m_Center - Pelvis()), m_Fwd) - m_HalfD,
                reach = ReachShare(out string which);
            string state =
                $"facing {off * Mathf.Rad2Deg:F0} deg off the box, front face {face:F2} m ahead, farthest hand target {reach * 100f:F0}% of the arm ({which})";
            if (reach > k_PlanReach)
            {
                Fail(
                    $"box out of reach from where the robot stands: {state} (limit {k_PlanReach * 100f:F0}%) - move the robot or the box, or lower EdgeClearance so it can stand closer"
                ); // retried; skipped after MaxBoxAttempts
                return false;
            }
            Log.Info($"[G1] body vs box: {state} -> reaching");
            return true;
        }

        // Waits for k_StillTime of unbroken stillness, saying so every k_StillLog s; after PhaseTimeout of waiting the attempt fails (no reach on
        // a moving body). True while still waiting (or just failed).
        private bool AwaitStill(float waited, string where)
        {
            if (m_Still >= k_StillTime)
                return false;
            if (waited > PhaseTimeout)
            {
                Fail(
                    $"body never settled {where}: {BodyMotion():F2} m/s at the hands after {waited:F0} s"
                );
                return true;
            }
            if ((int)(waited / k_StillLog) != (int)((waited - m_Dt) / k_StillLog))
                Log.Warn(
                    $"[G1] waiting for the body to settle {where}: {BodyMotion():F2} m/s at the hands, {waited:F0} s so far"
                );
            return true;
        }

        // Grip height from the live shoulders: raised up the box until each grasp site is within k_Comfort of the arm's reach, up to k_TopMargin
        // under its top; if even that isn't enough, says by how much the box sits too low. Logs the plan.
        private void PlanGrip()
        {
            m_GripLift = 0f;
            float reach = k_Comfort * k_ArmLength,
                need = 0f,
                maxLift = MaxGripLift();
            foreach (Arm a in m_Arms)
            {
                Vector3 d = GripPoint(a) - BodyPos(a.Shoulder);
                float across = Flat(d).Length();
                need = Math.Max(
                    need,
                    -d.Y - (float)Math.Sqrt(Math.Max(0f, reach * reach - across * across))
                ); // shoulder height over the grip beyond what the arm covers
            }
            m_GripLift = Math.Min(need, maxLift);
            Log.Info(
                $"[G1] reach plan: shoulders {BodyPos(m_Arms[0].Shoulder).Y:F2} m, grip L {GripPoint(m_Arms[0]).Y:F2} R {GripPoint(m_Arms[1]).Y:F2} m (box {m_Center.Y - m_HalfH:F2}-{m_Center.Y + m_HalfH:F2}), grip raised {m_GripLift:F3} m"
            );
            if (need > maxLift + 0.005f)
                Log.Warn(
                    $"[G1] box sits {need - maxLift:F2} m too low for the arms even gripping just under its top - raise the belt by that much (reaching anyway)"
                );
        }

        // Grasp-site target beside the box at grip height (palm ApproachGap clear, not raised, not lifted).
        private Vector3 GripPoint(Arm a) =>
            m_Center
            + m_Lat * (a.Side * (m_HalfW + PalmOffset + ApproachGap))
            + m_Fwd * (FrontInset - m_HalfD)
            + Vector3.Up * (GripHeight(a) - m_HalfH);

        // How far the grip can still rise before a grasp site comes within k_TopMargin of the box top.
        private float MaxGripLift() =>
            2f * m_HalfH
            - k_TopMargin
            - Math.Max(GripHeight(m_Arms[0]), GripHeight(m_Arms[1]))
            + m_GripLift;

        // A hand is beside the box, judged from its live grasp site in the box frame: palm outside its side face by no more than the closing
        // range, thumb within 3 cm of the front face, palm overlapping the side face above the belt. Outputs those three numbers for the log.
        private bool Placed(Arm a, out float side, out float front, out float up)
        {
            Vector3 p = SiteWorld(a.Site) - m_Center;
            side = a.Side * Vector3.Dot(p, m_Lat) - m_HalfW - PalmOffset;
            front = Vector3.Dot(p, m_Fwd) + m_HalfD - FrontInset;
            up = p.Y + m_HalfH;
            return side > -0.015f
                && side < ApproachGap + k_Widen
                && Math.Abs(front) < 0.03f
                && up > PalmBelow(a)
                && up < 2f * m_HalfH - 0.01f;
        }

        private bool BothPlaced() =>
            Placed(m_Arms[0], out _, out _, out _) && Placed(m_Arms[1], out _, out _, out _);

        // Inner loop, every frame of the reach: re-reads the box so the hands follow it (nudged, sliding) while follow is set, logs each
        // disturbance, and fails the attempt (next box) once the box has gone.
        private bool Track(bool follow)
        {
            Vector3 c = BoxCenter(m_Box);
            float gone = Flat(c - m_PickAt).Length(),
                moved = (c - m_BoxAt).Length();
            if (gone > k_LostDist || ChuteGap(m_Box) > 0.05f)
            {
                Fail(
                    $"box left the grasp: {gone:F2} m from where the reach started, {ChuteGap(m_Box):F2} m off the conveyor"
                );
                return false;
            }
            if (moved > k_Disturb)
            {
                Log.Warn(
                    $"[G1] box moved {moved:F3} m (now at {c}){(follow ? " -> hands follow it" : " - hands already touching, holding the frame")}"
                );
                m_BoxAt = c;
            }
            if (!follow)
                return true;
            Vector3 c0 = m_Center,
                f0 = m_Fwd,
                g0 = HandTarget(m_Arms[0]),
                g1 = HandTarget(m_Arms[1]);
            SetGripFrame(m_Fwd);
            float dc = (m_Center - c0).Length(),
                turn = Mathf.Acos(Mathf.Clamp(Vector3.Dot(m_Fwd, f0), -1f, 1f)) * Mathf.Rad2Deg;
            if (m_Phase == Phase.PreGrasp && (dc > 0.002f || turn > 0.5f))
                Log.Warn(
                    $"[G1] PG frame re-read: box centre moved {dc:F3} m {F(m_Center - c0)}, forward turned {turn:F1} deg -> hand goals moved L {(HandTarget(m_Arms[0]) - g0).Length():F3} m, R {(HandTarget(m_Arms[1]) - g1).Length():F3} m (step {m_Step})"
                );
            return true;
        }

        // Ready: once the body has been still for k_StillTime straight and every hand target is in reach (Reachable), the shoulders and elbows come off the IK and all seven arm joints bend into the
        // inspector's ready pose (right arm mirrored) over BendDuration, min-jerk - elbows bent, hands up clear of the belt. The moment it's
        // reached the IK takes over again and PreGrasp starts.
        private void StepReady()
        {
            if (!Track(true))
                return;
            if (m_Step == 0)
            {
                SendHands(Fold, false); // folded, body-relative, while settling
                if (AwaitStill(m_StepTime, "before bending the arms") || !Reachable())
                    return; // arm moves on a swaying body make the walker step
                Log.Info(
                    $"[G1] reach: {GripInfo()} | front-bottom corners L {Corner(1f)} R {Corner(-1f)} | palms touch at width + 2 x PalmOffset {2f * (m_HalfW + PalmOffset):F3} m | hands travel at {m_SafeY:F2} m"
                );
                foreach (Arm a in m_Arms)
                    a.BendFrom = Array.ConvertAll(a.Joints, j => (float)j.Value);
                ArmIK(false);
                NextStep();
                return;
            }
            float u = Math.Min(1f, m_StepTime / Math.Max(BendDuration, 0.01f)),
                k = u * u * u * (10f + u * (6f * u - 15f)); // min-jerk progress
            float[] pose =
            {
                ShoulderPitch,
                ShoulderRoll,
                ShoulderYaw,
                Elbow,
                WristRoll,
                WristPitch,
                WristYaw,
            };
            foreach (Arm a in m_Arms)
                for (int i = 0; i < 7; i++)
                {
                    float goal = pose[i] * Mathf.Deg2Rad * (a.Side > 0f ? 1f : s_Mirror[i]);
                    a.Joints[i].Value = a.BendFrom[i] + (goal - a.BendFrom[i]) * k;
                }
            if (u < 1f)
                return;
            ArmIK(true);
            foreach (Arm a in m_Arms)
                for (int i = 0; i < 3; i++)
                    a.WristQ[i] = (float)a.Joints[4 + i].Value; // the wrist servo carries on from the pose
            SendHands(HandTarget, true);
            Advance(Phase.PreGrasp);
        }

        // Shoulders and elbows to the IK (true), or to the script for the ready-pose bend (false).
        private void ArmIK(bool on)
        {
            foreach (Arm a in m_Arms)
                for (int i = 0; i < Math.Min(4, a.Joints.Length); i++)
                    a.Joints[i].IK = on;
        }

        // Over the corners the contact sensing is tared and the hands come straight down beside the box, ApproachGap clear of it (a hand that lands
        // on it slides outward). Grasp (Squeeze) once both are within ReachTolerance - or beside the box faces (Placed) and no longer improving.
        // Hands that stall short adapt (GraspOutOfReach).
        private void StepPreGrasp(float ts)
        {
            if (!Track(true))
                return;
            SendHands(HandTarget, false);
            LogPreGrasp(ts);
            if (Gliding())
            {
                ResetWatch();
                return;
            } // on its way along the planned path: that's progress
            ReadForces();
            if (m_Step == 1)
                foreach (Arm a in m_Arms)
                    if (a.Force > k_TouchForce)
                    {
                        if (a.Gap <= ApproachGap)
                            Log.Warn(
                                $"[G1] PG {a.Name} landed on the box ({a.Force:F1} N) -> sliding out, gap widening up to {ApproachGap + k_Widen:F3} m"
                            );
                        a.Gap = Math.Min(a.Gap + k_OpenSpeed * ts, ApproachGap + k_Widen);
                    }
            float err = Math.Max(HandError(m_Arms[0]), HandError(m_Arms[1]));
            bool stalled = err > ReachTolerance && Stalled(err, k_ReachProgress, PhaseTimeout);
            if (
                m_Step == 1
                && (err <= ReachTolerance || (BothPlaced() && m_SinceBest > k_PlacedWait))
            )
            {
                if (err > ReachTolerance)
                    Log.Info(
                        $"[G1] hands {err:F3} m from their targets but beside the box faces - grasping"
                    );
                Log.Info(
                    $"[G1] PG end of descent after {m_StepTime:F2} s: err L {HandError(m_Arms[0]):F3} R {HandError(m_Arms[1]):F3} m, placed L {Placed(m_Arms[0], out _, out _, out _)} R {Placed(m_Arms[1], out _, out _, out _)}"
                );
                foreach (Arm a in m_Arms)
                    a.PalmOn = false;
                Advance(Phase.Squeeze);
                return;
            }
            if (err > ReachTolerance)
            {
                if (stalled)
                    GraspOutOfReach();
                return;
            }
            if (AwaitStill(m_WatchTime, "over the corners"))
                return; // hold over the corners until the body stops swaying
            Log.Info(
                $"[G1] PG over the corners after {m_StepTime:F2} s (err L {HandError(m_Arms[0]):F3} R {HandError(m_Arms[1]):F3} m, body still {m_Still:F1} s, {BodyMotion():F3} m/s at the hands) -> tare, descend"
            );
            Tare();
            foreach (Arm a in m_Arms)
                a.Raise = 0f;
            SendHands(HandTarget, true);
            NextStep();
        }

        // PreGrasp diagnostics. Every frame: each hand geom starting or ending contact (body, raw and tared force), and the sim falling behind
        // real time. Every PreGraspLogInterval s of sim: one body line (sim speed, pelvis, box) and one line per hand (path segment, IK command
        // vs goal vs actual grasp site, tracking lag, reach, placement in the box frame, forces, arm joints and how far they moved).
        private void LogPreGrasp(float ts)
        {
            LogContacts("PG");
            m_DbgSim += ts;
            if (PreGraspLogInterval <= 0f || (m_DbgTimer += ts) < PreGraspLogInterval)
                return;
            float wall = (float)m_Wall.Elapsed.TotalSeconds,
                speed = m_DbgSim / Math.Max(wall, 1e-4f);
            m_Wall.Restart();
            m_DbgTimer = m_DbgSim = 0f;
            m_Mujoco!.GetBodyPoseMj(m_PelvisId, out _, out _, out Vector3 pv, out Vector3 pw);
            m_Mujoco.GetBodyPoseMj(
                m_Boxes[m_Box].Body,
                out _,
                out _,
                out Vector3 bv,
                out Vector3 bw
            );
            Vector3 c = BoxCenter(m_Box);
            Log.Info(
                $"[G1] PG t={m_PhaseTime:F2} step {m_Step} ({(m_Step == 0 ? "to over the corners" : "descending")}) | sim {speed:F2}x real time ({wall * 1000f:F0} ms wall)"
                    + $" | pelvis {F(Pelvis())} v {F(m_MjToHz * pv)} ({pv.Length():F3} m/s) yaw {YawOf(Heading()) * Mathf.Rad2Deg:F1} deg, ang vel {pw.Length():F3} rad/s | walk cmd {F(m_Cmd)}"
                    + $" | box {F(c)} v {bv.Length():F3} m/s, ang vel {bw.Length():F3} rad/s, {(c - m_PickAt).Length():F3} m from the reach start, {(c - m_BoxAt).Length():F3} m from the last re-read, belt gap {ChuteGap(m_Box):F3} m, contact {m_Mujoco.GetGeomContactForce(m_Boxes[m_Box].Geom):F1} N"
                    + $" | grip frame centre {F(m_Center)} fwd {F(m_Fwd)}, grip lift {m_GripLift:F3} m{(speed < 0.5f ? " | SIM SLOW" : "")}"
            );
            foreach (Arm a in m_Arms)
            {
                Vector3 site = SiteWorld(a.Site),
                    goal = HandTarget(a),
                    sv = m_MjToHz * m_Mujoco.GetSiteLinearVelocityMj(a.Site),
                    sh = BodyPos(a.Shoulder);
                bool placed = Placed(a, out float side, out float front, out float up);
                float[] q = Array.ConvertAll(a.Joints, j => (float)j.Value);
                float jump = 0f;
                for (int i = 0; i < Math.Min(q.Length, a.LoggedQ.Length); i++)
                    jump = Math.Max(jump, Math.Abs(q[i] - a.LoggedQ[i]));
                a.LoggedQ = q;
                var touching = new List<string>();
                for (int i = 0; i < a.Geoms.Length; i++)
                    if (a.Touch[i])
                        touching.Add(a.GeomName[i]);
                Log.Info(
                    $"[G1] PG   {a.Name}: {(a.Direct ? "tracking the goal directly" : $"segment {a.Tau:F2}/{a.T:F2} s {F(a.From)} -> {F(a.To)}{(a.Rising ? " (rising, wrist held)" : "")}")}"
                        + $" | IK cmd {F(a.Cmd)}, goal {F(goal)}, site {F(site)} v {sv.Length():F3} m/s | lag site->cmd {(a.Cmd - site).Length():F3} m, site->goal {(goal - site).Length():F3} m, cmd->goal {(goal - a.Cmd).Length():F3} m"
                        + $" | reach: site {(site - sh).Length() / k_ArmLength * 100f:F0}% / cmd {(a.Cmd - sh).Length() / k_ArmLength * 100f:F0}% of the arm"
                        + $" | box frame: palm {side:F3} out, thumb {front:F3} fwd, site {up:F3} up{(placed ? " (placed)" : "")}, gap {a.Gap:F3}, raise {a.Raise:F3} m"
                        + $" | force palm {a.Force:F1} thumb {a.ThumbForce:F1} N, touching [{string.Join(", ", touching)}] | palm {a.PalmErr:F0} deg off"
                        + $" | joints deg [{string.Join(", ", Array.ConvertAll(q, v => (v * Mathf.Rad2Deg).ToString("F0")))}], max change {jump * Mathf.Rad2Deg:F1} deg since the last line"
                );
            }
        }

        private static string F(Vector3 v) => $"({v.X:F3}, {v.Y:F3}, {v.Z:F3})";

        // Each hand geom starting or ending contact (body, raw and tared force), with the step, grasp site and box.
        private void LogContacts(string tag)
        {
            foreach (Arm a in m_Arms)
                for (int i = 0; i < a.Geoms.Length; i++)
                {
                    float raw = m_Mujoco!.GetGeomContactForce(a.Geoms[i]);
                    bool touch = raw - a.Base[i] > 0.5f;
                    if (touch != a.Touch[i])
                        Log.Warn(
                            $"[G1] {tag} contact {(touch ? "START" : "end")}: {a.GeomName[i]} raw {raw:F1} N (tare {a.Base[i]:F1}) | {m_Phase} step {m_Step}, site {F(SiteWorld(a.Site))}, box {F(BoxCenter(m_Box))}"
                        );
                    a.Touch[i] = touch;
                }
        }

        // Grip diagnostics (Squeeze, Lift, carry, Place): contact events every frame; every PreGraspLogInterval s of sim one body line (sim
        // speed, required force, pelvis, walk command, box, slip) and one line per hand: palm / finger / thumb force, gap target, the palm's
        // measured distance to its face, how much of the palm's height lies on the face, curl, palm tilt, touching geoms.
        private void LogGrip(float ts)
        {
            LogContacts("GR");
            m_DbgSim += ts;
            if (PreGraspLogInterval <= 0f || (m_DbgTimer += ts) < PreGraspLogInterval)
                return;
            float wall = (float)m_Wall.Elapsed.TotalSeconds,
                speed = m_DbgSim / Math.Max(wall, 1e-4f);
            m_Wall.Restart();
            m_DbgTimer = m_DbgSim = 0f;
            m_Mujoco!.GetBodyPoseMj(m_PelvisId, out _, out _, out Vector3 pv, out Vector3 pw);
            m_Mujoco.GetBodyPoseMj(
                m_Boxes[m_Box].Body,
                out _,
                out _,
                out Vector3 bv,
                out Vector3 bw
            );
            Vector3 c = BoxCenter(m_Box),
                lat = m_Carrying ? BodyDir(m_Lat) : m_Lat;
            string slip =
                m_Phase == Phase.Squeeze
                    ? "not lifted"
                    : $"{PalmMidY() - c.Y - m_Y0:F3} m (+ = box sliding down)";
            Log.Info(
                $"[G1] GR {m_Phase} t={m_PhaseTime:F2} | sim {speed:F2}x real time | need {RequiredForce():F1} N per palm (boost {m_Boost:F2}), lift {m_Lift:F3} m"
                    + $" | pelvis {F(Pelvis())} {pv.Length():F3} m/s, ang vel {pw.Length():F3} rad/s, yaw {YawOf(Heading()) * Mathf.Rad2Deg:F1} deg | walk cmd {F(m_Cmd)}"
                    + $" | box {F(c)} v {bv.Length():F3} m/s, ang vel {bw.Length():F3} rad/s, slip {slip}, hand gap {(SiteWorld(m_Arms[0].Site) - SiteWorld(m_Arms[1].Site)).Length():F3} m (palms touch at {2f * (m_HalfW + PalmOffset):F3}){(speed < 0.5f ? " | SIM SLOW" : "")}"
            );
            foreach (Arm a in m_Arms)
            {
                Vector3 site = SiteWorld(a.Site);
                float face = a.Side * Vector3.Dot(site - c, lat) - m_HalfW - PalmOffset,
                    lo = site.Y - PalmBelow(a),
                    on =
                        Math.Max(
                            0f,
                            Math.Min(lo + 2f * k_PalmHalf, c.Y + m_HalfH)
                                - Math.Max(lo, c.Y - m_HalfH)
                        ) / (2f * k_PalmHalf);
                var touching = new List<string>();
                for (int i = 0; i < a.Geoms.Length; i++)
                    if (a.Touch[i])
                        touching.Add(a.GeomName[i]);
                Log.Info(
                    $"[G1] GR   {a.Name}: palm {a.PalmForce:F1} N{(a.PalmOn ? "" : " (not on its face yet)")}, fingers {a.Force - a.PalmForce:F1} N, thumb {a.ThumbForce:F1} N"
                        + $" | gap target {a.Gap:F3} m, palm measured {face:F3} m off its face, palm height on the face {on * 100f:F0}% (lower edge {lo - (c.Y - m_HalfH):F3} m over the box bottom)"
                        + $" | curl {a.Curl:F2} rad, thumb {a.Thumb:F2} rad, palm {a.PalmErr:F0} deg off | err {HandError(a):F3} m | touching [{string.Join(", ", touching)}]"
                );
            }
        }

        // Grasp: the thumbs swing onto the front face and index + middle curl into the side faces while the arms close in, each hand pushing
        // toward the other, until both palms have reached their faces and each hand holds the required force for GripConfirm s (the hands follow the box until the first touch). A grip that
        // stops building with a hand at its squeeze limit (palm gap and finger curl both maxed) is reopened and re-centred (MaxSqueezeTries).
        private void StepSqueeze(float ts)
        {
            if (!Track(Contact() < k_TouchForce))
                return;
            SendHands(HandTarget, false);
            float req = RequiredForce();
            Squeeze(ts, req);
            LogGrip(ts);
            bool stuck = false;
            foreach (Arm a in m_Arms)
            {
                bool held = a.PalmOn && a.Force >= req; // palm has reached its face; palm + fingers hold the required force
                if (!held)
                    m_StepTime = 0f;
                stuck |= !held && a.Gap <= 0.001f - k_MaxSqueezeDepth;
            }
            if (!stuck)
                m_SinceBest = 0f; // still closing or holding: that's progress
            if (m_StepTime >= k_GripConfirm)
            {
                Log.Info(
                    $"[G1] grip: palm L={m_Arms[0].PalmForce:F1} R={m_Arms[1].PalmForce:F1} N (need {req:F1} N), fingers L={m_Arms[0].Force - m_Arms[0].PalmForce:F1} R={m_Arms[1].Force - m_Arms[1].PalmForce:F1} N, thumb L={m_Arms[0].ThumbForce:F1} R={m_Arms[1].ThumbForce:F1} N, curl L={m_Arms[0].Curl:F2} R={m_Arms[1].Curl:F2} rad, gap L={m_Arms[0].Gap:F3} R={m_Arms[1].Gap:F3} m"
                );
                m_Y0 = PalmMidY() - BoxCenter(m_Box).Y;
                m_Slot = m_Center - Vector3.Up * m_HalfH; // pick spot (bottom centre): where the box goes back when not carried to the table
                m_Repositions = 0;
                Advance(
                    LiftBox ? Phase.Lift
                    : PlaceBox ? Phase.Release
                    : Phase.Holding
                );
            }
            else if (m_SinceBest > StallTime || m_WatchTime > PhaseTimeout)
                RetrySqueeze(req);
        }

        // Grip didn't build: reopen (fingers ease straight, thumbs back), re-measure the box and re-centre the hands beside it, then grasp again;
        // fail after MaxSqueezeTries.
        private void RetrySqueeze(float req)
        {
            string why =
                $"palm L={m_Arms[0].PalmForce:F1} R={m_Arms[1].PalmForce:F1} N of {req:F1} N (fingers L={m_Arms[0].Force - m_Arms[0].PalmForce:F1} R={m_Arms[1].Force - m_Arms[1].PalmForce:F1} N), gap L={m_Arms[0].Gap:F3} R={m_Arms[1].Gap:F3} m";
            if (++m_SqueezeTries >= MaxSqueezeTries)
            {
                Fail($"grip too weak after {MaxSqueezeTries} squeezes: {why}");
                return;
            }
            Log.Warn(
                $"[G1] {why} -> reopening and re-centring on the box ({m_SqueezeTries}/{MaxSqueezeTries - 1})"
            );
            SetGripFrame(m_Fwd);
            foreach (Arm a in m_Arms)
            {
                a.Gap = ApproachGap;
                a.PalmOn = false;
            }
            Advance(Phase.PreGrasp);
            NextStep(); // straight to "beside the box": the hands just open and re-centre at grip height
            SendHands(HandTarget, true); // after the phase change, so the reopen eases along a path instead of jumping like a grip target
        }

        // Hands stopped short: grip higher up the box if the worse hand is below its target and there's box left; else fail, stating the shortfall.
        private void GraspOutOfReach()
        {
            Arm w = HandError(m_Arms[0]) >= HandError(m_Arms[1]) ? m_Arms[0] : m_Arms[1];
            Vector3 miss = HandTarget(w) - SiteWorld(w.Site);
            float along = Vector3.Dot(Flat(miss), m_Fwd),
                below = -miss.Y,
                room = MaxGripLift() - m_GripLift;
            string why =
                $"{w.Name} hand short by {miss.Length():F3} m ({along:F3} along the grip axis, {below:F3} below)";
            ResetWatch();
            if (below > 0.01f && room > 0.005f)
            {
                float lift = Math.Min(below, room);
                m_GripLift += lift;
                foreach (Arm a in m_Arms)
                    if (a.Raise > 0f)
                        a.Raise -= lift; // over-the-corner targets stay put
                Log.Warn($"[G1] {why} -> gripping {m_GripLift:F3} m higher up the box"); // the hand goals jump up by this much
                return;
            }
            Fail(
                $"{why}: no higher grip left (grip raised {m_GripLift:F3} m) - raise the belt or move the robot"
            );
        }

        // Next stand point m_Reach in front of p along the grip axis, a little closer each failed try (>= MinStandDistance, EdgeClearance
        // kept from the surface edge). False if the repositions are used up or the new point isn't a real move.
        private bool StepCloser(Entity surface, Vector3 p, string why)
        {
            float dist = Vector3.Dot(Flat(p - Pelvis()), m_Fwd);
            if (dist <= m_Reach + RepositionTolerance)
                m_Reach = Math.Max(MinStandDistance, dist - RepositionStep); // already this close: a few cm closer
            Vector3 stand = StandFor(surface, p, m_Reach, m_Fwd);
            float move = Flat(stand - Pelvis()).Length();
            if (m_Repositions >= MaxRepositions || move < RepositionTolerance)
                return false;
            m_Stand = stand;
            Log.Warn(
                $"[G1] {why} -> stepping {move:F2} m to stand {m_Reach:F2} m from it ({++m_Repositions}/{MaxRepositions})"
            );
            return true;
        }

        // Steps onto the closer stand point without turning, holding the box, settles, then places again.
        private void StepReposition(float ts)
        {
            if (!Hold(ts))
                return;
            if (m_Step == 0)
            {
                bool there = WalkTo(m_Stand, RepositionTolerance, false);
                if (!there && !Stalled(m_ToGo, k_WalkProgress, PhaseTimeout))
                    return;
                if (!there)
                    Log.Warn($"[G1] stepping stalled {m_ToGo:F2} m short -> placing from here");
                NextStep();
            }
            else if (m_StepTime >= k_Settle)
                Advance(Phase.Place);
        }

        // Lift straight up, then hold the box in pelvis coords so it follows the body; next carry it, or put it back down where it was picked.
        private void StepLift(float ts)
        {
            if (!Hold(ts))
                return;
            m_Lift = Math.Min(m_Lift + k_LiftSpeed * ts, LiftHeight);
            if (m_Lift < LiftHeight)
                return;
            m_Center = ToBody(m_Center + Vector3.Up * m_Lift - Pelvis());
            m_Lat = ToBody(m_Lat);
            m_Fwd = ToBody(m_Fwd);
            m_Lift = 0f;
            m_Carrying = true;
            Advance(
                CarryToTable ? Phase.PullIn
                : PlaceBox ? Phase.Place
                : Phase.Holding
            );
        }

        // Picks the nearest free table slot and where to stand for it, then turns toward that spot.
        private void EnterTurnToTable()
        {
            if (!FreeSlot(out m_Slot))
            {
                Fail("the table is full", false);
                return;
            }
            m_Reach = PlaceStandDistance;
            m_WalkRetries = 0;
            m_Stand = StandFor(PlaceTarget!, m_Slot, m_Reach, FaceOf(PlaceTarget!));
            Log.Info(
                $"[G1] placing at {m_Slot}, standing at {m_Stand}, {Vector3.Dot(m_Slot - m_Stand, FaceOf(PlaceTarget!)):F2} m from the slot"
            );
            Advance(Phase.TurnToTable);
        }

        // Settle, switch the grip to world coords, move the box over its spot, then lower it until the surface holds it. If the hands
        // can't get it down, step closer and place again (MaxRepositions); else release where it is.
        private void StepPlace(float ts)
        {
            if (!Hold(ts))
                return;
            Vector3 rest = m_Slot + Vector3.Up * m_HalfH; // box centre resting on the surface
            if (m_Step == 0 && m_StepTime >= k_Settle)
            {
                m_Center = BodyPoint(m_Center);
                m_Lat = BodyDir(m_Lat);
                m_Fwd = BodyDir(m_Fwd);
                m_Carrying = false;
                NextStep();
            }
            else if (
                m_Step == 1
                && MoveTowards(ref m_Center, rest + Vector3.Up * k_PlaceHover, k_CarrySpeed * ts)
            )
                NextStep();
            else if (m_Step == 2)
            {
                m_Center = m_Center - Vector3.Up * (k_PlaceLowerSpeed * ts);
                float above = BoxCenter(m_Box).Y - rest.Y; // how far the box still is above its resting height
                if (above > 0.02f && m_Center.Y > BoxCenter(m_Box).Y - 0.02f)
                    return; // still coming down
                bool low = LowBy(rest.Y) > 0f;
                string why = $"box {above:F3} m above its spot, the hands can't go lower";
                if (
                    above > 0.02f
                    && !low
                    && StepCloser(CarryToTable ? PlaceTarget! : ChuteEntity!, m_Slot, why)
                )
                {
                    // Carry it in pelvis coords while stepping.
                    m_Center = ToBody(m_Center - Pelvis());
                    m_Lat = ToBody(m_Lat);
                    m_Fwd = ToBody(m_Fwd);
                    m_Carrying = true;
                    Advance(Phase.Reposition);
                    return;
                }
                Log.Info(
                    $"[G1] releasing {above:F3} m above the spot{(above <= 0.02f ? "" : low ? " - the spot is too low for the arms" : " - no closer stand point helps")}"
                );
                Advance(Phase.Release);
            }
        }

        // Open both palms away from the box, then start lifting the hands clear.
        private void StepRelease(float ts)
        {
            foreach (Arm a in m_Arms)
                a.Gap = Math.Min(a.Gap + k_OpenSpeed * ts, k_ReleaseGap);
            SendHands(HandTarget, false);
            if (m_PhaseTime < k_ReleaseTime)
                return;
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
                if (Gliding())
                    return;
                SendHands(Fold, true);
                NextStep();
            }
            else
            {
                SendHands(Fold, false);
                if (Gliding())
                    return;
                Log.Info(
                    $"[G1] placed {Flat(BoxCenter(m_Box) - m_Slot).Length():F3} m from its spot"
                );
                Advance(RepeatLoop ? Phase.FindBox : Phase.Done);
            }
        }

        // Walker test, one command per pass: settle (still body), note pose, send the command for its time (rotator swapped in for its
        // commands), zero it (rotator: while still active, then back to the walker), coast, then log the real move against the commanded one.
        private void StepWalkerTest()
        {
            SendHands(Fold, false);
            var c = s_WalkerTests[m_TestIdx];
            switch (m_Step)
            {
                case 0:
                    if (
                        m_StepTime < k_TestSettle
                        || (m_Still < k_StillTime && m_StepTime < k_TestWait)
                    )
                        return;
                    m_TestFrom = Pelvis();
                    m_TestYaw = YawOf(Heading());
                    m_TestFootL = BodyPos(m_Mujoco!.GetBodyID("left_ankle_roll_link"));
                    m_TestFootR = BodyPos(m_Mujoco.GetBodyID("right_ankle_roll_link"));
                    if (c.Rot)
                        SetLocomotion(RotatorSlotId);
                    NextStep();
                    return;
                case 1:
                    Drive(c.Vx, c.Vy, c.Yaw);
                    if ((int)(m_StepTime / 0.25f) != (int)((m_StepTime - m_Dt) / 0.25f))
                    {
                        m_Mujoco!.GetBodyPoseMj(
                            m_PelvisId,
                            out _,
                            out _,
                            out Vector3 v,
                            out Vector3 w
                        );
                        Vector3 hv = m_MjToHz * v;
                        Log.Info(
                            $"[G1] WT   t={m_StepTime:F1} s: pelvis moving fwd {Vector3.Dot(hv, Heading()):F2} left {Vector3.Dot(hv, LeftOf(Heading())):F2} m/s, turning {w.Length():F2} rad/s, height {Pelvis().Y:F3} m"
                        );
                    }
                    if (m_StepTime >= c.T)
                    {
                        m_TestEnd = Pelvis();
                        m_TestEndYaw = YawOf(Heading());
                        NextStep();
                    }
                    return;
                default:
                    if (m_Active == RotatorSlotId)
                    {
                        if (m_StepTime < k_DecelTime)
                            return;
                        SetLocomotion(WalkerSlotId);
                    }
                    if (m_StepTime < k_TestCoast)
                        return;
                    Vector3 fwd = new Vector3(Mathf.Cos(m_TestYaw), 0f, -Mathf.Sin(m_TestYaw)),
                        d = Flat(Pelvis() - m_TestFrom),
                        dc = Flat(m_TestEnd - m_TestFrom);
                    Log.Info(
                        $"[G1] WT {m_TestIdx + 1}/{s_WalkerTests.Length} {(c.Rot ? "rotator" : "walker")} vx {c.Vx:F2} vy {c.Vy:F2} yaw {c.Yaw:F2} for {c.T:F2} s"
                            + $" -> while commanded: fwd {Vector3.Dot(dc, fwd):F3} m, left {Vector3.Dot(dc, LeftOf(fwd)):F3} m, turned {Mathf.WrapToPi(m_TestEndYaw - m_TestYaw) * Mathf.Rad2Deg:F1} deg"
                            + $" | total after coasting {k_TestCoast:F1} s: fwd {Vector3.Dot(d, fwd):F3} m, left {Vector3.Dot(d, LeftOf(fwd)):F3} m, turned {Mathf.WrapToPi(YawOf(Heading()) - m_TestYaw) * Mathf.Rad2Deg:F1} deg"
                            + $" (asked fwd {c.Vx * c.T:F2} m, left {c.Vy * c.T:F2} m, turn {c.Yaw * c.T * Mathf.Rad2Deg:F0} deg)"
                            + $" | feet moved L {Flat(BodyPos(m_Mujoco!.GetBodyID("left_ankle_roll_link")) - m_TestFootL).Length():F3} m, R {Flat(BodyPos(m_Mujoco.GetBodyID("right_ankle_roll_link")) - m_TestFootR).Length():F3} m"
                    );
                    if (++m_TestIdx < s_WalkerTests.Length)
                        Advance(Phase.WalkerTest);
                    else
                    {
                        Log.Info("[G1] walker test done");
                        Advance(Phase.Done);
                    }
                    return;
            }
        }

        // After a failure: zero commands while the rotator is still active, hand back to the walker (G9), fold the arms, then go again or stop.
        private void StepRecover()
        {
            SendHands(Fold, false);
            if (m_Active == RotatorSlotId)
            {
                if (m_StepTime < k_DecelTime)
                    return;
                SetLocomotion(WalkerSlotId);
                NextStep();
            }
            else if (!Gliding() && m_StepTime >= k_Settle)
                Advance(m_Retry && RepeatLoop ? Phase.FindBox : Phase.Failed);
        }

        // While holding: keep squeezing, track the hand targets, boost the force if the box slips, fail if it slides out.
        private bool Hold(float ts)
        {
            Squeeze(ts, RequiredForce());
            SendHands(HandTarget, false);
            LogGrip(ts);
            float slip = PalmMidY() - BoxCenter(m_Box).Y - m_Y0; // + = box sliding down between the palms
            if (slip > k_DropTol)
            {
                Fail($"box slipped out of the hands ({slip:F3} m)");
                return false;
            }
            if (slip > k_SlipTol)
                m_Boost = Math.Min(m_Boost + ts, 2f);
            return true;
        }

        // Squeeze per hand, on the palm's own force only (fingertips can't hold the palm off the face): the gap closes at full rate until the
        // palm touches, then presses toward k_SqueezeMargin x target (admittance). Index + middle curl in only once the palm is on its face.
        private void Squeeze(float ts, float target)
        {
            ReadForces();
            foreach (Arm a in m_Arms)
            {
                a.Gap = Mathf.Clamp(
                    a.Gap
                        - Mathf.Clamp(
                            k_DepthGain * (k_SqueezeMargin * target - a.PalmForce),
                            -k_MaxSqueezeRate,
                            k_MaxSqueezeRate
                        ) * ts,
                    -k_MaxSqueezeDepth,
                    ApproachGap + k_Widen
                );
                if (!a.PalmOn && a.PalmForce >= k_TouchForce)
                {
                    a.PalmOn = true;
                    Log.Info(
                        $"[G1] GR {a.Name} palm on its face in {m_Phase} ({a.PalmForce:F1} N, gap {a.Gap:F3} m, fingers {a.Force - a.PalmForce:F1} N) -> fingers curl in"
                    );
                }
                if (a.PalmOn)
                    a.Curl = Math.Min(a.Curl + k_CurlRate * ts, k_MaxCurl);
            }
        }

        // Tared contact force of each hand: palm + fingers (the squeeze on the side face, with the palm's share apart) and thumb (front face).
        private void ReadForces()
        {
            foreach (Arm a in m_Arms)
            {
                a.Force = a.ThumbForce = a.PalmForce = 0f;
                for (int i = 0; i < a.Geoms.Length; i++)
                {
                    float f = Math.Max(0f, m_Mujoco!.GetGeomContactForce(a.Geoms[i]) - a.Base[i]);
                    if (a.IsThumb[i])
                        a.ThumbForce += f;
                    else
                        a.Force += f;
                    if (a.IsPalm[i])
                        a.PalmForce += f;
                }
            }
        }

        // Total tared contact of both hands, thumbs included.
        private float Contact() =>
            m_Arms[0].Force + m_Arms[1].Force + m_Arms[0].ThumbForce + m_Arms[1].ThumbForce;

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
                float speed = BoxSpeed(i),
                    dist = Flat(BoxCenter(i) - Pelvis()).Length();
                if (speed > 0.2f)
                    m_Fails[i] = 0; // a box on the move (dropped, respawned) gets fresh attempts
                if (
                    Math.Abs(ChuteGap(i)) < 0.02f
                    && speed < k_RestSpeed
                    && (
                        !WalkToBox || ReachShort(ChuteEntity!, BoxCenter(i), StandDistance) <= 0.05f
                    )
                    && m_Fails[i] < MaxBoxAttempts
                    && dist < bestDist
                )
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
                if (
                    !m_Mujoco.GetGeomDescriptor(
                        g,
                        out int body,
                        out int type,
                        out _,
                        out _,
                        out _,
                        out Vector3 half
                    )
                    || type != 6
                )
                    continue; // 6 = mjGEOM_BOX
                uint b = (uint)body;
                if (
                    (m_Mujoco.GetBodyNameByIndex(b) ?? "").StartsWith("lucky_body_")
                    && m_Mujoco.GetBodyFreeJointIndex(b) < m_Mujoco.ModelNjnt
                )
                    m_Boxes.Add((b, g, half));
            }
        }

        // Nearest free slot on the table top: along its near edge, PlaceEdgeInset in from the edge and ends, PlaceSpacing apart. False if all taken.
        private bool FreeSlot(out Vector3 slot)
        {
            TransformComponent t = PlaceTarget!.Transform;
            Vector3 face = FaceOf(PlaceTarget),
                along = LeftOf(face);
            float halfLen = 0.5f * Math.Max(t.WorldScale.X, t.WorldScale.Z),
                halfWidth = 0.5f * Math.Min(t.WorldScale.X, t.WorldScale.Z);
            Vector3 line = TopOf(PlaceTarget) - face * (halfWidth - PlaceEdgeInset);
            float best = float.MaxValue;
            slot = line;
            for (
                float s = -halfLen + PlaceEdgeInset;
                s <= halfLen - PlaceEdgeInset;
                s += PlaceSpacing
            )
            {
                Vector3 p = line + along * s;
                bool taken = false;
                for (int i = 0; i < m_Boxes.Count; i++)
                    taken |=
                        i != m_Box
                        && Flat(BoxCenter(i) - p).Length() < 0.5f * PlaceSpacing
                        && Math.Abs(BoxCenter(i).Y - p.Y) < 0.3f;
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
            var names = new List<string>();
            var palm = new List<bool>();
            for (uint g = 0; g < m_Mujoco!.ModelNgeom; g++)
            {
                if (
                    !m_Mujoco.GetGeomDescriptor(
                        g,
                        out int body,
                        out _,
                        out _,
                        out int contype,
                        out int conaffinity,
                        out _
                    )
                    || (contype | conaffinity) == 0
                )
                    continue;
                string name = m_Mujoco.GetBodyNameByIndex((uint)body) ?? "";
                if (name != a.Name + "_wrist_yaw_link" && !name.StartsWith(a.Name + "_hand_"))
                    continue;
                geoms.Add(g);
                thumb.Add(name.StartsWith(a.Name + "_hand_thumb"));
                palm.Add(name == a.Name + "_wrist_yaw_link");
                names.Add($"{name}#{g}");
            }
            a.Geoms = geoms.ToArray();
            a.IsThumb = thumb.ToArray();
            a.IsPalm = palm.ToArray();
            a.GeomName = names.ToArray();
        }

        // Grip frame from the box as it is now: centre, its own horizontal axis closest to near (the heading, then the last forward so it
        // never flips) as forward, squeeze axis to its left, half extents.
        private void SetGripFrame(Vector3 near)
        {
            m_Mujoco!.GetGeomPoseMj(m_Boxes[m_Box].Geom, out _, out Quaternion q);
            Quaternion r = m_MjToHz * q;
            float best = 0f;
            m_Fwd = near;
            foreach (
                Vector3 axis in new[] { r * Vector3.Right, r * Vector3.Up, r * Vector3.Forward }
            )
            {
                float d = Vector3.Dot(Flat(axis), near); // unnormalised, so a near-vertical axis never wins
                if (Math.Abs(d) <= best)
                    continue;
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
            Vector3 lat = m_Carrying ? BodyDir(m_Lat) : m_Lat,
                fwd = m_Carrying ? BodyDir(m_Fwd) : m_Fwd;
            Vector3 t =
                c
                + lat * (a.Side * (m_HalfW + PalmOffset + a.Gap))
                + fwd * (FrontInset - m_HalfD)
                + Vector3.Up * (GripHeight(a) - m_HalfH + m_Lift + a.Raise);
            if (!m_Carrying && m_Phase is Phase.PreGrasp or Phase.Squeeze)
                t.Y = Math.Max(t.Y, SurfaceY(t) + GripHeight(a));
            return t;
        }

        // Grasp-site height over the box bottom that leaves BeltClearance under the palm's lower edge (the left hand 2 x k_ThumbStagger higher),
        // plus any raise from the reach plan.
        private float GripHeight(Arm a) =>
            BeltClearance + PalmBelow(a) + (1f + a.Side) * k_ThumbStagger + m_GripLift;

        // How far the palm's lower edge hangs below the grasp site (hand upright): left 0.034 m, right 0.054 m.
        private static float PalmBelow(Arm a) => k_PalmHalf - a.Side * k_SiteOffset;

        // Front-bottom corner of the box on one side (world, from the grip frame).
        private Vector3 Corner(float side) =>
            m_Center + m_Lat * (side * m_HalfW) - m_Fwd * m_HalfD - Vector3.Up * m_HalfH;

        // Thumbs flared back (open) except while gripping, when they swing straight across the front face; index + middle follow a.Curl while
        // gripping (Squeeze curls them in) and ease straight otherwise. Everything moves at a set rate - nothing snaps into the box.
        private void PoseHands(float ts)
        {
            bool grip = Gripping();
            foreach (Arm a in m_Arms)
            {
                if (a.Hand.Length < 4)
                    continue;
                a.Thumb += Mathf.Clamp(
                    (grip ? 0f : k_ThumbFlare) - a.Thumb,
                    -k_ThumbRate * ts,
                    k_ThumbRate * ts
                );
                if (!grip)
                    a.Curl = Math.Max(0f, a.Curl - 2f * k_CurlRate * ts);
                a.Hand[0].Value = -a.Side * a.Thumb;
                a.Hand[1].Value = 0.0;
                a.Hand[2].Value = a.Hand[3].Value = -a.Side * a.Curl;
            }
        }

        // Height of the conveyor's top plane under point p.
        private float SurfaceY(Vector3 p) =>
            m_ChuteTop.Y
            - (m_ChuteN.X * (p.X - m_ChuteTop.X) + m_ChuteN.Z * (p.Z - m_ChuteTop.Z)) / m_ChuteN.Y;

        // Wrist servo: turns each hand toward its zero-pose frame on the grip axes - fingers (X) along forward, Y along the squeeze axis, so the
        // left palm (-Y) and right palm (+Y) face the box; heading axes when not gripping. Closed on the measured hand frame, whatever the IK does.
        // A hand still climbing out from low down keeps its wrist still (turning there would swing the fingers into the belt).
        private void OrientWrists(float ts)
        {
            if (m_Phase == Phase.Ready && m_Step == 1)
                return; // the ready-pose bend drives the wrists
            bool grip = m_Phase >= Phase.Ready && m_Phase <= Phase.Release;
            Vector3 x = grip ? (m_Carrying ? BodyDir(m_Fwd) : m_Fwd) : Heading(),
                y = LeftOf(x),
                z = Vector3.Cross(x, y);
            foreach (Arm a in m_Arms)
            {
                if (a.Joints.Length < 7 || a.Rising || SiteWorld(a.Site).Y < a.Cmd.Y - k_Snap)
                    continue;
                Quaternion r = BodyRot(a.WristLink[2]);
                Vector3 cy = r * s_Axes[1],
                    w =
                        (
                            Vector3.Cross(r * s_Axes[0], x)
                            + Vector3.Cross(cy, y)
                            + Vector3.Cross(r * s_Axes[2], z)
                        ) * 0.5f; // rotation error
                a.PalmErr = Mathf.Acos(Mathf.Clamp(Vector3.Dot(cy, y), -1f, 1f)) * Mathf.Rad2Deg;
                for (int i = 0; i < 3; i++)
                {
                    m_Mujoco!.GetActuatorCtrlRange(
                        (uint)a.Joints[4 + i].ActuatorID,
                        out float lo,
                        out float hi
                    );
                    a.WristQ[i] = Mathf.Clamp(
                        a.WristQ[i]
                            + Mathf.Clamp(
                                k_WristGain * Vector3.Dot(BodyRot(a.WristLink[i]) * s_Axes[i], w),
                                -k_WristRate,
                                k_WristRate
                            ) * ts,
                        lo,
                        hi
                    );
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
            if (m_PelvisId == uint.MaxValue)
                return;
            bool grip = Gripping();
            foreach (Arm a in m_Arms)
            {
                Vector3 goal = target(a);
                if (newMove)
                {
                    a.Cmd = SiteWorld(a.Site);
                    a.T = a.Tau = 0f;
                }
                if (!grip && !newMove && a.Tau >= a.T && target.Method.Name == nameof(Fold))
                {
                    // Folded: follow the body's fold point with a k_FoldSmooth lag - drift is followed, bounces never reach the arms (and never
                    // come back through them into the walker), and no path is re-planned.
                    a.Cmd += (goal - a.Cmd) * (1f - (float)Math.Exp(-m_Dt / k_FoldSmooth));
                    a.Direct = true;
                    a.Rising = false;
                    SendTarget(a);
                    continue;
                }
                bool direct = grip || (a.Tau >= a.T && (goal - a.Cmd).Length() <= k_Snap);
                if (m_Phase == Phase.PreGrasp && direct != a.Direct)
                    Log.Info(
                        $"[G1] PG {a.Name} -> {(direct ? $"tracking its goal directly ({(goal - a.Cmd).Length():F3} m away)" : $"path to its goal ({(goal - a.Cmd).Length():F3} m away)")}"
                    );
                a.Direct = direct;
                if (direct)
                {
                    a.Cmd = goal;
                    a.T = a.Tau = 0f;
                    a.Rising = false;
                }
                else
                {
                    if (a.Tau >= a.T)
                        PlanSegment(a, goal);
                    a.Tau = Math.Min(a.Tau + m_Dt, a.T);
                    float u = a.Tau / a.T;
                    a.Cmd = a.From + (a.To - a.From) * (u * u * u * (10f + u * (6f * u - 15f))); // min-jerk position profile
                }
                SendTarget(a);
            }
            if (!newMove)
                return;
            if (m_Phase == Phase.PreGrasp)
                Log.Info(
                    $"[G1] PG IK re-seed (Start, {k_SeedBlend:F1} s blend from the current pose), paths re-planned from the live hands"
                );
            m_Robot!.SetInputFloat(s_Duration, k_SeedBlend);
            m_Robot.SetInputTrigger(s_Start);
        }

        // Sends a hand's IK target and elbow pole. Cap: a target beyond k_MaxReach of the shoulder is pulled in toward it (logged when the cap
        // starts and stops).
        private void SendTarget(Arm a)
        {
            Vector3 d = a.Cmd - BodyPos(a.Shoulder);
            float max = k_MaxReach * k_ArmLength;
            if (d.Length() > max != a.Capped)
                Log.Warn(
                    $"[G1] {a.Name} IK target {(d.Length() > max ? $"{d.Length() / k_ArmLength * 100f:F0}% of the arm away -> pulled in to {k_MaxReach * 100f:F0}%" : "back within reach")} in {m_Phase}"
                );
            a.Capped = d.Length() > max;
            m_Robot!.SetInputVector3(
                a.Target,
                a.Capped ? a.Cmd - d * (1f - max / d.Length()) : a.Cmd
            );
            m_Robot.SetInputVector3(a.Pole, BodyPoint(Mirror(PoleOffset, a.Side)));
        }

        // Next straight segment toward goal. Pure vertical moves go straight. Near a box (NearBox) and below m_SafeY, a long move across or a big climb first rises
        // straight up to m_SafeY (wrist held). A move that ends lower first crosses at the current height, then the next segment comes straight
        // down. Anything else (short local moves, climbs above m_SafeY) goes straight.
        private void PlanSegment(Arm a, Vector3 goal)
        {
            float across = Flat(goal - a.Cmd).Length();
            a.Rising =
                across > 0.02f
                && (NearBox(a.Cmd) || NearBox(goal)) // away from the boxes a move just goes straight
                && a.Cmd.Y < m_SafeY - 0.005f
                && (across > k_LocalMove || goal.Y > a.Cmd.Y + k_LocalMove);
            a.From = a.Cmd;
            a.To =
                across <= 0.02f ? goal
                : a.Rising ? new Vector3(a.Cmd.X, m_SafeY, a.Cmd.Z)
                : goal.Y < a.Cmd.Y - 0.02f ? new Vector3(goal.X, a.Cmd.Y, goal.Z)
                : goal;
            a.T = Math.Max(k_MinSegment, (a.To - a.From).Length() / HandMoveSpeed);
            a.Tau = 0f;
            if (m_Phase == Phase.PreGrasp)
                Log.Info(
                    $"[G1] PG {a.Name} new segment ({(across <= 0.02f ? "straight to the goal (vertical)" : a.Rising ? "straight up to the travel height" : a.To.Y > goal.Y + 0.001f ? "across at this height, down next" : "straight to the goal")}): {F(a.From)} -> {F(a.To)}, {(a.To - a.From).Length():F3} m in {a.T:F2} s; goal {F(goal)} {(goal - a.To).Length():F3} m beyond"
                );
        }

        // Point within k_BoxNear (horizontally) of a box on the conveyor.
        private bool NearBox(Vector3 p)
        {
            for (int i = 0; i < m_Boxes.Count; i++)
                if (Math.Abs(ChuteGap(i)) < 0.05f && Flat(BoxCenter(i) - p).Length() < k_BoxNear)
                    return true;
            return false;
        }

        // A hand is still running a path segment.
        private bool Gliding() => m_Arms[0].Tau < m_Arms[0].T || m_Arms[1].Tau < m_Arms[1].T;

        // The hands hold (or are closing on) the box: targets tracked directly, thumbs and fingers in their grip pose.
        private bool Gripping() =>
            m_Phase
                is Phase.Squeeze
                    or Phase.Holding
                    or Phase.Lift
                    or Phase.PullIn
                    or Phase.TurnToTable
                    or Phase.WalkToTable
                    or Phase.FaceTable
                    or Phase.Place
                    or Phase.Reposition;

        // Safe travel height for the hands: k_SafeMargin over the conveyor top and the boxes on it.
        private void SetSafeY()
        {
            m_SafeY = m_ChuteTop.Y;
            for (int i = 0; i < m_Boxes.Count; i++)
                if (Math.Abs(ChuteGap(i)) < 0.05f)
                    m_SafeY = Math.Max(m_SafeY, BoxCenter(i).Y + HalfExtent(i, Vector3.Up));
            m_SafeY += k_SafeMargin;
        }

        // Closed-loop walk to a point: steer at it while far (if allowed); otherwise step straight onto it (forward + sideways, no turning,
        // slowing near it) so the robot never circles it. True within tol.
        private bool WalkTo(Vector3 goal, float tol, bool steer)
        {
            Vector3 to = Flat(goal - Pelvis()),
                fwd = Heading();
            float err = Mathf.WrapToPi(YawOf(to) - YawOf(fwd));
            m_ToGo = to.Length();
            if (m_ToGo <= tol)
                return true;
            if (steer && m_ToGo > k_NearRadius)
                Drive(
                    Math.Abs(err) < 0.6f ? WalkSpeed : 0f,
                    0f,
                    Mathf.Clamp(err * k_YawGain, -k_MaxYawRate, k_MaxYawRate)
                );
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
            if (++m_WalkRetries > MaxWalkRetries)
            {
                Fail($"walk stalled {m_ToGo:F2} m from the goal after {MaxWalkRetries} re-turns");
                return;
            }
            Log.Warn(
                $"[G1] walk stalled {m_ToGo:F2} m from the goal -> turning toward it again ({m_WalkRetries}/{MaxWalkRetries})"
            );
            Advance(turn);
        }

        // Turn in place with the rotator to the heading given when the turn starts: settle, swap in, ramped P-turn (at least MinTurnRate,
        // doubled on a retry after a stall), stop, swap back, settle (G9). True when done; a turn still stalled after MaxTurnTries fails.
        private bool Turn(float yaw)
        {
            if (m_Step == 0)
                m_TurnYaw = yaw; // aim fixed once the turn starts, so body sway doesn't move it
            float err = Mathf.WrapToPi(m_TurnYaw - YawOf(Heading()));
            switch (m_Step)
            {
                case 0: // settle on the walker; no turn needed if already facing
                    if (m_StepTime < k_Settle)
                        return false;
                    if (Math.Abs(err) < TurnTolerance)
                        return true;
                    SetLocomotion(RotatorSlotId);
                    m_TurnTries = 1;
                    break;
                case 1: // turn, yaw rate ramped in from standstill
                    Drive(
                        0f,
                        0f,
                        Math.Sign(err)
                            * Mathf.Clamp(
                                Math.Abs(err) * k_YawGain,
                                Math.Min(MinTurnRate * m_TurnTries, k_TurnRate),
                                k_TurnRate
                            )
                            * Math.Min(1f, m_StepTime / k_TurnRampIn)
                    );
                    if (Math.Abs(err) <= TurnTolerance)
                        break;
                    if (!Stalled(Math.Abs(err), k_TurnProgress, PhaseTimeout))
                        return false;
                    if (m_TurnTries >= MaxTurnTries)
                    {
                        Fail(
                            $"turn stalled {err * Mathf.Rad2Deg:F0} deg short after {MaxTurnTries} tries"
                        );
                        return false;
                    }
                    Log.Warn(
                        $"[G1] turn stalled {err * Mathf.Rad2Deg:F0} deg short -> turning again harder ({++m_TurnTries}/{MaxTurnTries})"
                    );
                    m_StepTime = 0f;
                    ResetWatch();
                    return false;
                case 2: // zero yaw while the rotator is still active, then hand back to the walker
                    if (m_StepTime < k_DecelTime)
                        return false;
                    SetLocomotion(WalkerSlotId);
                    break;
                default: // settle on the walker
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
            m_DbgTimer = PreGraspLogInterval; // first PreGrasp line on the first frame
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
            if (metric < m_Best - minGain)
            {
                m_Best = metric;
                m_SinceBest = 0f;
            }
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
            string s =
                $"[G1] {m_Phase}: pelvis ({p.X:F2}, {p.Y:F2}, {p.Z:F2}) yaw {YawOf(Heading()) * Mathf.Rad2Deg:F0} deg | cmd vx={m_Cmd.X:F2} vy={m_Cmd.Y:F2} yaw={m_Cmd.Z:F2}";
            if (
                m_Phase
                is Phase.FaceBox
                    or Phase.WalkToBox
                    or Phase.Reposition
                    or Phase.TurnToTable
                    or Phase.WalkToTable
            )
                s += $" | goal ({m_Stand.X:F2}, {m_Stand.Z:F2}), to go {m_ToGo:F2} m";
            if (m_Phase == Phase.FindBox)
                for (int i = 0; i < m_Boxes.Count; i++)
                    s +=
                        $" | box {i}: {Flat(BoxCenter(i) - p).Length():F2} m away, gap {ChuteGap(i):F3} m, speed {BoxSpeed(i):F2}"
                        + (
                            WalkToBox
                                ? $", out of reach by {Math.Max(0f, ReachShort(ChuteEntity!, BoxCenter(i), StandDistance)):F2} m"
                                : ""
                        )
                        + (m_Fails[i] >= MaxBoxAttempts ? ", skipped until it moves" : "");
            else if (m_Box >= 0)
                s += $" | box {m_Box} at {BoxCenter(m_Box)}";
            if (m_Phase >= Phase.Ready && m_Phase <= Phase.Release)
            {
                s += $" | {GripInfo()}";
                foreach (Arm a in m_Arms)
                {
                    bool placed = Placed(a, out float side, out float front, out float up);
                    s +=
                        $" | {a.Name} err {HandError(a):F3} m, force {a.Force:F1} N, thumb {a.ThumbForce:F1} N, gap {a.Gap:F3} m, curl {a.Curl:F2} rad, palm {a.PalmErr:F0} deg off"
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
            float target = (HandTarget(m_Arms[0]) - HandTarget(m_Arms[1])).Length(),
                actual = (SiteWorld(m_Arms[0].Site) - SiteWorld(m_Arms[1].Site)).Length();
            float Under(Arm a) => SiteWorld(a.Site).Y - PalmBelow(a) - (c.Y - m_HalfH); // palm's lower edge over the box bottom
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
            ArmIK(true); // in case it failed mid-bend
            Advance(Phase.Recover);
            SendHands(Fold, true); // after the phase change, so hands that were gripping still leave along the safe path
        }

        // Surfaces (conveyor, table) are unit-cube meshes: centre of the top face, and the direction that faces one square
        // (across its long horizontal axis) from the robot's side.
        private static Vector3 TopOf(Entity e) =>
            e.Transform.WorldTranslation
            + e.Transform.WorldRotationQuat * Vector3.Up * (0.5f * e.Transform.WorldScale.Y);

        private Vector3 FaceOf(Entity e)
        {
            TransformComponent t = e.Transform;
            Vector3 across = FlatDir(
                t.WorldRotationQuat
                    * (t.WorldScale.X > t.WorldScale.Z ? Vector3.Forward : Vector3.Right)
            );
            return Vector3.Dot(across, t.WorldTranslation - Pelvis()) >= 0f ? across : across * -1f;
        }

        // Pelvis spot to work on point p of a surface: reach behind p along dir (horizontal), pushed back so the pelvis stays at least
        // EdgeClearance outside the surface's near edge and the legs never hit it.
        private Vector3 StandFor(Entity surface, Vector3 p, float reach, Vector3 dir)
        {
            Vector3 face = FaceOf(surface),
                stand = p - dir * reach;
            float clear =
                Vector3.Dot(surface.Transform.WorldTranslation - stand, face)
                - 0.5f * Math.Min(surface.Transform.WorldScale.X, surface.Transform.WorldScale.Z);
            return clear >= EdgeClearance ? stand : stand - face * (EdgeClearance - clear);
        }

        // How much farther than reach the surface edge forces the robot to stand from p (> 0: p is out of reach by that much).
        private float ReachShort(Entity surface, Vector3 p, float reach) =>
            Vector3.Dot(p - StandFor(surface, p, reach, FaceOf(surface)), FaceOf(surface)) - reach;

        // How far height y lies beyond ArmReach below the higher shoulder (> 0: out of reach from any stand point).
        private float LowBy(float y) =>
            Math.Max(BodyPos(m_Arms[0].Shoulder).Y, BodyPos(m_Arms[1].Shoulder).Y) - y - ArmReach;

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

        private float ChuteGap(int i) =>
            Vector3.Dot(BoxCenter(i) - m_ChuteTop, m_ChuteN) - HalfExtent(i, m_ChuteN);

        private float HalfExtent(int i, Vector3 dir)
        {
            m_Mujoco!.GetGeomPoseMj(m_Boxes[i].Geom, out _, out Quaternion q);
            Quaternion r = m_MjToHz * q;
            Vector3 h = m_Boxes[i].Half;
            return Math.Abs(Vector3.Dot(dir, r * Vector3.Right)) * h.X
                + Math.Abs(Vector3.Dot(dir, r * Vector3.Up)) * h.Y
                + Math.Abs(Vector3.Dot(dir, r * Vector3.Forward)) * h.Z;
        }

        // Squeeze force per hand that holds the box: factor * m*g / (2*mu), boosted while slipping.
        private float RequiredForce() =>
            GripSafetyFactor
            * m_Mujoco!.GetBodyMass(m_Boxes[m_Box].Body)
            * 9.81f
            / (2f * BoxFriction)
            * m_Boost;

        // Distance from a hand's grasp site to its target.
        private float HandError(Arm a) => (SiteWorld(a.Site) - HandTarget(a)).Length();

        // Average height of the two grasp sites.
        private float PalmMidY() =>
            0.5f * (SiteWorld(m_Arms[0].Site).Y + SiteWorld(m_Arms[1].Site).Y);

        // MuJoCo poses in Hazel world coordinates: axis swap + scene translation, as the reference project did (PLAN.md G22).
        private Vector3 SiteWorld(uint site) =>
            m_MjToHz * m_Mujoco!.GetSitePositionMj(site) + m_MjOffset;

        private Vector3 BodyPos(uint body)
        {
            m_Mujoco!.GetBodyPoseMj(body, out Vector3 p, out _, out _, out _);
            return m_MjToHz * p + m_MjOffset;
        }

        private Quaternion BodyRot(uint body)
        {
            m_Mujoco!.GetBodyPoseMj(body, out _, out Quaternion q, out _, out _);
            return m_MjToHz * q;
        }

        // Pelvis position (world) and horizontal heading, from MuJoCo.
        private Vector3 Pelvis() => BodyPos(m_PelvisId);

        // How fast the body carries a hand held out in front: pelvis speed + its turn (and sway) rate x k_HandLever (m/s).
        private float BodyMotion()
        {
            m_Mujoco!.GetBodyPoseMj(m_PelvisId, out _, out _, out Vector3 v, out Vector3 w);
            return v.Length() + k_HandLever * w.Length();
        }

        private Vector3 Heading() => FlatDir(BodyRot(m_PelvisId) * Vector3.Right);

        // Pelvis coords (forward, up, left) <-> world, using the pelvis heading only so the grip ignores body sway.
        private Vector3 BodyDir(Vector3 b) =>
            Heading() * b.X + Vector3.Up * b.Y + LeftOf(Heading()) * b.Z;

        private Vector3 BodyPoint(Vector3 b) => Pelvis() + BodyDir(b);

        private Vector3 ToBody(Vector3 v) =>
            new Vector3(Vector3.Dot(v, Heading()), v.Y, Vector3.Dot(v, LeftOf(Heading())));

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
