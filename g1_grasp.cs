using System;
using System.Collections.Generic;
using Hazel;

namespace G1Pipe
{
    // Picks up each box resting on the conveyor with both hands, places it on the table to the left, and comes back for the next.
    // FindBox -> FaceBox -> WalkToBox -> FaceChute -> Approach -> Ready -> PreGrasp -> Squeeze -> (DragIn) -> Lift -> PullIn -> TurnToTable -> WalkToTable -> FaceTable -> Place -> Release -> Retract -> FindBox ...
    // CatchBox: a box sliding down the tilted conveyor is tracked from the start; FindBox -> Catch puts the downhill hand's palm in its path
    // ahead of where the box will be by the time the hand gets there, and the box stops against it. The robot catches from where it stands.
    // Caught, it goes straight into the grasp: the stopper holds still on the box (m_Holder) while the other hand reaches in front of it and
    // slides in along its side face (PreGrasp), then both squeeze and the usual Lift -> ... follows.
    // The "Phases" checkboxes switch groups of phases off for testing. Every phase has a progress watchdog: a stall triggers a corrective
    // retry (step closer, re-squeeze, re-turn); when those run out the attempt fails (Recover) and the next box is tried; a box that fails
    // MaxBoxAttempts times is skipped until it moves.
    public partial class G1BoxGrasp : Entity
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
            Catch,
            PreGrasp,
            Squeeze,
            DragIn,
            Holding,
            Lift,
            PullIn,
            BackOff,
            TurnToTable,
            WalkToTable,
            FaceTable,
            Place,
            Release,
            Retract,
            Recover,
            WalkerTest,
            ArmTest,
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
                Force; // air between palm and side face (m, - = squeezing), height above contact (m), tared force (N)
            public MujocoActuator[] Joints = new MujocoActuator[0]; // shoulder pitch, roll, yaw, elbow (IK), wrist roll, pitch, yaw (script)
            public uint[] WristLink = new uint[0]; // the links those joints turn
            public float[] WristQ = new float[0]; // wrist ctrl targets (rad)
            public bool WristHeld; // wrists held where they were when the grip was confirmed
            public uint[] WristAdr = new uint[0]; // wrist joints' qpos addresses (measured angles for the log)
            public uint[] ArmAdr = new uint[0]; // shoulder pitch, roll, yaw, elbow qpos addresses (measured angles for the log)
            public float[] LastArmQ = new float[4]; // arm angles (rad) at the last GR line, for their rates
            public Vector3 LastSite; // grasp site in pelvis coords at the last GR line, for its velocity
            public float PalmErr; // deg between the palm normal and the squeeze axis
            public MujocoActuator[] Hand = new MujocoActuator[0]; // thumb 1, thumb 2, index 0, middle 0 (script-owned)
            public bool[] IsThumb = new bool[0]; // which hand geoms are thumb links
            public bool[] IsPalm = new bool[0]; // which hand geoms are the palm (wrist_yaw_link)
            public float ThumbForce,
                PalmForce; // tared thumb contact force on the front face; palm-only share of Force (N)
            public bool PalmOn; // the palm is on its face now (fingers may curl only while it is)
            public float PalmLow; // s the palm has been under its touch force (it counts as off after k_PalmHold)
            public bool Capped; // its IK target is being pulled in to MaxReach of the shoulder
            public float Thumb,
                Tip,
                Curl; // thumb swing back from straight (- = in), thumb tip bend, index + middle curl (rad; right hand mirrored)
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
        private static readonly string[] s_LegsOnly = s_LegsAndWaistOnly[..^3]; // the walker's joints while LockWaist holds the waist

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
        private const float k_CarryRamp = 0.5f,
            k_WalkCoast = 0.10f,
            k_BackCoast = 0.4f,
            k_CarryTurnTol = 0.26f,
            k_TurnEase = 0.35f,
            k_TurnFloor = 0.7f,
            k_SteadySpin = 0.2f,
            k_SteadyMax = 2f,
            k_TableClear = 0.05f,
            k_MaxRaise = 0.10f; // s every walk command ramps in (and a carry turn out) over while carrying; m the body coasts on after walking (at k_FwdCmd); s of back-off speed it coasts on after backing off (0.17 m from 0.4 m/s); rad the turn to the table may stay off (the walk steers); rad before its stop a carry turn eases its command down over, to this fraction of CarryTurnCmd (lower stalls); rad/s the box may spin in the hands before a carry turn or walk starts, s it waits for that at most; m the carried box's bottom stays over the table top, m it may be raised for that
        private const float k_TurnRate = 0.7f,
            k_TurnRampIn = 0.5f; // rotator: max rad/s, ramp-in s
        private const float k_WalkProgress = 0.02f,
            k_TurnProgress = 0.02f; // m, rad: smallest gain that counts as progress
        private const float k_RestSpeed = 0.02f; // m/s, box counts as resting
        private const float k_CatchZone = 0.03f,
            k_CatchClear = 0.05f,
            k_CatchLead = 0.3f,
            k_CatchFront = 0.10f,
            k_AccelSmooth = 0.3f,
            k_CatchMargin = 0.5f,
            k_SlideSpare = 0.2f,
            k_StopSmooth = 0.1f,
            k_GiveMatch = 0.05f,
            k_GiveTouch = 0.005f; // m short of its stop spot a stopped box still counts as caught; m the box must still be short of it when the palm arrives; s the arm lags its IK target;

        private const float k_CentreSpeed = 0.05f,
            k_CentreMax = 0.15f,
            k_Centred = 0.005f; // m/s the holder pushes a caught box up the belt to the body's centre line; m at most; m off it that counts as centred

        // m the grasp site waits in front of the box's front face before sliding in (+ FrontInset; the fingertips reach ~8.5 cm past it); s smoothing of the measured box acceleration; s spare the stopper leaves itself when it must go before the body is still; s spare the slide in must have before the box comes; s easing time constant of the stopper's command near its target; m short of the palm the box is when the palm starts matching its speed; m short of the palm the box counts as on it
        private const float k_DepthGain = 0.002f,
            k_MaxSqueezeRate = 0.02f; // squeeze admittance: m/(N*s), m/s
        private const float k_SqueezeMargin = 1.25f,
            k_GripConfirm = 0.5f; // palms press toward this x the required force; grip OK: both palms on their faces and each hand (palm + fingers) >= the required force for 0.5 s
        private const float k_PalmHold = 0.3f; // s a palm must stay off its face before it counts as off (carry jostle makes it flicker)
        private const float k_LiftSpeed = 0.05f,
            k_ClearLift = 0.03f; // m/s; m the box rises before it is also pulled in toward the chest
        private const float k_DriftTol = 0.10f,
            k_DriftDone = 0.04f; // m a heavy box may walk the standing body off its spot before the walker steps it back; m short of the spot it stops
        private const float k_CarrySpeed = 0.1f; // m/s, box moves in the hands
        private const float k_PlaceLowerSpeed = 0.03f; // m/s
        private const float k_OpenSpeed = 0.3f,
            k_ReleaseGap = 0.10f,
            k_ReleaseTime = 0.5f,
            k_RetractUp = 0.10f; // m/s, m, s, m
        private const float k_Widen = 0.05f; // m, a hand landing on the box slides out at most this far
        private const float k_WristGain = 2f,
            k_WristRate = 1f; // wrist servo: rad/s per rad of palm error, cap
        private const float k_ThumbRate = 1.5f; // rad/s the thumb swings
        private const float k_PoleRate = 0.15f; // m/s the elbow hint eases out or back
        private const float k_MaxCurl = 0.8f,
            k_CurlRate = 0.5f; // rad index + middle curl at most into the side face; rad/s they curl while the grip is short
        private const float k_ThumbStagger = 0.018f,
            k_ThumbReach = 0.095f; // m, left hand raised 2x this over the right so the straight thumbs pass; m a straight thumb reaches across the front face past its palm (MJCF)
        private const float k_Snap = 0.05f,
            k_LocalMove = 0.10f,
            k_SafeMargin = 0.10f; // m: target tracked directly within this; shorter moves across go straight; travel height over belt and boxes
        private const float k_OverTableInset = 0.05f; // m in from the table top's edges the box centre must be to go straight down
        private const float k_HeadBottom = 0.30f,
            k_HeadFront = 0.08f; // m over the pelvis the head starts (MJCF head mesh 0.33, less a margin); m ahead of the pelvis its front is
        private const float k_FingerTip = 0.09f,
            k_ParkClear = 0.04f,
            k_ParkMin = 0.05f; // m the straight fingertips reach ahead of the grasp site; m a parked hand keeps them behind a passing box; m ahead of the pelvis a parked hand stays at least
        private const float k_ParkSpeed = 0.05f; // m/s a hand parked through the catch moves at most
        private const float k_FoldSmooth = 0.8f,
            k_BoxNear = 0.35f; // s lag of a folded hand behind its (swaying, body-relative) fold point; m, hands this close to a box take the safe climb
        private const float k_MinSegment = 0.5f,
            k_SeedBlend = 0.5f; // s shortest path segment; s IK re-seed blend (eases any change of arm pose)
        private const float k_PalmHalf = 0.044f,
            k_SiteOffset = 0.01f; // m (MJCF): palm half-height about the wrist axis; grasp site below (left) / above (right) it
        private const float k_StillLog = 4f,
            k_HandLever = 0.45f; // s between "waiting for the body" lines; m pelvis -> hands

        // Approach (lining the body up with the box), from the walker test: smallest commands that step (side 0.4, forward 0.6, back 0.4 m/s)
        // move ~10-20 cm per 0.25 s burst after coasting ~1 s; a 2 rad/s turn command turns in place (~57 deg/s, ~2 deg coast).
        private const float k_TurnStop = 0.1f,
            k_TurnCmd = 2f,
            k_TurnMax = 3f; // rad short of the aim a turn stops (coast); rad/s turn command; s longest turn
        private const float k_SideCmd = 0.4f,
            k_FwdCmd = 0.6f,
            k_BackCmd = 0.4f,
            k_Burst = 0.25f,
            k_LongBurst = 0.5f,
            k_ApCoast = 1.5f; // m/s burst commands; s burst (long one when far off); s to coast to a stop before measuring again
        private const int k_MaxMoves = 12; // approach corrections per attempt before reaching from where the body is

        // Reach from the front (arm test: hands accurate 0-0.15 m over the pelvis, 0.25-0.40 m ahead, >= ~0.12 m off the centre line): pre-grip
        // points in front of the box at grip height, a slide in along the side faces, and a drag in to bend the elbows for the lift.
        private const float k_PreFront = 0.10f,
            k_PreWiden = 0.03f,
            k_SlideSpeed = 0.10f; // m pre-grip sites ahead of the grip sites' spot (5 cm in front of the front face); m extra gap; m/s slide in
        private const float k_DragFrom = 0.35f,
            k_DragTo = 0.30f,
            k_DragSpeed = 0.08f,
            k_DragLag = 0.05f; // m ahead beyond which a gripped box is dragged in, to this; m/s; m the box may trail the hands
        private float PlanReach => HandLimit / 100f;
        private float MaxReach => IKTargetLimit / 100f;
        private const float k_ArmLength = 0.517f; // m shoulder -> grasp site at full stretch (MJCF)
        private const float k_Disturb = 0.02f; // m box moves logged as a disturbance
        private static readonly Vector3[] s_Axes =
        {
            new Vector3(1f, 0f, 0f),
            new Vector3(0f, 1f, 0f),
            new Vector3(0f, 0f, 1f),
        }; // MuJoCo body X, Y, Z

        [Group("Phases")]
        public bool WalkToBox = true; // walk up to the box and square up to the conveyor (off: grasp from where the robot stands)

        [Group("Phases")]
        public bool CatchBox = true; // catch boxes sliding down the conveyor from where the robot stands, stopping each with the downhill hand (off: only boxes resting on the belt are picked)

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

        [Group("Phases")]
        public bool ArmTest = false; // no grasp: both hands go to random reachable target pairs (box sizes and spots), hold, and log tracking and shake per trial

        [Group("Phases")]
        public int ArmTestTrials = 20; // target pairs per arm test run

        [Group("Phases")]
        public int ArmTestSeed = 0; // arm test random seed (0 = a new set every run; the seed used is logged so a run can be repeated)

        [Group("Phases")]
        public bool ArmTestSweep = false; // arm test: instead of random pairs, sweep height / distance / separation / offset one at a time, past the arms' limits

        [Group("Walk")]
        public float WalkSpeed = 0.3f; // m/s

        [Group("Walk")]
        public float CarryBackDistance = 0.20f; // m the body backs away from the conveyor with the box before turning to the table

        [Group("Walk")]
        public float CarrySmoothing = 0.2f; // s, the carry frame (pelvis pose the box is held in) is smoothed over this: gait bounce and sway never reach the arms (0 = off)

        [Group("Walk")]
        public float CarryTurnCmd = 2.0f; // rad/s walker yaw command for the turns with the box (the walker turns in place: no rotator swap)

        [Group("Walk")]
        public float CarryTurnRamp = 0.5f; // s a carry turn's command eases in over (smoothstep)

        [Group("Walk")]
        public float CarryWalkSpeed = 0.7f; // m/s forward command walking the box to the table (the walker steps from 0.6)

        [Group("Walk")]
        public float CarryBackSpeed = 0.4f; // m/s backward command backing away from the conveyor with the box (the walker steps from 0.4)

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

        [Group("Catch")]
        public float CatchHandSpeed = 0.5f; // m/s top speed of the stopper hand's IK target as it tracks its spot (eases in over the last few cm)

        [Group("Catch")]
        public float CatchPalmSide = 0.17f; // m out from the pelvis centre line the stopper palm stops the box, whatever its size (its centre sits half its width inside)

        [Group("Catch")]
        public float CatchShiftMax = 0.15f; // m the stop spot may move down the belt from straight ahead when the box would beat the hand there

        [Group("Catch")]
        public float CatchGive = 0.06f; // m the palm yields down the belt with the box as it arrives, braking it to a stop (0.005 = almost stiff)

        [Group("Catch")]
        public float CatchSpeed = 0.03f; // m/s, a box slower than this against the stopper palm counts as stopped

        [Group("Catch")]
        public float CatchTime = 0.3f; // s the box must stay stopped against the stopper palm to count as caught

        [Group("Arms")]
        public Vector3 FoldOffset = new Vector3(0.15f, 0.15f, 0.20f); // hand target while walking, from the pelvis: (forward, up, outward)

        [Group("Arms")]
        public Vector3 StartShoulderDeg = new Vector3(25f, 10f, 0f); // deg left shoulder at the start - pitch (+ = upper arm back), roll (+ = out), yaw; right mirrored

        [Group("Arms")]
        public float StartElbowDeg = -40f; // deg elbow at the start (0 = forearm level, + = straightening to 90, - = folding up to -60): with the shoulder above, hands start near the fold, nothing sticking out toward the conveyor

        [Group("Arms")]
        public Vector3 PoleOffset = new Vector3(-0.30f, 0.10f, 0.15f); // elbow hint from the pelvis: (forward, up, outward)

        [Group("Arms")]
        public float GripPoleOut = 0.15f; // m the elbow hint eases further out from the reach to the release: the IK closes the hands by turning the shoulders instead of swinging the upper arms into the torso

        [Group("Arms")]
        public Vector3 CarryOffset = new Vector3(0.24f, 0.10f, 0f); // box centre while carrying, from the pelvis: (forward, up, left)

        [Group("Arms")]
        public float HeadClearance = 0.25f; // m a box tall enough to reach head height is carried with its near face this far in front of the head

        [Group("Arms")]
        public float HandMoveSpeed = 0.2f; // m/s average along each hand path segment (min-jerk: eases in and out, peaks at ~1.9x this)

        [Group("Tolerances")]
        public float ReachTolerance = 0.05f; // m, grasp site this close to its target = in position

        [Group("Tolerances")]
        public float ReachProgress = 0.005f; // m, smallest drop in hand error that counts as progress (else the hand has stalled short)

        [Group("Tolerances")]
        public float PlacedInside = 0.015f; // m a palm may press into its side face and still count as beside the box

        [Group("Tolerances")]
        public float PlacedFront = 0.03f; // m the thumb may sit off the front face and still count as beside the box

        [Group("Tolerances")]
        public float PlacedTop = 0.01f; // m a grasp site must stay under the box top to count as beside it (negative: may ride over)

        [Group("Tolerances")]
        public float PlacedWait = 0.5f; // s hands beside the box (but not within ReachTolerance) must stop improving before the grasp

        [Group("Tolerances")]
        public float TopMargin = 0.02f; // m a raised grip keeps its grasp sites under the box top

        [Group("Tolerances")]
        public float AlignYawDeg = 8f; // deg Approach may face off the box

        [Group("Tolerances")]
        public float AlignSide = 0.06f; // m the box centre may sit off the straight-ahead line after Approach

        [Group("Tolerances")]
        public float AlignForward = 0.05f; // m the front face may sit off FaceDistance after Approach

        [Group("Tolerances")]
        public float MinSide = 0.11f; // m a grip point must stay off the body's centre line, else the reach fails

        [Group("Tolerances")]
        public float MaxGripUp = 0.15f; // m over the pelvis a grip point may sit before a warning

        [Group("Tolerances")]
        public float StillSpeed = 0.05f; // m/s at the hands under which the body counts as still

        [Group("Tolerances")]
        public float StillTime = 0.5f; // s the body must stay still before a reach or slide in

        [Group("Tolerances")]
        public float TouchForce = 2f; // N, a hand this loaded is at the box

        [Group("Tolerances")]
        public float SlipTolerance = 0.01f; // m the box may slip down the palms before the lift pauses and squeezes harder

        [Group("Tolerances")]
        public float DropTolerance = 0.05f; // m of slip that counts as the box dropped

        [Group("Tolerances")]
        public float LostDistance = 0.25f; // m the box may move from where the reach started before it counts as gone

        [Group("Arms")]
        public float LiftHeight = 0.08f; // m

        [Group("Arms")]
        public float GripSafetyFactor = 2f; // squeeze per hand = factor * m*g / (2*mu)

        [Group("Arms")]
        public float MaxSqueezeDepth = 0.03f; // m past its side face a palm's target may go while squeezing (covers arm tracking error)

        [Group("Arms")]
        public float HeldPalmForce = 20f; // N each palm presses while holding the box, fingers wrapped (more if its weight needs it, never past MaxPalmForce)

        [Group("Arms")]
        public float MaxFingerForce = 25f; // N index + middle of a hand press at most: pressing 16-22 cm out from the wrist, harder ones twist it open past its 5 Nm

        [Group("Arms")]
        public float MaxPalmForce = 40f; // N a palm presses at most: the squeeze twists the hand about the 5 Nm wrist yaw, and past it the palm flares off the face

        [Group("Arms")]
        public float GripForceLimit = 100f; // % of the required squeeze each hand must hold for the grip to count (lower it if squeezes stall short)

        [Group("Arms")]
        public bool LockWaist = true; // while the box is held the waist holds still (the walker drives the legs only): the torso can't twist and swing the box

        [Group("Arms")]
        public bool LockWrists = false; // wrist servo off: the wrists hold where they are (their zero pose if on from the start)

        [Group("Thumb")]
        public float ThumbOpenDeg = 41.25f; // deg the thumb swings back (out) while not gripping (joint limit 41.5)

        [Group("Thumb")]
        public float ThumbGripDeg = 0f; // deg the thumb swings in past straight while gripping (0 = straight across the front face; joint limit 60)

        [Group("Thumb")]
        public float ThumbTipDeg = 0f; // deg the thumb tip bends in while gripping (0 = straight; joint limit 100)

        [Group("Arms")]
        public float GripComfort = 85f; // % of the arm the grip is planned within: a grip lower than that is raised up the box (up to TopMargin under its top)

        [Group("Arms")]
        public float HandLimit = 93f; // % of the arm every hand target of a reach must lie within (from the standing pose), else the attempt fails

        [Group("Arms")]
        public float IKTargetLimit = 95f; // % of the arm from the shoulder (each frame) any IK target is pulled in to; >= 100 lets the arm lock straight

        [Group("Arms")]
        public float ArmReach = 0.50f; // m, shoulder -> grasp site at full stretch (MJCF: 0.517); a grip lower than this below the shoulders is unreachable

        [Group("Place")]
        public Entity? PlaceTarget; // the table top slab (unit-cube mesh, local +Y up); boxes go on its top face

        [Group("Place")]
        public float PlaceEdgeInset = 0.12f; // m, the table spot the robot walks to is this far in from the table's near edge (and ends)

        [Group("Place")]
        public float PlaceStandDistance = 0.33f; // m, pelvis -> slot when placing (arm reach limit)

        [Group("Recovery")]
        public float StallTime = 3f; // s without progress (hand error, distance, turn error, grip) before a phase counts as stalled

        [Group("Recovery")]
        public float PhaseTimeout = 15f; // s, hard cap on one try of a reach, squeeze, turn or step (walks use WalkTimeout)

        [Group("Recovery")]
        public int MaxBoxAttempts = 2; // failed attempts before a box is skipped (until it moves)

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
        private Arm? m_Holder; // the hand that caught the box: pinned where it caught it (no pre-grip, no opening) for the rest of the box
        private Vector3 m_HoldOff; // the holder's target offset from its HandTarget, in the grip frame (along m_Lat, m_Fwd, up)
        private Vector3 m_Downhill,
            m_Path,
            m_StopCmd; // direction a box slides down the conveyor (in its plane); catch: a point on the box's path, on the belt; the stopper's IK command
        private float m_Ahead,
            m_Shift,
            m_Give,
            m_GiveV,
            m_GiveDecel,
            m_Accel,
            m_PrevV; // catch: along-path position of the spot straight ahead of the pelvis; stop spot shift down the belt from it and give (m); give speed (m/s) and braking (m/s^2); box acceleration measured down the belt (m/s^2) and its last speed (m/s)
        private float m_PeakSpeed,
            m_PeakForce,
            m_Caught; // catch: fastest the box came, hardest it hit the stopper hand; m_Give when caught (the centring push starts there)
        private int m_Box = -1; // index of the box being handled
        private uint m_PelvisId = uint.MaxValue,
            m_TorsoId = uint.MaxValue;
        private uint[] m_WaistAdr = new uint[0]; // waist yaw, roll, pitch qpos addresses (for the log)
        private MujocoActuator[] m_Waist = new MujocoActuator[0]; // their actuators (script-held while LockWaist locks them)
        private readonly float[] m_WaistQ = new float[3]; // rad the locked waist holds
        private bool m_WaistLocked;
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
        private float m_TurnCmd; // rad/s last yaw command of the current carry turn (ramped out from it)
        private Vector3 m_BackFrom; // pelvis when backing off began
        private Vector3 m_Anchor; // pelvis spot the body holds while standing with the box (grip confirmed; Place began)
        private Vector3 m_LiftTo; // box centre (pelvis coords) at the top of the lift
        private bool m_Restoring; // the walker is stepping the body back onto m_Anchor
        private Vector2 m_RestoreDir; // that correction's one direction: (+1 forward / -1 back, 0) or (0, +1 left / -1 right)
        private int m_TurnTries,
            m_WalkRetries,
            m_SqueezeTries; // retries used in the current turn / walk goal / reach / grip
        private float m_Reach; // m, current pelvis -> box (or slot) stand distance; shrinks on failed reaches
        private bool m_Retry; // Recover goes back for the next box (else stops)
        private float m_Still; // s the body has been still without a break (BodyMotion under StillSpeed)
        private float m_PoleOut; // m the elbow hint is out past PoleOffset now (eases toward GripPoleOut while reaching and gripping)
        private float m_Slide; // m the hand targets sit ahead of the grip (- = in front of the box: the pre-grip points)
        private int m_Moves; // approach corrections made for the current attempt
        private Vector3 m_ApCmd; // approach: command of the current correction (vx, vy, yaw rate)
        private float m_ApTime,
            m_ApTurnTo; // approach: s the command runs; heading a turn aims at
        private int m_TestIdx; // walker test: command being measured
        private System.Random m_Rng = new System.Random();
        private int m_ArmIdx; // arm test: trial
        private readonly Vector3[] m_ArmGoal = new Vector3[2]; // arm test: world targets of the trial (left, right)
        private string m_ArmPlan = ""; // arm test: the trial's targets, for its result line
        private float m_ArmReached,
            m_JointMax,
            m_BodyMax; // arm test: s to arrive; deg/s fastest arm joint and m/s body motion while holding
        private readonly float[] m_ErrSum = new float[2],
            m_ErrSq = new float[2],
            m_ErrMax = new float[2];
        private readonly float[][] m_PrevQ = { new float[4], new float[4] };
        private int m_ErrN;
        private bool m_ArmCapped;
        private readonly List<(
            float Err,
            float Jit,
            float Speed,
            string Line,
            string Key
        )> m_ArmResults = new List<(float, float, float, string, string)>();
        private string m_ArmKey = ""; // arm test: sweep and value of the trial (sweep mode)

        // Arm test sweeps: (sweep, centre ahead, centre over the pelvis, centre left, hands apart, slide back to this far ahead (0 = no slide); m,
        // pelvis coords). One value changes at a time from a usual grip (0.32 ahead, 0.05 up, centred, 0.32 apart), well past the arms' limits
        // (no reach filter); "far" stretches both arms straight ahead to full reach, "pull" then slides the hands back like dragging a far box in.
        private static readonly (
            string Sweep,
            float F,
            float Up,
            float C,
            float Apart,
            float PullTo
        )[] s_ArmSweep =
        {
            ("height", 0.32f, -0.10f, 0f, 0.32f, 0f),
            ("height", 0.32f, -0.05f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.05f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.10f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.15f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.20f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.25f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.30f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.35f, 0f, 0.32f, 0f),
            ("ahead", 0.20f, 0.05f, 0f, 0.32f, 0f),
            ("ahead", 0.25f, 0.05f, 0f, 0.32f, 0f),
            ("ahead", 0.30f, 0.05f, 0f, 0.32f, 0f),
            ("ahead", 0.35f, 0.05f, 0f, 0.32f, 0f),
            ("ahead", 0.40f, 0.05f, 0f, 0.32f, 0f),
            ("ahead", 0.45f, 0.05f, 0f, 0.32f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.16f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.20f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.24f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.28f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.32f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.36f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.40f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.44f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.48f, 0f),
            ("offset", 0.32f, 0.05f, -0.15f, 0.32f, 0f),
            ("offset", 0.32f, 0.05f, -0.10f, 0.32f, 0f),
            ("offset", 0.32f, 0.05f, -0.05f, 0.32f, 0f),
            ("offset", 0.32f, 0.05f, 0f, 0.32f, 0f),
            ("offset", 0.32f, 0.05f, 0.05f, 0.32f, 0f),
            ("offset", 0.32f, 0.05f, 0.10f, 0.32f, 0f),
            ("offset", 0.32f, 0.05f, 0.15f, 0.32f, 0f),
            ("far", 0.40f, 0.05f, 0f, 0.32f, 0f),
            ("far", 0.43f, 0.05f, 0f, 0.32f, 0f),
            ("far", 0.46f, 0.05f, 0f, 0.32f, 0f),
            ("far", 0.49f, 0.05f, 0f, 0.32f, 0f),
            ("far", 0.40f, 0.10f, 0f, 0.32f, 0f),
            ("far", 0.43f, 0.10f, 0f, 0.32f, 0f),
            ("far", 0.46f, 0.10f, 0f, 0.32f, 0f),
            ("far", 0.49f, 0.10f, 0f, 0.32f, 0f),
            ("pull", 0.46f, 0.05f, 0f, 0.32f, 0.28f),
            ("pull", 0.49f, 0.05f, 0f, 0.32f, 0.28f),
            ("pull", 0.46f, 0.05f, 0f, 0.36f, 0.28f),
            ("pull", 0.49f, 0.10f, 0f, 0.36f, 0.28f),
        };

        private int ArmTrials => ArmTestSweep ? s_ArmSweep.Length : ArmTestTrials;
        private Vector4 m_ArmPull; // arm test trial slide: (slide back to this far ahead, 0 = none; up; centre left; half separation)
        private const float k_PullSpeed = 0.08f; // m/s the arm test slides the hands back (dragging a far box in)

        // Both arms' IK joints (deg): shoulder pitch / roll / yaw and elbow, with how far the elbow is from straight (0 = straight).
        private string ArmAngles()
        {
            string A(Arm a) =>
                $"{a.Name[0]} shoulder {a.Joints[0].Value * Mathf.Rad2Deg:F0}/{a.Joints[1].Value * Mathf.Rad2Deg:F0}/{a.Joints[2].Value * Mathf.Rad2Deg:F0} elbow {a.Joints[3].Value * Mathf.Rad2Deg:F0} (bent {90f - a.Joints[3].Value * Mathf.Rad2Deg:F0})";
            return $"angles {A(m_Arms[0])}, {A(m_Arms[1])} deg";
        }

        // Resets the arm test error / joint-speed / body statistics.
        private void ResetArmStats()
        {
            m_ErrN = 0;
            m_JointMax = m_BodyMax = 0f;
            m_ArmCapped = false;
            for (int s = 0; s < 2; s++)
            {
                m_ErrSum[s] = m_ErrSq[s] = m_ErrMax[s] = 0f;
                for (int i = 0; i < 4; i++)
                    m_PrevQ[s][i] = (float)m_Arms[s].Joints[i].Value;
            }
        }

        // One frame of arm test statistics: each hand's error against its reference point (target, or the path point while sliding).
        private void ArmStats(Func<Arm, Vector3> reference)
        {
            for (int s = 0; s < 2; s++)
            {
                Arm a = m_Arms[s];
                float e = (SiteWorld(a.Site) - reference(a)).Length();
                m_ErrSum[s] += e;
                m_ErrSq[s] += e * e;
                m_ErrMax[s] = Math.Max(m_ErrMax[s], e);
                for (int i = 0; i < 4; i++)
                {
                    float q = (float)a.Joints[i].Value;
                    m_JointMax = Math.Max(
                        m_JointMax,
                        Math.Abs(q - m_PrevQ[s][i]) / m_Dt * Mathf.Rad2Deg
                    );
                    m_PrevQ[s][i] = q;
                }
                m_ArmCapped |= a.Capped;
            }
            m_ErrN++;
            m_BodyMax = Math.Max(m_BodyMax, BodyMotion());
        }

        private const float k_ArmSettle = 1f,
            k_ArmHold = 2f,
            k_ArmMoveMax = 6f; // arm test: s still before a trial, s held at the targets, s a move may take
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
        private Vector3 m_CarryPos,
            m_CarryVel,
            m_LastPelvis; // carry frame: smoothed pelvis position, its smoothed velocity, last pelvis position
        private float m_CarryYaw,
            m_CarryRate,
            m_LastYaw; // carry frame: smoothed heading, its smoothed rate, last heading (rad)
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
            {
                m_Still = BodyMotion() < StillSpeed ? m_Still + ts : 0f;
                TrackCarryFrame(ts);
                float pole =
                    m_Phase >= Phase.PreGrasp && m_Phase <= Phase.Release ? GripPoleOut : 0f;
                m_PoleOut += Mathf.Clamp(pole - m_PoleOut, -k_PoleRate * ts, k_PoleRate * ts); // eased, so the arms re-fold slowly and never jolt the box
            }
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
                    StartEpisodeState();
                    Begin();
                    break;
                case Phase.FindBox:
                    SendHands(Fold, false);
                    if (ArmTest)
                    {
                        int seed =
                            ArmTestSeed != 0 ? ArmTestSeed : Environment.TickCount & 0x7fffffff;
                        m_Rng = new System.Random(seed);
                        m_ArmIdx = 0;
                        m_ArmResults.Clear();
                        Log.Info(
                            $"[G1] arm test: {(ArmTestSweep ? $"{s_ArmSweep.Length} sweep targets (height, distance, separation, offset)" : $"{ArmTestTrials} random reachable target pairs, seed {seed}")}, no grasp"
                        );
                        Advance(Phase.ArmTest);
                        break;
                    }
                    if (WalkerTest)
                    {
                        m_TestIdx = 0;
                        Log.Info($"[G1] walker test: {s_WalkerTests.Length} commands, no grasp");
                        Advance(Phase.WalkerTest);
                        break;
                    }
                    m_Box = CatchBox ? IncomingBox() : -1;
                    if (m_Box >= 0)
                    {
                        EnterCatch();
                        break;
                    }
                    m_Box = RestingBox();
                    if (m_Box < 0)
                        break; // nothing to pick yet: the status line lists every box and why
                    m_Reach = StandDistance;
                    m_SqueezeTries = m_WalkRetries = 0;
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
                case Phase.Catch:
                    StepCatch(ts);
                    break;
                case Phase.PreGrasp:
                    StepPreGrasp(ts);
                    break;
                case Phase.Squeeze:
                    StepSqueeze(ts);
                    break;
                case Phase.DragIn:
                    StepDragIn(ts);
                    break;
                case Phase.Holding:
                    Hold(ts);
                    break; // test end state: keep the grip
                case Phase.Lift:
                    StepLift(ts);
                    break;
                case Phase.PullIn:
                    if (
                        Hold(ts)
                        && MoveTowards(
                            ref m_Center,
                            new Vector3(CarryX(), CarryOffset.Y, CarryOffset.Z),
                            k_CarrySpeed * ts
                        )
                    )
                        EnterTurnToTable();
                    break;
                case Phase.BackOff:
                    if (!Hold(ts))
                        break;
                    if (m_Step == 2)
                        HoldStation(); // the back-off spot, once backed away
                    if (StepBackOff())
                        Advance(Phase.TurnToTable);
                    break;
                case Phase.TurnToTable:
                    RaiseOverTable(ts);
                    if (!Hold(ts))
                        break;
                    bool turned = WalkerTurn(YawOf(FaceOf(PlaceTarget!)), k_CarryTurnTol);
                    HoldStation(); // on the back-off spot, turns included
                    if (turned)
                        Advance(Phase.WalkToTable);
                    break;
                case Phase.WalkToTable:
                    RaiseOverTable(ts);
                    if (!Hold(ts))
                        break;
                    bool walked = CarryWalk();
                    if (m_Step != 1)
                        HoldStation(); // the back-off spot before the walk, the stand spot after it
                    if (walked)
                        Advance(Phase.FaceTable);
                    break;
                case Phase.FaceTable:
                    RaiseOverTable(ts);
                    if (!Hold(ts))
                        break;
                    bool faced = WalkerTurn(YawOf(FaceOf(PlaceTarget!)), TurnTolerance);
                    HoldStation(); // on the stand spot, turns included
                    if (faced)
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
                case Phase.ArmTest:
                    StepArmTest();
                    break;
                default:
                    SendHands(Fold, false);
                    break; // Done / Failed: stand with the arms folded
            }
            if (m_Phase == Phase.Start)
                return;
            OrientWrists(ts);
            HoldWaist();
            PoseHands(ts);
            RecordTick(ts);
        }

        // LockWaist: while the box is held (grip confirmed until Release) the walker drives the legs only and the waist holds the angles
        // it had then, so the torso can't twist on the pelvis and swing the arms and box around (given back while placing, the walker
        // swung a heavy box loose: yaw +-10 deg, pitch 8-15 deg).
        private void HoldWaist()
        {
            bool lockIt = LockWaist && m_Phase > Phase.Squeeze && m_Phase < Phase.Release;
            if (lockIt != m_WaistLocked)
            {
                m_Robot!.SetDrivenJoints(m_Active, lockIt ? s_LegsOnly : s_LegsAndWaistOnly);
                for (int i = 0; i < 3; i++)
                    m_WaistQ[i] = (float)m_Mujoco!.GetJointPosition(m_WaistAdr[i]);
                m_WaistLocked = lockIt;
                Log.Info(
                    $"[G1] waist {(lockIt ? $"locked at (yaw, roll, pitch) [{Degs(m_WaistAdr)}] deg in {m_Phase}" : $"back to the walker in {m_Phase}")}"
                );
            }
            if (m_WaistLocked)
                for (int i = 0; i < 3; i++)
                    m_Waist[i].Value = m_WaistQ[i];
        }

        // First frames (MuJoCo isn't ready in OnCreate): find the pelvis and boxes, set up the arm IK (failing loudly on a missing part), read the conveyor, fold the arms.
        private void Begin()
        {
            m_MjToHz = m_Mujoco!.GetAxisHzToMj().Conjugate;
            m_MjOffset = m_Mujoco.GetSceneTranslation();
            m_PelvisId = m_Mujoco.GetBodyID("pelvis");
            m_TorsoId = m_Mujoco.GetBodyID("torso_link");
            m_WaistAdr = Array.ConvertAll(
                s_LegsAndWaistOnly[^3..],
                j => m_Mujoco.GetJointQposAdr(m_Mujoco.GetJointID(j))
            );
            m_Waist = Array.ConvertAll(s_LegsAndWaistOnly[^3..], j => m_Mujoco.GetActuator(j)!);
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
                // Start tucked: the shoulder and elbow start pose written straight into the sim, so the spawn's forearms-forward pose
                // never swings out over the conveyor while the body drops onto its feet.
                float[] tuck =
                {
                    StartShoulderDeg.X,
                    a.Side * StartShoulderDeg.Y,
                    a.Side * StartShoulderDeg.Z,
                    StartElbowDeg,
                };
                string[] arm =
                {
                    "_shoulder_pitch_joint",
                    "_shoulder_roll_joint",
                    "_shoulder_yaw_joint",
                    "_elbow_joint",
                };
                for (int i = 0; i < 4; i++)
                {
                    float q = tuck[i] * Mathf.Deg2Rad;
                    m_Mujoco.SetQposByIndex(
                        m_Mujoco.GetJointQposAdr(m_Mujoco.GetJointID(a.Name + arm[i])),
                        q
                    );
                    acts[i].Value = q;
                }
                a.ArmAdr = Array.ConvertAll(
                    arm,
                    j => m_Mujoco.GetJointQposAdr(m_Mujoco.GetJointID(a.Name + j))
                );
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
                a.WristAdr = Array.ConvertAll(
                    new[] { "_wrist_roll_joint", "_wrist_pitch_joint", "_wrist_yaw_joint" },
                    j => m_Mujoco.GetJointQposAdr(m_Mujoco.GetJointID(a.Name + j))
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
            m_Downhill = m_ChuteN * m_ChuteN.Y - Vector3.Up; // gravity along the belt plane: length = sin(tilt)
            if (CatchBox)
            {
                if (m_Downhill.Length() < 0.01f)
                {
                    Fail(
                        "CatchBox needs the conveyor tilted (> 0.6 deg along its length) so boxes slide down it",
                        false
                    );
                    return;
                }
                m_Downhill = m_Downhill.Normalized();
                Log.Info(
                    $"[G1] catch: belt tilted {Mathf.Asin(Math.Abs(m_Downhill.Y)) * Mathf.Rad2Deg:F1} deg, boxes slide toward {F(m_Downhill)} (frictionless: {SlideAccel():F2} m/s^2)"
                );
            }
            SetSafeY();
            if (ApproachGap > 0.1f)
                Log.Warn(
                    $"[G1] ApproachGap is {ApproachGap:F2} m: each palm aims that far outside the box (arms flung sideways) - expected 0.02-0.05"
                );
            Log.Info(
                $"[G1] ready: {m_Boxes.Count} boxes, hand geoms L={m_Arms[0].Geoms.Length} R={m_Arms[1].Geoms.Length}, pelvis at {Pelvis()}"
            );
            SendHands(Fold, true);
            foreach (Arm a in m_Arms)
            {
                a.Cmd = Fold(a); // first IK targets at the fold, not where the hands were before the tuck
                SendTarget(a);
            }
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
                if (Math.Abs(off) > AlignYawDeg * Mathf.Deg2Rad)
                {
                    m_ApTurnTo = YawOf(m_Fwd);
                    m_ApCmd = new Vector3(0f, 0f, Math.Sign(off) * k_TurnCmd);
                    m_ApTime = k_TurnMax;
                    why = $"turning in place {(off > 0f ? "left" : "right")}";
                }
                else if (Math.Abs(side) > AlignSide)
                {
                    m_ApCmd = new Vector3(0f, Math.Sign(side) * k_SideCmd, 0f);
                    m_ApTime = Math.Abs(side) > 0.2f ? k_LongBurst : k_Burst;
                    why = $"side-stepping {(side > 0f ? "left" : "right")} ({m_ApTime:F2} s burst)";
                }
                else if (face > FaceDistance + AlignForward && room > 0.1f)
                {
                    m_ApCmd = new Vector3(k_FwdCmd, 0f, 0f);
                    m_ApTime = k_Burst;
                    why = $"stepping forward ({m_ApTime:F2} s burst)";
                }
                else if (face < FaceDistance - AlignForward)
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
            m_Slide = 0f;
            m_Holder = null;
            foreach (Arm a in m_Arms)
                a.Gap = ApproachGap;
            Advance(Phase.Ready);
        }

        // Farthest hand target of the reach (in front of the box and beside it) from its shoulder, as a share of the arm.
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
                    which = $"{a.Name} hand {(i == 0 ? "in front of the box" : "beside the box")}";
                }
            return worst;
        }

        // Grip planned from the live shoulders, then the reach checked from where the robot stands: every hand target (pre-grip in front of the
        // box, and beside it) within PlanReach of its shoulder, grip points at least MinSide off the body's centre line (Approach has lined the body up as close as the conveyor edge allows), else the attempt
        // fails here (retried; the box is skipped after MaxBoxAttempts). Logs how the body stands to the box.
        private bool Reachable()
        {
            PlanGrip();
            m_Slide = -k_PreFront; // targets = pre-grip points (ReachShare checks them and the grip points)
            foreach (Arm a in m_Arms)
                a.Gap = ApproachGap + k_PreWiden;
            float side = float.MaxValue,
                up = 0f;
            foreach (Arm a in m_Arms)
            {
                side = Math.Min(
                    side,
                    a.Side * Vector3.Dot(Flat(GripPoint(a) - Pelvis()), LeftOf(Heading()))
                );
                up = Math.Max(up, GripPoint(a).Y - Pelvis().Y);
            }
            if (side < MinSide)
            {
                Fail(
                    $"box too narrow or off-centre for the arms: a grip point comes within {side:F2} m of the body's centre line (needs {MinSide:F2})"
                );
                return false;
            }
            if (up > MaxGripUp)
                Log.Warn(
                    $"[G1] grip points {up:F2} m over the pelvis (arms are accurate up to {MaxGripUp:F2}) - reaching anyway"
                );
            float off = Mathf.WrapToPi(YawOf(m_Fwd) - YawOf(Heading())),
                face = Vector3.Dot(Flat(m_Center - Pelvis()), m_Fwd) - m_HalfD,
                reach = ReachShare(out string which);
            string state =
                $"facing {off * Mathf.Rad2Deg:F0} deg off the box, front face {face:F2} m ahead, farthest hand target {reach * 100f:F0}% of the arm ({which})";
            if (reach > PlanReach)
            {
                Fail(
                    $"box out of reach from where the robot stands: {state} (limit {PlanReach * 100f:F0}%) - move the robot or the box, or lower EdgeClearance so it can stand closer"
                ); // retried; skipped after MaxBoxAttempts
                return false;
            }
            Log.Info($"[G1] body vs box: {state} -> reaching");
            return true;
        }

        // Waits for StillTime of unbroken stillness, saying so every k_StillLog s; after PhaseTimeout of waiting the attempt fails (no reach on
        // a moving body). True while still waiting (or just failed).
        private bool AwaitStill(float waited, string where)
        {
            if (m_Still >= StillTime)
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

        // Grip height from the live shoulders: raised up the box until each grasp site is within GripComfort of the arm's reach, up to TopMargin
        // under its top; if even that isn't enough, says by how much the box sits too low. Logs the plan.
        private void PlanGrip()
        {
            m_GripLift = 0f;
            float reach = GripComfort / 100f * k_ArmLength,
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

        // How far the grip can still rise before a grasp site comes within TopMargin of the box top.
        private float MaxGripLift() =>
            2f * m_HalfH
            - TopMargin
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
            return side > -PlacedInside
                && side < ApproachGap + k_Widen
                && Math.Abs(front) < PlacedFront
                && up > PalmBelow(a)
                && up < 2f * m_HalfH - PlacedTop;
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
            if (gone > LostDistance || ChuteGap(m_Box) > 0.05f)
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

        // Ready: once the body has been still for StillTime straight and every hand target is in reach (Reachable), both hands glide together
        // in one straight move from the fold to the pre-grip points - grip height, k_PreFront in front of the box, k_PreWiden wider - (PreGrasp).
        private void StepReady()
        {
            if (!Track(true))
                return;
            SendHands(Fold, false); // folded, body-relative, while settling
            if (AwaitStill(m_StepTime, "before reaching") || !Reachable())
                return; // arm moves on a swaying body make the walker step
            Log.Info(
                $"[G1] reach: {GripInfo()} | front-bottom corners L {Corner(1f)} R {Corner(-1f)} | palms touch at width + 2 x PalmOffset {2f * (m_HalfW + PalmOffset):F3} m"
            );
            GlideTo(HandMoveSpeed);
            Advance(Phase.PreGrasp);
        }

        // The downhill hand: the box slides toward it and stops against its palm.
        private Arm Stopper => Vector3.Dot(m_Downhill, m_Lat) > 0f ? m_Arms[0] : m_Arms[1];

        // A hand the catch keeps folded like while walking (smoothed fold point, wrist on the heading), so it stays still: the free hand, and the stopper until it goes.
        private bool Parked(Arm a) => m_Phase == Phase.Catch && (a != Stopper || m_Step == 0);

        // Catch pose: the stopper's palm on the stop spot of the box's downhill face (m_Center = the box centre there) once it goes; parked hands folded.
        private Vector3 CatchPose(Arm a) => Parked(a) ? ParkPose(a) : m_StopCmd;

        // A hand parked through the catch: the fold point, pulled back so its fingertips (k_FingerTip ahead of the grasp site) stay
        // k_ParkClear behind the passing box's near face - a tall box slid into the parked right hand and was pushed further out.
        private Vector3 ParkPose(Arm a)
        {
            float near =
                Vector3.Dot(Flat(BoxCenter(m_Box) - Pelvis()), Heading())
                - HalfExtent(m_Box, Heading());
            Vector3 f = FoldOffset;
            f.X = Mathf.Clamp(near - k_FingerTip - k_ParkClear, k_ParkMin, f.X);
            return BodyPoint(Mirror(f, a.Side));
        }

        // The stopper's target: HandTarget with the palm centred on the box's mid-height (a hit above the centre of mass twists the box away),
        // its lower edge never under BeltClearance.
        private Vector3 StopTarget()
        {
            Arm s = Stopper;
            float up = Math.Max(m_HalfH + PalmBelow(s) - k_PalmHalf, BeltClearance + PalmBelow(s));
            return HandTarget(s) + Vector3.Up * (up - GripHeight(s));
        }

        // s the stopper takes to cover d to a still target: at CatchHandSpeed, plus its easing in.
        private float Travel(float d) => d / CatchHandSpeed + 2f * k_StopSmooth;

        // How far the box centre is past its stop spot along its path (- = still uphill of it).
        private float PastStop() =>
            Vector3.Dot(BoxCenter(m_Box) - m_Path, m_Downhill) - m_Ahead - m_Shift - m_Give;

        // Catch from the box's first frames: grip frame from the box and its path down the belt (through where it rests on the belt); the
        // robot catches from where it stands, so the belt must run across its front.
        private void EnterCatch()
        {
            m_Holder = null;
            SetGripFrame(Heading());
            m_Path = m_Center - m_ChuteN * ChuteGap(m_Box);
            m_Shift = m_Give = m_GiveV = m_PeakSpeed = m_PeakForce = m_PrevV = 0f;
            m_Accel = SlideAccel(); // frictionless until measured: the earliest the box can come
            m_Carrying = false;
            m_Lift = m_GripLift = 0f;
            m_Slide = -(FrontInset + k_CatchFront); // stopper waits in front of the path
            foreach (Arm a in m_Arms)
                a.Gap = a == Stopper ? 0f : ApproachGap;
            float across = Vector3.Dot(m_Downhill, LeftOf(Heading()));
            if (Math.Abs(across) < 0.5f)
            {
                Fail(
                    $"the belt doesn't run across the robot's front ({Mathf.Acos(Math.Abs(across)) * Mathf.Rad2Deg:F0} deg off) - turn the robot to face the conveyor",
                    false
                );
                return;
            }
            Log.Info(
                $"[G1] catching box {m_Box} at {F(m_Center)} ({BoxSpeed(m_Box):F2} m/s) with the {Stopper.Name} hand | {GripInfo()}"
            );
            Advance(Phase.Catch);
        }

        // Catch, every frame. Steps: 0 both hands folded while the body settles (the stop spot follows the pelvis: on the path with the palm
        // CatchPalmSide out from its centre line, so the arm's catch pose is the same for any box size), until it has been still for StillTime or the box is so close the stopper must go; 1 stopper to its waiting point in front of the
        // box's path (palm height, fingertips k_CatchFront clear of its front face); 2 slide in along the forward axis onto the stop spot; 3 in
        // place (contact tared); 4 give; 5 caught (then pushed up the belt, at most k_CentreMax, until its centre is on the body's centre line, whatever its size). From step 1 the spot is fixed in the world, except that once the stopper is at its waiting
        // point (and only there) it moves down the belt (never back up, at most CatchShiftMax) to k_CatchClear past where the box will be when the
        // palm gets there - predicted from its speed and its measured acceleration; the slide only starts with time to finish before the box
        // (+ k_SlideSpare), and from then the spot is fixed, so the palm only moves straight forward into the box's path, never along it.
        // Give: over the box's last k_GiveMatch before it reaches the palm, the spot speeds up so the gap closes at a steadily falling rate (they meet
        // with almost no speed between them); once the box is on the palm (contact, or within k_GiveTouch) it brakes to a stop over CatchGive, so the box is slowed by a
        // yielding palm instead of bouncing off a stiff one. The palm is centred on the box's mid-height (StopTarget). Caught once the give is over
        // and the box stays slower than CatchSpeed for CatchTime within k_CatchZone of the spot - or of the real palm, when the arm can't
        // quite reach its spot (from step 2 on: a box already resting on a late palm counts too); then it stays held there. Fails if it gets
        // past the stopper, falls off the belt, or never arrives.
        private void StepCatch(float ts)
        {
            Arm s = Stopper;
            Vector3 c = BoxCenter(m_Box);
            m_Mujoco!.GetBodyPoseMj(m_Boxes[m_Box].Body, out _, out _, out Vector3 vel, out _);
            float v = Vector3.Dot(m_MjToHz * vel, m_Downhill),
                speed = BoxSpeed(m_Box);
            ReadForces();
            LogContacts("CA");
            bool touch = m_Step >= 3 && s.Force + s.ThumbForce > TouchForce; // tared from step 3 on
            if (!touch && Math.Abs(ChuteGap(m_Box)) < 0.01f)
                m_Accel = Mathf.Clamp(
                    m_Accel
                        + ((v - m_PrevV) / ts - m_Accel)
                            * (1f - (float)Math.Exp(-ts / k_AccelSmooth)),
                    0f,
                    SlideAccel()
                ); // sliding freely on the belt: learn its acceleration
            m_PrevV = v;
            Vector3 goal = StopTarget(),
                spot = goal - m_Fwd * m_Slide; // the stopper's target now; its palm on the stop spot
            float err = (SiteWorld(s.Site) - goal).Length(),
                slide = Travel((spot - goal).Length()), // s the slide in takes
                hand =
                    (
                        m_Step <= 1
                            ? (err <= ReachTolerance ? 0f : Travel(err)) + slide
                            : Travel((SiteWorld(s.Site) - spot).Length())
                    ) + k_CatchLead; // s until the palm is on the stop spot
            if (m_Step == 0)
            {
                Vector3 left = LeftOf(Heading());
                m_Ahead =
                    (
                        Vector3.Dot(Pelvis() - m_Path, left)
                        + s.Side * (CatchPalmSide - m_HalfW - PalmOffset)
                    ) / Vector3.Dot(m_Downhill, left);
            }
            else if (m_Step == 1 && err <= ReachTolerance)
            {
                float then =
                    Vector3.Dot(c - m_Path, m_Downhill)
                    - m_Ahead
                    + v * hand
                    + 0.5f * m_Accel * hand * hand; // box centre past straight ahead when the palm gets there
                m_Shift = Math.Min(Math.Max(m_Shift, then + k_CatchClear), CatchShiftMax);
            }
            if (m_Step == 3)
            {
                float gap = Math.Max(0f, -PastStop()); // box centre short of the spot = its face short of the palm
                m_GiveV =
                    Math.Max(0f, v) * (1f - (float)Math.Sqrt(Math.Min(1f, gap / k_GiveMatch))); // relative speed falls at a steady rate: meets the box at its speed
                m_Give += m_GiveV * ts;
            }
            else if (m_Step == 4)
            {
                m_Give += m_GiveV * ts;
                m_GiveV = Math.Max(0f, m_GiveV - m_GiveDecel * ts);
            }
            m_Center = m_Path + m_Downhill * (m_Ahead + m_Shift + m_Give);
            float past = PastStop(),
                a = Math.Max(m_Accel, 1e-3f),
                eta = past >= 0f
                    ? 0f
                    : (-v + (float)Math.Sqrt(Math.Max(0f, v * v - 2f * a * past))) / a; // s until the box is at the spot
            if (m_Step == 0 && (m_Still >= StillTime || eta <= hand + k_CatchMargin))
            {
                Log.Info(
                    $"[G1] {s.Name} hand going: {(m_Still >= StillTime ? $"body still for {m_Still:F1} s" : $"box getting close (body still only {m_Still:F1} s)")}, box {-past:F3} m short of the stop spot, there in ~{eta:F2} s, the palm needs ~{hand:F2} s | stop spot fixed at {F(m_Center)}"
                );
                m_StopCmd = s.Cmd; // tracking starts from where the parked hand's command is
                s.T = s.Tau = 0f;
                s.Rising = false;
                NextStep();
            }
            else if (
                m_Step == 1
                && err <= ReachTolerance
                && eta > slide + k_CatchLead + k_SlideSpare
            ) // the slide in only starts with time to finish before the box comes
            {
                Log.Info(
                    $"[G1] {s.Name} hand in front of the box's path ({err:F3} m off) -> sliding in, box {-past:F3} m short of the stop spot, there in ~{eta:F2} s"
                );
                m_Slide = 0f;
                NextStep();
            }
            else if (m_Step == 2 && err <= ReachTolerance)
            {
                Tare();
                Log.Info(
                    $"[G1] {s.Name} palm in the box's path ({err:F3} m off the stop spot, {m_Shift:F3} m down the belt from straight ahead), box {-past:F3} m short, there in ~{eta:F2} s"
                );
                NextStep();
            }
            else if (m_Step == 3 && (touch || past >= -k_GiveTouch))
            {
                m_GiveDecel = m_GiveV * m_GiveV / (2f * Math.Max(CatchGive, 0.005f));
                Log.Info(
                    $"[G1] give: box on the {s.Name} palm ({(touch ? $"{s.Force + s.ThumbForce:F1} N" : $"{-past:F3} m short of it")}) at {v:F3} m/s, palm already moving {m_GiveV:F3} m/s after {m_Give:F3} m -> braking over {CatchGive:F3} m ({m_GiveDecel:F1} m/s^2)"
                );
                NextStep();
            }
            // The stopper's IK command tracks its target continuously - at up to CatchHandSpeed, easing in over k_StopSmooth near it, so a
            // moved target (sway, shift, slide in) is never a fresh fixed-length move; in the give it moves with the target directly.
            if (m_Step >= 1)
            {
                Vector3 d = StopTarget() - m_StopCmd;
                float len = d.Length(),
                    move = m_GiveV > 0f
                        ? len
                        : Math.Min(
                            len * (1f - (float)Math.Exp(-ts / k_StopSmooth)),
                            CatchHandSpeed * ts
                        );
                if (len > 1e-6f)
                    m_StopCmd += d * (move / len);
            }
            SendHands(CatchPose, false);
            m_PeakSpeed = Math.Max(m_PeakSpeed, speed);
            m_PeakForce = Math.Max(m_PeakForce, m_Step >= 3 ? s.Force + s.ThumbForce : 0f);
            if (PreGraspLogInterval > 0f && (m_DbgTimer += ts) >= PreGraspLogInterval)
            {
                m_DbgTimer = 0f;
                Log.Info(
                    $"[G1] CA t={m_PhaseTime:F2} step {m_Step} | box {F(c)} {v:F3} m/s down the belt (accel {m_Accel:F3} m/s^2), {-past:F3} m short of the stop spot, there in ~{eta:F2} s, belt gap {ChuteGap(m_Box):F3} m"
                        + $" | stop spot {F(m_Center)}: shift {m_Shift:F3} give {m_Give:F3} m ({m_GiveV:F3} m/s) | {s.Name} {err:F3} m off its target (palm on the spot in ~{hand:F2} s), reach {(SiteWorld(s.Site) - BodyPos(s.Shoulder)).Length() / k_ArmLength * 100f:F0}% / target {(goal - BodyPos(s.Shoulder)).Length() / k_ArmLength * 100f:F0}% of the arm{(s.Capped ? " (capped)" : "")}"
                        + $" | force palm {s.PalmForce:F1} hand {s.Force:F1} thumb {s.ThumbForce:F1} N | pelvis {F(Pelvis())} yaw {YawOf(Heading()) * Mathf.Rad2Deg:F0} deg"
                );
            }
            if (m_Step == 5)
            {
                float off =
                        s.Side * Vector3.Dot(Flat(BoxCenter(m_Box) - Pelvis()), LeftOf(Heading())), // box centre off the body's centre line, on the stopper's side
                    need = Math.Min(off, k_CentreMax + m_Give - m_Caught);
                if (need > k_Centred)
                {
                    if (m_Give == m_Caught)
                        Log.Info(
                            $"[G1] box centre {off:F3} m off the body's centre line -> {s.Name} hand pushes it up the belt to the centre"
                        );
                    m_Give -= Math.Min(k_CentreSpeed * ts, need);
                    m_StepTime = 0f;
                    return;
                }
                if (m_StepTime <= 1.5f * ts && m_Give != m_Caught)
                    Log.Info(
                        $"[G1] box centred after a {m_Caught - m_Give:F3} m push: centre {off:F3} m off the body's centre line"
                    );
                if (!AwaitStill(m_StepTime, "before grasping the caught box"))
                    EnterGrasp();
                return; // caught: pushed to the centre, held there until the body is still, then grasp
            }
            if (past > m_HalfW || ChuteGap(m_Box) < -0.05f || m_PhaseTime > PhaseTimeout)
            {
                Fail(
                    past > m_HalfW
                        ? $"box got past the {s.Name} hand ({past:F3} m beyond the stop spot, step {m_Step}{(m_Step <= 1 ? " - too late to slide the palm in" : "")}, hand {err:F3} m off its target)"
                    : ChuteGap(m_Box) < -0.05f ? "box fell off the belt"
                    : $"box never stopped at the {s.Name} hand: {-past:F3} m short of the stop spot, {Vector3.Dot(SiteWorld(s.Site) - c, m_Downhill) - PalmOffset - m_HalfW:F3} m off the real palm, {speed:F3} m/s after {m_PhaseTime:F0} s"
                );
                return;
            }
            float palmGap = Vector3.Dot(SiteWorld(s.Site) - c, m_Downhill) - PalmOffset - m_HalfW; // box's downhill face short of the real palm (the arm may not reach its spot)
            bool atHand = past >= -k_CatchZone || palmGap <= k_CatchZone;
            if (m_Step < 2 || m_GiveV > 0f || speed > CatchSpeed || !atHand)
                m_StepTime = 0f; // from here m_StepTime = time stopped at the hand (after the give, or resting on a palm that got there late)
            if (m_StepTime < CatchTime)
                return;
            Log.Info(
                $"[G1] caught box {m_Box} after {m_PhaseTime:F2} s{(m_Step < 4 ? $" (at rest on the palm in step {m_Step}, before the give)" : "")}: {-past:F3} m short of the stop spot ({palmGap:F3} m off the real palm), came at up to {m_PeakSpeed:F2} m/s, gave {m_Give:F3} m, hand force up to {m_PeakForce:F1} N, now {s.Force:F1} N | holding it there for the grasp"
            );
            m_Caught = m_Give;
            NextStep();
        }

        // Caught -> grasp. The stopper becomes the holder: its target is pinned where its IK command is now (HoldHere), so it doesn't move -
        // later only its Gap (the squeeze), the lift and the carry move it. The other hand comes in as in any grasp: the grip frame from the
        // box as it rests on the holder, its pre-grip point in front of the box, its palm level with the holder's - or, on a box too narrow
        // for the straight thumbs to pass (under 2 x k_ThumbReach), 2 x k_ThumbStagger over it - and always all on the face, BeltClearance
        // over the box bottom and under its top; checked within PlanReach (a grip near the body's centre line only warns); then it glides
        // there from the fold and PreGrasp slides it in.
        private void EnterGrasp()
        {
            Arm h = Stopper,
                r = h == m_Arms[0] ? m_Arms[1] : m_Arms[0];
            m_Holder = h;
            SetGripFrame(m_Fwd);
            m_Carrying = false;
            m_Lift = 0f;
            m_Boost = 1f;
            m_PickAt = m_BoxAt = BoxCenter(m_Box);
            m_Reach = StandDistance;
            m_SqueezeTries = m_WalkRetries = 0;
            SetSafeY();
            h.Gap = 0f;
            r.Gap = ApproachGap + k_PreWiden;
            m_Slide = -k_PreFront;
            float bottom = m_Center.Y - m_HalfH,
                lowest = BeltClearance + k_PalmHalf, // palm centre range over the box bottom that keeps the palm all on the face
                palm = Mathf.Clamp(
                    h.Cmd.Y
                        - PalmBelow(h)
                        + k_PalmHalf
                        - bottom
                        + (m_HalfW < k_ThumbReach ? 2f * k_ThumbStagger : 0f),
                    lowest,
                    Math.Max(lowest, 2f * m_HalfH - k_PalmHalf)
                ), // reaching palm's centre over the box bottom
                low = BeltClearance + PalmBelow(r) + (1f + r.Side) * k_ThumbStagger; // its grip height with no raise
            m_GripLift = Math.Max(0f, palm + PalmBelow(r) - k_PalmHalf - low);
            HoldHere();
            Vector3 sh = BodyPos(r.Shoulder);
            float reach =
                    Math.Max((HandTarget(r) - sh).Length(), (GripPoint(r) - sh).Length())
                    / k_ArmLength,
                side = r.Side * Vector3.Dot(Flat(GripPoint(r) - Pelvis()), LeftOf(Heading()));
            if (reach > PlanReach)
            {
                Fail(
                    $"caught box out of the {r.Name} hand's reach: {reach * 100f:F0}% of the arm (limit {PlanReach * 100f:F0}%)"
                );
                return;
            }
            if (side < MinSide)
                Log.Warn(
                    $"[G1] {r.Name} grip point {side:F2} m off the body's centre line (MinSide {MinSide:F2}) - the box stopped off-centre, reaching anyway"
                );
            Log.Info(
                $"[G1] grasping the caught box: {h.Name} hand holds still, {r.Name} hand reaches in ({reach * 100f:F0}% of the arm, grip {GripHeight(r):F3} m over the box bottom, raised {m_GripLift:F3} m) | {GripInfo()}"
            );
            GlideTo(HandMoveSpeed);
            Advance(Phase.PreGrasp);
        }

        // Pins the holding hand where its IK command is now: from here its target is that point in the grip frame, moved only by its Gap,
        // the lift and the carry (HandTarget adds this offset).
        private void HoldHere()
        {
            Arm h = m_Holder!;
            m_HoldOff = new Vector3(0f, 0f, 0f);
            Vector3 d = h.Cmd - HandTarget(h);
            m_HoldOff = new Vector3(Vector3.Dot(d, m_Lat), Vector3.Dot(d, m_Fwd), d.Y);
        }

        // Both hands start one straight min-jerk move from their current IK targets to HandTarget, both taking the time the longer one needs at
        // speed (they arrive together); SendHands then runs it - no path planning, so nothing climbs over the box.
        private void GlideTo(float speed)
        {
            float t = k_MinSegment;
            foreach (Arm a in m_Arms)
                t = Math.Max(t, (HandTarget(a) - a.Cmd).Length() / speed);
            foreach (Arm a in m_Arms)
            {
                a.From = a.Cmd;
                a.To = HandTarget(a);
                a.T = t;
                a.Tau = 0f;
                a.Rising = false;
            }
        }

        // At the pre-grip points (in front of the box, once the body is still) contact sensing is tared and both hands slide in together along the
        // side faces to the grip points, ApproachGap clear of them (a hand that lands on the box slides outward). Grasp (Squeeze) once both are
        // within ReachTolerance - or beside the box faces (Placed) and no longer improving. Hands that stall short adapt (GraspOutOfReach).
        private void StepPreGrasp(float ts)
        {
            if (!Track(m_Holder == null)) // a held box stays put: following it would let the holding palm chase it
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
                    if (a != m_Holder && a.Force > TouchForce)
                    {
                        if (a.Gap <= ApproachGap)
                            Log.Warn(
                                $"[G1] PG {a.Name} landed on the box ({a.Force:F1} N) -> sliding out, gap widening up to {ApproachGap + k_Widen:F3} m"
                            );
                        a.Gap = Math.Min(a.Gap + k_OpenSpeed * ts, ApproachGap + k_Widen);
                    }
            float err = Math.Max(HandError(m_Arms[0]), HandError(m_Arms[1]));
            bool stalled = err > ReachTolerance && Stalled(err, ReachProgress, PhaseTimeout);
            if (
                m_Step == 1
                && (err <= ReachTolerance || (BothPlaced() && m_SinceBest > PlacedWait))
            )
            {
                if (err > ReachTolerance)
                    Log.Info(
                        $"[G1] hands {err:F3} m from their targets but beside the box faces - grasping"
                    );
                Log.Info(
                    $"[G1] PG end of the slide in after {m_StepTime:F2} s: err L {HandError(m_Arms[0]):F3} R {HandError(m_Arms[1]):F3} m, placed L {Placed(m_Arms[0], out _, out _, out _)} R {Placed(m_Arms[1], out _, out _, out _)}"
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
            if (AwaitStill(m_WatchTime, "in front of the box"))
                return; // hold in front of the box until the body stops swaying
            Log.Info(
                $"[G1] PG in front of the box after {m_StepTime:F2} s (err L {HandError(m_Arms[0]):F3} R {HandError(m_Arms[1]):F3} m, body still {m_Still:F1} s, {BodyMotion():F3} m/s at the hands) -> tare, slide in along the side faces"
            );
            Tare();
            m_Slide = 0f;
            foreach (Arm a in m_Arms)
                a.Gap = a == m_Holder ? 0f : ApproachGap;
            GlideTo(k_SlideSpeed);
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
                $"[G1] PG t={m_PhaseTime:F2} step {m_Step} ({(m_Step == 0 ? "to the pre-grip points" : "sliding in")}) | sim {speed:F2}x real time ({wall * 1000f:F0} ms wall)"
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
                        + $" | box frame: palm {side:F3} out, thumb {front:F3} fwd, site {up:F3} up{(placed ? " (placed)" : "")}, gap {a.Gap:F3} m, slide {m_Slide:F3} m"
                        + $" | force palm {a.Force:F1} thumb {a.ThumbForce:F1} N, touching [{string.Join(", ", touching)}] | palm {a.PalmErr:F0} deg off"
                        + $" | joints deg [{string.Join(", ", Array.ConvertAll(q, v => (v * Mathf.Rad2Deg).ToString("F0")))}], max change {jump * Mathf.Rad2Deg:F1} deg since the last line"
                        + $" | {WristLog(a)}"
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
            float dt = Math.Max(m_DbgSim, 1e-3f); // sim s since the last line (joint rates, site velocity)
            m_DbgTimer = m_DbgSim = 0f;
            m_Mujoco!.GetBodyPoseMj(m_PelvisId, out _, out _, out Vector3 pv, out Vector3 pw);
            m_Mujoco.GetBodyPoseMj(m_TorsoId, out _, out _, out _, out Vector3 tw);
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
                    + $" | turn rate (roll, yaw, pitch) pelvis {F(ToBody(m_MjToHz * pw))} torso {F(ToBody(m_MjToHz * tw))} rad/s, waist (yaw, roll, pitch) [{Degs(m_WaistAdr)}] deg"
                    + $" | box {F(c)} v {bv.Length():F3} m/s, ang vel {F(ToBody(m_MjToHz * bw))} rad/s, slip {slip}, hand gap {(SiteWorld(m_Arms[0].Site) - SiteWorld(m_Arms[1].Site)).Length():F3} m (palms touch at {2f * (m_HalfW + PalmOffset):F3}){(speed < 0.5f ? " | SIM SLOW" : "")}"
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
                    $"[G1] GR   {a.Name}: palm {a.PalmForce:F1} N{(a.PalmOn ? "" : " (off its face)")}, fingers {a.Force - a.PalmForce:F1} N, thumb {a.ThumbForce:F1} N"
                        + $" | gap target {a.Gap:F3} m, palm measured {face:F3} m off its face, palm height on the face {on * 100f:F0}% (lower edge {lo - (c.Y - m_HalfH):F3} m over the box bottom)"
                        + $" | curl {a.Curl:F2} rad, thumb {a.Thumb:F2} rad, palm {a.PalmErr:F0} deg off | err {HandError(a):F3} m | touching [{string.Join(", ", touching)}]"
                        + $" | {WristLog(a)}"
                        + $" | {ArmLog(a, dt)}"
                );
            }
        }

        // Grasp: the thumbs swing onto the front face and index + middle curl into the side faces while the arms close in, each hand pushing
        // toward the other, until both palms have reached their faces and each hand holds the required force for GripConfirm s (the hands follow the box until the first touch). A grip that
        // stops building with a hand at its squeeze limit (palm gap and finger curl both maxed) is reopened and re-centred (MaxSqueezeTries).
        private void StepSqueeze(float ts)
        {
            if (!Track(m_Holder == null && Contact() < TouchForce))
                return;
            SendHands(HandTarget, false);
            float req = RequiredForce(),
                need = req * GripForceLimit / 100f;
            Squeeze(ts, req);
            LogGrip(ts);
            bool stuck = false;
            foreach (Arm a in m_Arms)
            {
                bool held = a.PalmOn && a.Force >= need; // palm on its face; palm + fingers hold GripForceLimit of the required force
                if (!held)
                    m_StepTime = 0f;
                stuck |= !held && a.Gap <= 0.001f - MaxSqueezeDepth;
            }
            if (!stuck)
                m_SinceBest = 0f; // still closing or holding: that's progress
            if (m_StepTime >= k_GripConfirm)
            {
                Log.Info(
                    $"[G1] grip: palm L={m_Arms[0].PalmForce:F1} R={m_Arms[1].PalmForce:F1} N (need {need:F1} of {req:F1} N), fingers L={m_Arms[0].Force - m_Arms[0].PalmForce:F1} R={m_Arms[1].Force - m_Arms[1].PalmForce:F1} N, thumb L={m_Arms[0].ThumbForce:F1} R={m_Arms[1].ThumbForce:F1} N, curl L={m_Arms[0].Curl:F2} R={m_Arms[1].Curl:F2} rad, gap L={m_Arms[0].Gap:F3} R={m_Arms[1].Gap:F3} m"
                );
                m_Y0 = PalmMidY() - BoxCenter(m_Box).Y;
                m_Slot = m_Center - Vector3.Up * m_HalfH; // pick spot (bottom centre): where the box goes back when not carried to the table
                bool far = GripAhead() > k_DragFrom;
                if (far && LiftBox)
                    Log.Info(
                        $"[G1] gripped {GripAhead():F2} m ahead -> dragging the box in to {k_DragTo:F2} m (elbows bent for the lift)"
                    );
                Advance(
                    LiftBox ? (far ? Phase.DragIn : Phase.Lift)
                    : PlaceBox ? Phase.Release
                    : Phase.Holding
                );
            }
            else if (m_SinceBest > StallTime || m_WatchTime > PhaseTimeout)
                RetrySqueeze(need);
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
                a.Gap = a == m_Holder ? 0f : ApproachGap; // a holding hand stays on its face
                a.PalmOn = false;
            }
            if (m_Holder != null)
                HoldHere(); // re-pinned to the re-read frame where it is now
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
                Log.Warn($"[G1] {why} -> gripping {m_GripLift:F3} m higher up the box"); // the hand goals jump up by this much
                return;
            }
            Fail(
                $"{why}: no higher grip left (grip raised {m_GripLift:F3} m) - raise the belt or move the robot"
            );
        }

        // How far ahead of the pelvis the grasp sites' grip spot is (m).
        private float GripAhead() =>
            Vector3.Dot(Flat(m_Center - Pelvis()), Heading()) - m_HalfD + FrontInset;

        // Drag in: still squeezing, the grip slides toward the body at k_DragSpeed (the box sliding on the belt) until the grip spot is
        // k_DragTo ahead, then Lift. A box trailing the hands by more than k_DragLag is lifted from where it is.
        private void StepDragIn(float ts)
        {
            if (!Hold(ts))
                return;
            float lag = Vector3.Dot(Flat(BoxCenter(m_Box) - m_Center), m_Fwd);
            if (lag > k_DragLag)
            {
                Log.Warn(
                    $"[G1] box trails the hands by {lag:F3} m while dragging -> lifting from here"
                );
                m_Center = BoxCenter(m_Box);
            }
            else if (GripAhead() > k_DragTo)
            {
                m_Center -= m_Fwd * (k_DragSpeed * ts);
                return;
            }
            Log.Info($"[G1] dragged in: grip {GripAhead():F2} m ahead | {ArmAngles()}");
            m_Slot = m_Center - Vector3.Up * m_HalfH;
            Advance(Phase.Lift);
        }

        // Lift in pelvis coords from the start, so the box follows the body (a body pushed back by the box's weight never stretches the
        // arms after it): up k_ClearLift, then up the rest of LiftHeight and in toward CarryOffset together, so a heavy box comes close to
        // the chest early. No station hold while picking up: its correction steps rocked the body to and fro and flung a heavy box. Next
        // carry it, or put it back down where it was picked.
        private void StepLift(float ts)
        {
            if (m_Step == 0)
            {
                SetAnchor();
                m_Center = ToBody(m_Center - Pelvis());
                m_Lat = ToBody(m_Lat);
                m_Fwd = ToBody(m_Fwd);
                m_Carrying = true;
                m_LiftTo = new Vector3(
                    Math.Min(m_Center.X, CarryX()),
                    m_Center.Y + LiftHeight,
                    m_Center.Z
                );
                NextStep();
            }
            if (!Hold(ts))
                return;
            float up = Math.Min(m_Center.Y + k_LiftSpeed * ts, m_LiftTo.Y),
                fwd = up >= m_LiftTo.Y - LiftHeight + k_ClearLift
                    ? Math.Max(m_Center.X - k_CarrySpeed * ts, m_LiftTo.X)
                    : m_Center.X;
            m_Center = new Vector3(fwd, up, m_Center.Z);
            if (up < m_LiftTo.Y || fwd > m_LiftTo.X)
                return;
            Advance(
                CarryToTable ? Phase.PullIn
                : PlaceBox ? Phase.Place
                : Phase.Holding
            );
        }

        // How far ahead of the pelvis the box is carried: CarryOffset, or - if its top can reach head height anywhere in the carry
        // (raised over the table included) - far enough that its near face stays HeadClearance in front of the head.
        private float CarryX() =>
            CarryOffset.Y + k_MaxRaise + m_HalfH > k_HeadBottom
                ? Math.Max(CarryOffset.X, k_HeadFront + HeadClearance + m_HalfD)
                : CarryOffset.X;

        // The pelvis spot to hold from now on (standing with the box).
        private void SetAnchor()
        {
            m_Anchor = Pelvis();
            m_Restoring = false;
        }

        // Standing with the box: once its weight has walked the body more than k_DriftTol off m_Anchor, and the body has been still for
        // StillTime, the walker steps it back at the slowest command it steps at, along one direction only - forward / back first, sideways
        // once that is within k_DriftDone - until that direction is within k_DriftDone (an overshoot ends it; the next waits for stillness).
        // Never during a turn: a held box made combined commands stumble.
        private void HoldStation()
        {
            Vector3 d = Flat(m_Anchor - Pelvis());
            float fwd = Vector3.Dot(d, Heading()),
                side = Vector3.Dot(d, LeftOf(Heading()));
            bool turning = m_Cmd.Z != 0f;
            if (!m_Restoring && !turning && d.Length() > k_DriftTol && m_Still >= StillTime)
            {
                m_RestoreDir =
                    Math.Abs(fwd) > k_DriftDone
                        ? new Vector2(Math.Sign(fwd), 0f)
                        : new Vector2(0f, Math.Sign(side)); // one axis, one way
                m_Restoring = true;
                Log.Info(
                    $"[G1] the box walked the body {d.Length():F3} m off its spot in {m_Phase} ({-fwd:F3} fwd, {-side:F3} left) -> stepping {(m_RestoreDir.X > 0f ? "forward" : m_RestoreDir.X < 0f ? "back" : m_RestoreDir.Y > 0f ? "left" : "right")} onto it"
                );
            }
            if (!m_Restoring)
                return;
            float left = m_RestoreDir.X != 0f ? m_RestoreDir.X * fwd : m_RestoreDir.Y * side; // still to go along the chosen direction
            if (turning || left < k_DriftDone)
            {
                m_Restoring = false;
                Log.Info(
                    $"[G1] {(turning ? "correction stopped for the turn" : "correction done")} in {m_Phase} ({d.Length():F3} m off its spot)"
                );
                return;
            }
            Drive(
                m_RestoreDir.X > 0f ? k_FwdCmd
                    : m_RestoreDir.X < 0f ? -k_BackCmd
                    : 0f,
                m_RestoreDir.Y * k_SideCmd,
                0f
            );
        }

        // Picks the nearest free table slot and where to stand for it, then backs off the conveyor, turns to face the table, walks onto that spot and squares up.
        private void EnterTurnToTable()
        {
            m_Slot = TableSpot(m_Anchor - Heading() * CarryBackDistance); // nearest to where it turns, after backing off
            m_Reach = PlaceStandDistance;
            m_WalkRetries = 0;
            m_Stand = StandFor(PlaceTarget!, m_Slot, m_Reach, FaceOf(PlaceTarget!));
            Log.Info(
                $"[G1] placing at {m_Slot}, standing at {m_Stand}, {Vector3.Dot(m_Slot - m_Stand, FaceOf(PlaceTarget!)):F2} m from the slot"
            );
            Advance(Phase.BackOff);
        }

        // Settle and switch the grip to world coords; a box not yet over the table comes in over the table spot first; then it goes
        // straight down where it is until the table holds it (or the hands can't go lower) and is let go there.
        private void StepPlace(float ts)
        {
            if (m_PhaseTime <= 1.5f * ts)
                SetAnchor();
            if (!Hold(ts))
                return;
            HoldStation();
            Vector3 c = BoxCenter(m_Box);
            if (m_Step == 0)
            {
                if (m_StepTime < k_Settle)
                    return;
                m_Center = CarryPoint(m_Center);
                m_Lat = CarryDir(m_Lat);
                m_Fwd = CarryDir(m_Fwd);
                m_Carrying = false;
                NextStep();
            }
            else if (m_Step == 1)
            {
                if (
                    OverTable(c, k_OverTableInset)
                    || MoveTowards(
                        ref m_Center,
                        new Vector3(m_Slot.X, m_Center.Y, m_Slot.Z),
                        k_CarrySpeed * ts
                    )
                )
                    NextStep();
            }
            else
            {
                m_Center = m_Center - Vector3.Up * (k_PlaceLowerSpeed * ts);
                float above = c.Y - m_HalfH - TopOf(PlaceTarget!).Y; // box bottom over the table top
                if (above > 0.01f && m_Center.Y > c.Y - 0.02f)
                    return; // still coming down
                Log.Info($"[G1] releasing {above:F3} m over the table");
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
                Advance(Repeat ? Phase.FindBox : Phase.Done);
            }
        }

        // Arm test trial target pair, in pelvis coords (forward, up, left) when it starts. Random: centre 0.25-0.40 m ahead, 0-0.30 m over the
        // pelvis, up to 0.08 m off centre; hands 0.10-0.22 m either side of it (boxes ~0.16-0.40 m wide), resampled until both are within
        // PlanReach. Sweep (ArmTestSweep): the next s_ArmSweep entry, whatever its reach.
        private bool PickArmTrial()
        {
            float R(float lo, float hi) => lo + (float)m_Rng.NextDouble() * (hi - lo);
            for (int k = 0; k < 50; k++)
            {
                (string Sweep, float F, float Up, float C, float Apart, float PullTo) sw =
                    ArmTestSweep
                        ? s_ArmSweep[m_ArmIdx]
                        : (
                            "",
                            R(0.25f, 0.40f),
                            R(0f, 0.30f),
                            R(-0.08f, 0.08f),
                            2f * R(0.10f, 0.22f),
                            0f
                        );
                float f = sw.F,
                    up = sw.Up,
                    c = sw.C,
                    w = 0.5f * sw.Apart;
                Vector3 l = BodyPoint(new Vector3(f, up, c + w)),
                    r = BodyPoint(new Vector3(f, up, c - w));
                float rl = (l - BodyPos(m_Arms[0].Shoulder)).Length() / k_ArmLength,
                    rr = (r - BodyPos(m_Arms[1].Shoulder)).Length() / k_ArmLength;
                if (!ArmTestSweep && Math.Max(rl, rr) > PlanReach)
                    continue;
                float v =
                    sw.Sweep == "height" ? up
                    : sw.Sweep == "ahead" ? f
                    : sw.Sweep == "apart" ? sw.Apart
                    : sw.Sweep == "offset" ? c
                    : f;
                m_ArmKey = ArmTestSweep ? $"{sw.Sweep} {v:+0.00;-0.00} m" : "";
                m_ArmGoal[0] = l;
                m_ArmGoal[1] = r;
                m_ArmPull = new Vector4(sw.PullTo, up, c, w);
                m_ArmPlan =
                    $"centre {f:F2} m ahead, {up:F2} m over the pelvis, {c:+0.00;-0.00} m left | hands {2f * w:F2} m apart | reach L {rl * 100f:F0}% R {rr * 100f:F0}%";
                return true;
            }
            return false;
        }

        private Vector3 ArmGoal(Arm a) => m_ArmGoal[a.Side > 0f ? 0 : 1];

        // Arm test, one trial per pass: still body, new target pair, both hands move there together (same paths, IK re-seed, wrist servo
        // and cap as a reach), hold k_ArmHold s measuring each hand's error (mean, max, jitter), the fastest arm joint and the body's motion,
        // log the trial, fold again; after the last trial a summary of the worst ones.
        private void StepArmTest()
        {
            switch (m_Step)
            {
                case 0:
                    SendHands(Fold, false);
                    if (m_StepTime < k_ArmSettle || (m_Still < StillTime && m_StepTime < 10f))
                        return;
                    if (!PickArmTrial())
                    {
                        Log.Warn(
                            "[G1] arm test: no reachable target pair found from here - move the robot"
                        );
                        Advance(Phase.Done);
                        return;
                    }
                    Log.Info($"[G1] AT {m_ArmIdx + 1}/{ArmTrials} go: {m_ArmPlan}");
                    SendHands(ArmGoal, true);
                    NextStep();
                    return;
                case 1:
                    SendHands(ArmGoal, false);
                    if (Gliding() && m_StepTime < k_ArmMoveMax)
                        return;
                    m_ArmReached = m_StepTime;
                    ResetArmStats();
                    NextStep();
                    return;
                case 2:
                    SendHands(ArmGoal, false);
                    ArmStats(ArmGoal);
                    if (m_StepTime < k_ArmHold)
                        return;
                    float Mean(int s) => m_ErrSum[s] / m_ErrN;
                    float Jit(int s) =>
                        (float)Math.Sqrt(Math.Max(0f, m_ErrSq[s] / m_ErrN - Mean(s) * Mean(s)));
                    string line =
                        $"{m_ArmPlan} | {(m_ArmReached >= k_ArmMoveMax ? "move timed out" : $"arrived in {m_ArmReached:F1} s")}"
                        + $" | held {k_ArmHold:F0} s: error L mean {Mean(0):F3} max {m_ErrMax[0]:F3} jitter {Jit(0):F3}, R mean {Mean(1):F3} max {m_ErrMax[1]:F3} jitter {Jit(1):F3} m"
                        + $" | fastest arm joint {m_JointMax:F0} deg/s | body {m_BodyMax:F2} m/s at the hands{(m_ArmCapped ? " | IK target capped" : "")} | {ArmAngles()}";
                    Log.Info($"[G1] AT {m_ArmIdx + 1}/{ArmTrials} result: {line}");
                    m_ArmResults.Add(
                        (
                            Math.Max(Mean(0), Mean(1)),
                            Math.Max(Jit(0), Jit(1)),
                            m_JointMax,
                            line,
                            $"{m_ArmKey}: error L {Mean(0):F3} R {Mean(1):F3} m (max {Math.Max(m_ErrMax[0], m_ErrMax[1]):F3}), jitter {Math.Max(Jit(0), Jit(1)):F3} m, fastest joint {m_JointMax:F0} deg/s, body {m_BodyMax:F2} m/s{(m_ArmCapped ? ", capped" : "")} | {ArmAngles()}"
                        )
                    );
                    if (m_ArmPull.X > 0f)
                    {
                        // Slide: both hands straight back to PullTo ahead at k_PullSpeed (one fixed path), tracking measured against it.
                        m_ArmGoal[0] = BodyPoint(
                            new Vector3(m_ArmPull.X, m_ArmPull.Y, m_ArmPull.Z + m_ArmPull.W)
                        );
                        m_ArmGoal[1] = BodyPoint(
                            new Vector3(m_ArmPull.X, m_ArmPull.Y, m_ArmPull.Z - m_ArmPull.W)
                        );
                        foreach (Arm a in m_Arms)
                        {
                            a.From = a.Cmd;
                            a.To = ArmGoal(a);
                            a.T = Math.Max(k_MinSegment, (a.To - a.From).Length() / k_PullSpeed);
                            a.Tau = 0f;
                            a.Rising = false;
                        }
                        ResetArmStats();
                        Log.Info(
                            $"[G1] AT {m_ArmIdx + 1}/{ArmTrials} sliding the hands back to {m_ArmPull.X:F2} m ahead at {k_PullSpeed:F2} m/s"
                        );
                        NextStep();
                        return;
                    }
                    SendHands(Fold, true);
                    m_Step = 3; // no slide: straight to folding
                    NextStep();
                    return;
                case 3:
                    SendHands(ArmGoal, false);
                    ArmStats(a => a.Cmd); // tracking error along the slide path
                    if (Gliding() || m_StepTime < m_Arms[0].T + 1f)
                        return;
                    float pm = Math.Max(m_ErrSum[0], m_ErrSum[1]) / m_ErrN;
                    string pull =
                        $"slide back to {m_ArmPull.X:F2} m ahead: tracking error L mean {m_ErrSum[0] / m_ErrN:F3} max {m_ErrMax[0]:F3}, R mean {m_ErrSum[1] / m_ErrN:F3} max {m_ErrMax[1]:F3} m"
                        + $" | end error L {(SiteWorld(m_Arms[0].Site) - m_ArmGoal[0]).Length():F3} R {(SiteWorld(m_Arms[1].Site) - m_ArmGoal[1]).Length():F3} m"
                        + $" | fastest arm joint {m_JointMax:F0} deg/s | body {m_BodyMax:F2} m/s{(m_ArmCapped ? " | capped" : "")} | {ArmAngles()}";
                    Log.Info($"[G1] AT {m_ArmIdx + 1}/{ArmTrials} {pull}");
                    m_ArmResults.Add((pm, 0f, m_JointMax, pull, $"{m_ArmKey} {pull}"));
                    SendHands(Fold, true);
                    NextStep();
                    return;
                default:
                    SendHands(Fold, false);
                    if (Gliding() && m_StepTime < k_ArmMoveMax)
                        return;
                    if (++m_ArmIdx < ArmTrials)
                    {
                        Advance(Phase.ArmTest);
                        return;
                    }
                    int missed = 0,
                        shaky = 0;
                    foreach (var r in m_ArmResults)
                    {
                        missed += r.Err > 0.03f ? 1 : 0;
                        shaky += r.Jit > 0.005f || r.Speed > 120f ? 1 : 0;
                    }
                    Log.Info(
                        $"[G1] arm test done: {m_ArmResults.Count} trials, {missed} missed (mean error > 3 cm), {shaky} shaky (jitter > 5 mm or a joint > 120 deg/s)"
                    );
                    if (ArmTestSweep)
                        foreach (var r in m_ArmResults)
                            Log.Info($"[G1] AT sweep {r.Key}");
                    m_ArmResults.Sort((x, y) => y.Err.CompareTo(x.Err));
                    for (int i = 0; i < Math.Min(5, m_ArmResults.Count); i++)
                        Log.Info($"[G1] AT worst error #{i + 1}: {m_ArmResults[i].Line}");
                    m_ArmResults.Sort((x, y) => y.Jit.CompareTo(x.Jit));
                    for (int i = 0; i < Math.Min(3, m_ArmResults.Count); i++)
                        Log.Info($"[G1] AT most jitter #{i + 1}: {m_ArmResults[i].Line}");
                    Advance(Phase.Done);
                    return;
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
                        || (m_Still < StillTime && m_StepTime < k_TestWait)
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
                Advance(m_Retry && Repeat ? Phase.FindBox : Phase.Failed);
        }

        // While holding: keep squeezing at HeldPalmForce (or what the weight needs), track the hand targets, boost the force if the box slips, fail if it slides out.
        private bool Hold(float ts)
        {
            Squeeze(
                ts,
                Math.Max(RequiredForce(), Math.Min(HeldPalmForce, MaxPalmForce) / k_SqueezeMargin),
                true
            );
            SendHands(HandTarget, false);
            LogGrip(ts);
            float slip = PalmMidY() - BoxCenter(m_Box).Y - m_Y0; // + = box sliding down between the palms
            if (slip > DropTolerance)
            {
                m_Carrying = false; // it's out of the hands: nothing left to hold on to
                Fail($"box slipped out of the hands ({slip:F3} m)");
                return false;
            }
            if (slip > SlipTolerance)
                m_Boost = Math.Min(m_Boost + ts, 2f);
            return true;
        }

        // Squeeze per hand, on the palm's own force: the gap closes at full rate until the palm touches, then presses toward k_SqueezeMargin x
        // target (admittance). The palm is on its face from TouchForce until it stays under half of it for k_PalmHold. Index + middle curl in
        // while the palm is on (easing back while it is off) up to k_MaxCurl, wrapping the box's front corners, but never press past
        // MaxFingerForce (they ease back over it, curl on again under 0.8 x it); once held they stay wrapped, a palm that comes off closes
        // straight back in past them, and the hand never spreads unless the palm presses past MaxPalmForce.
        private void Squeeze(float ts, float target, bool held = false)
        {
            ReadForces();
            foreach (Arm a in m_Arms)
            {
                bool free = !a.PalmOn && (held || a.Force + a.ThumbForce < TouchForce); // palm not on its face yet (or off it while held): close at full rate
                float close = free
                    ? k_MaxSqueezeRate
                    : Mathf.Clamp(
                        k_DepthGain * (k_SqueezeMargin * target - a.PalmForce),
                        -k_MaxSqueezeRate,
                        k_MaxSqueezeRate
                    );
                if (held && a.PalmForce < MaxPalmForce)
                    close = Math.Max(close, 0f);
                a.Gap = Mathf.Clamp(
                    a.Gap - close * ts,
                    -MaxSqueezeDepth,
                    held ? 0f : ApproachGap + k_Widen
                );
                bool touch = a.PalmForce >= (a.PalmOn ? 0.5f * TouchForce : TouchForce);
                a.PalmLow = touch ? 0f : a.PalmLow + ts;
                bool on = touch || (a.PalmOn && a.PalmLow < k_PalmHold);
                if (on != a.PalmOn)
                    Log.Info(
                        $"[G1] GR {a.Name} palm {(on ? "on" : "off")} its face in {m_Phase} ({a.PalmForce:F1} N, gap {a.Gap:F3} m, fingers {a.Force - a.PalmForce:F1} N, curl {a.Curl:F2} rad) -> {(on ? "fingers curl in" : held ? "palm closes in" : "fingers ease back")}"
                    );
                a.PalmOn = on;
                float fingers = a.Force - a.PalmForce;
                a.Curl =
                    fingers > MaxFingerForce ? Math.Max(0f, a.Curl - k_CurlRate * ts)
                    : on
                        ? fingers < 0.8f * MaxFingerForce
                                ? Math.Min(a.Curl + k_CurlRate * ts, k_MaxCurl)
                            : a.Curl
                    : held ? a.Curl
                    : Math.Max(0f, a.Curl - 2f * k_CurlRate * ts);
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

        // Nearest box over the conveyor still uphill of the pelvis (and not failed MaxBoxAttempts times): the next to catch; -1 if none.
        private int IncomingBox()
        {
            int best = -1;
            float bestAt = float.MinValue;
            for (int i = 0; i < m_Boxes.Count; i++)
            {
                float at = Vector3.Dot(BoxCenter(i) - Pelvis(), m_Downhill); // - = uphill of the pelvis
                if (
                    OverChute(BoxCenter(i))
                    && at < 0f
                    && at > bestAt
                    && m_Fails[i] < MaxBoxAttempts
                )
                {
                    best = i;
                    bestAt = at;
                }
            }
            return best;
        }

        // Fastest a box can speed up sliding down the belt (frictionless): g sin(tilt).
        private float SlideAccel() => 9.81f * Math.Abs(m_Downhill.Y);

        // Point p lies over the conveyor's top face.
        private bool OverChute(Vector3 p)
        {
            TransformComponent t = ChuteEntity!.Transform;
            Vector3 d = t.WorldRotationQuat.Conjugate * (p - t.WorldTranslation);
            return d.Y > 0f
                && Math.Abs(d.X) < 0.5f * t.WorldScale.X
                && Math.Abs(d.Z) < 0.5f * t.WorldScale.Z;
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

        // The table spot nearest from: on the line PlaceEdgeInset in from the table top's near edge, PlaceEdgeInset in from its ends (where
        // the robot walks to; the box goes down wherever it is over the table, no free-slot search for now).
        private Vector3 TableSpot(Vector3 from)
        {
            TransformComponent t = PlaceTarget!.Transform;
            Vector3 face = FaceOf(PlaceTarget),
                along = LeftOf(face);
            float halfLen = 0.5f * Math.Max(t.WorldScale.X, t.WorldScale.Z) - PlaceEdgeInset,
                halfWidth = 0.5f * Math.Min(t.WorldScale.X, t.WorldScale.Z);
            Vector3 line = TopOf(PlaceTarget) - face * (halfWidth - PlaceEdgeInset);
            return line
                + along * Mathf.Clamp(Vector3.Dot(Flat(from - line), along), -halfLen, halfLen);
        }

        // Whether point p is over the table top, at least inset in from its edges.
        private bool OverTable(Vector3 p, float inset)
        {
            TransformComponent t = PlaceTarget!.Transform;
            Vector3 face = FaceOf(PlaceTarget),
                d = Flat(p - TopOf(PlaceTarget));
            return Math.Abs(Vector3.Dot(d, LeftOf(face)))
                    <= 0.5f * Math.Max(t.WorldScale.X, t.WorldScale.Z) - inset
                && Math.Abs(Vector3.Dot(d, face))
                    <= 0.5f * Math.Min(t.WorldScale.X, t.WorldScale.Z) - inset;
        }

        // Collision geoms of one hand (wrist_yaw_link + hand links) for contact force sensing, and which of them are thumb and palm links.
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
        // (straight thumb across the front face, fingers along the side) + m_Slide (- = in front of the box; not for a holding hand), GripHeight up, + m_Lift; while
        // reaching for the box, never lower than GripHeight over the conveyor surface under it.
        private Vector3 HandTarget(Arm a)
        {
            Vector3 c = m_Carrying ? CarryPoint(m_Center) : m_Center;
            Vector3 lat = m_Carrying ? CarryDir(m_Lat) : m_Lat,
                fwd = m_Carrying ? CarryDir(m_Fwd) : m_Fwd;
            Vector3 t =
                c
                + lat * (a.Side * (m_HalfW + PalmOffset + a.Gap))
                + fwd * (FrontInset - m_HalfD + (a == m_Holder ? 0f : m_Slide))
                + Vector3.Up * (GripHeight(a) - m_HalfH + m_Lift);
            if (a == m_Holder)
                return t + lat * m_HoldOff.X + fwd * m_HoldOff.Y + Vector3.Up * m_HoldOff.Z;
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

        // Thumbs flared back (ThumbOpenDeg) except while gripping, when they swing to ThumbGripDeg past straight with the tip bent ThumbTipDeg; index + middle follow a.Curl while
        // gripping (Squeeze curls them in) and ease straight otherwise. Everything moves at a set rate - nothing snaps into the box.
        private void PoseHands(float ts)
        {
            bool grip = Gripping();
            foreach (Arm a in m_Arms)
            {
                if (a.Hand.Length < 4)
                    continue;
                float open = Math.Clamp(ThumbOpenDeg, 0f, 41.5f) * Mathf.Deg2Rad,
                    shut = Math.Clamp(ThumbGripDeg, 0f, 60f) * Mathf.Deg2Rad,
                    tip = Math.Clamp(ThumbTipDeg, 0f, 100f) * Mathf.Deg2Rad;
                a.Thumb += Mathf.Clamp(
                    (grip ? -shut : open) - a.Thumb,
                    -k_ThumbRate * ts,
                    k_ThumbRate * ts
                );
                a.Tip += Mathf.Clamp(
                    (grip ? tip : 0f) - a.Tip,
                    -k_ThumbRate * ts,
                    k_ThumbRate * ts
                );
                if (!grip)
                    a.Curl = Math.Max(0f, a.Curl - 2f * k_CurlRate * ts);
                a.Hand[0].Value = -a.Side * a.Thumb;
                a.Hand[1].Value = a.Side * a.Tip;
                a.Hand[2].Value = a.Hand[3].Value = -a.Side * a.Curl;
            }
        }

        // Height of the conveyor's top plane under point p.
        private float SurfaceY(Vector3 p) =>
            m_ChuteTop.Y
            - (m_ChuteN.X * (p.X - m_ChuteTop.X) + m_ChuteN.Z * (p.Z - m_ChuteTop.Z)) / m_ChuteN.Y;

        // Wrist servo: turns each gripping hand toward its zero-pose frame on the grip axes - fingers (X) along forward, Y along the squeeze
        // axis, so the left palm (-Y) and right palm (+Y) face the box. Closed on the measured hand frame, whatever the IK does. A hand not
        // gripping (or parked through the catch) isn't steered: its wrist eases back to neutral.
        // A hand still climbing out from low down keeps its wrist still (turning there would swing the fingers into the belt); LockWrists keeps both still always.
        // Once the grip is confirmed (until Release) the box fixes the hands' orientation, so the wrists hold the angles they had then: steering on
        // only winds the command up to the joint limit, pinning the actuator at its torque limit with no stiffness or damping left to stop the box swinging.
        private void OrientWrists(float ts)
        {
            bool grip = m_Phase >= Phase.Ready && m_Phase <= Phase.Release,
                held = m_Phase > Phase.Squeeze && m_Phase < Phase.Release;
            foreach (Arm a in m_Arms)
            {
                if (held && !a.WristHeld && a.Joints.Length == 7)
                    for (int i = 0; i < 3; i++)
                        a.Joints[4 + i].Value = a.WristQ[i] = m_Mujoco!.GetJointPosition(
                            a.WristAdr[i]
                        );
                a.WristHeld = held;
                if (a.Joints.Length < 7)
                    continue;
                if (!LockWrists && (!grip || Parked(a)))
                {
                    for (int i = 0; i < 3; i++) // no steering: back to neutral (steering a parked hand wound its wrist to 73-93 deg)
                        a.Joints[4 + i].Value = a.WristQ[i] =
                            a.WristQ[i]
                            - Mathf.Clamp(a.WristQ[i], -k_WristRate * ts, k_WristRate * ts);
                    continue;
                }
                if (a.Rising || SiteWorld(a.Site).Y < a.Cmd.Y - k_Snap)
                    continue;
                Vector3 x = grip && !Parked(a) ? (m_Carrying ? BodyDir(m_Fwd) : m_Fwd) : Heading(),
                    y = LeftOf(x),
                    z = Vector3.Cross(x, y);
                Quaternion r = BodyRot(a.WristLink[2]);
                Vector3 cy = r * s_Axes[1],
                    w =
                        (
                            Vector3.Cross(r * s_Axes[0], x)
                            + Vector3.Cross(cy, y)
                            + Vector3.Cross(r * s_Axes[2], z)
                        ) * 0.5f; // rotation error
                a.PalmErr = Mathf.Acos(Mathf.Clamp(Vector3.Dot(cy, y), -1f, 1f)) * Mathf.Rad2Deg;
                if (LockWrists || held)
                    continue;
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

        // Wrist roll / pitch / yaw: commanded vs measured (deg) - the actuators' torque limits let contact loads push the joints off their command.
        private string WristLog(Arm a) =>
            $"wrist cmd [{string.Join(", ", Array.ConvertAll(a.WristQ, v => (v * Mathf.Rad2Deg).ToString("F0")))}]"
            + $" measured [{string.Join(", ", Array.ConvertAll(a.WristAdr, i => (m_Mujoco!.GetJointPosition(i) * Mathf.Rad2Deg).ToString("F0")))}] deg";

        // Shoulder pitch / roll / yaw + elbow (measured, deg and deg/s since the last line) and the grasp site vs its IK target in pelvis coords
        // (forward, up, left; velocity in that frame) - shows how the arms swing on the body.
        private string ArmLog(Arm a, float dt)
        {
            float[] q = Array.ConvertAll(a.ArmAdr, i => (float)m_Mujoco!.GetJointPosition(i));
            Vector3 site = ToBody(SiteWorld(a.Site) - Pelvis());
            string log =
                $"arm (sh pitch, roll, yaw, elbow) [{string.Join(", ", Array.ConvertAll(q, v => (v * Mathf.Rad2Deg).ToString("F0")))}] deg"
                + $" rate [{string.Join(", ", Array.ConvertAll(new[] { 0, 1, 2, 3 }, i => ((q[i] - a.LastArmQ[i]) / dt * Mathf.Rad2Deg).ToString("F0")))}] deg/s"
                + $" | site on body (fwd, up, left) {F(site)} v {F((site - a.LastSite) / dt)} m/s, IK target {F(ToBody(HandTarget(a) - Pelvis()))}";
            a.LastArmQ = q;
            a.LastSite = site;
            return log;
        }

        // Measured joint angles (deg) at these qpos addresses, for the log.
        private string Degs(uint[] adr) =>
            string.Join(
                ", ",
                Array.ConvertAll(
                    adr,
                    i => (m_Mujoco!.GetJointPosition(i) * Mathf.Rad2Deg).ToString("F0")
                )
            );

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
                if (
                    Parked(a)
                    || !grip && !newMove && a.Tau >= a.T && target.Method.Name == nameof(Fold)
                )
                {
                    // Folded: follow the body's fold point with a k_FoldSmooth lag - drift is followed, bounces never reach the arms (and never
                    // come back through them into the walker), and no path is re-planned. A hand parked through the catch also moves no faster
                    // than k_ParkSpeed: a quick parked arm shook the body while the other hand waited for the box.
                    Vector3 step = (goal - a.Cmd) * (1f - (float)Math.Exp(-m_Dt / k_FoldSmooth));
                    if (Parked(a) && step.Length() > k_ParkSpeed * m_Dt)
                        step *= k_ParkSpeed * m_Dt / step.Length();
                    a.Cmd += step;
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

        // Sends a hand's IK target and elbow pole. Cap: a target beyond MaxReach of the shoulder is pulled in toward it (logged when the cap
        // starts and stops).
        private void SendTarget(Arm a)
        {
            Vector3 t = a.Cmd,
                d = t - BodyPos(a.Shoulder);
            float max = MaxReach * k_ArmLength;
            if (d.Length() > max != a.Capped)
                Log.Warn(
                    $"[G1] {a.Name} IK target {(d.Length() > max ? $"{d.Length() / k_ArmLength * 100f:F0}% of the arm away -> pulled in to {MaxReach * 100f:F0}%" : "back within reach")} in {m_Phase}"
                );
            a.Capped = d.Length() > max;
            m_Robot!.SetInputVector3(a.Target, a.Capped ? t - d * (1f - max / d.Length()) : t);
            Vector3 pole = Mirror(PoleOffset + new Vector3(0f, 0f, m_PoleOut), a.Side);
            m_Robot.SetInputVector3(a.Pole, m_Carrying ? CarryPoint(pole) : BodyPoint(pole));
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
                    or Phase.DragIn
                    or Phase.Holding
                    or Phase.Lift
                    or Phase.PullIn
                    or Phase.BackOff
                    or Phase.TurnToTable
                    or Phase.WalkToTable
                    or Phase.FaceTable
                    or Phase.Place;

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

        // Carrying, first move: settle, back straight away from the conveyor (ramped in) until its coast short of CarryBackDistance behind the start (at least half way) (the sway
        // after stopping goes either way, ~4 cm), then coast to a stop. True when stopped.
        private bool StepBackOff()
        {
            switch (m_Step)
            {
                case 0:
                    if (m_StepTime < k_Settle)
                        return false;
                    m_BackFrom = m_Anchor; // from where it gripped: any drift counts toward the back-off
                    break;
                case 1:
                    Drive(-CarryBackSpeed * Math.Min(1f, m_StepTime / k_CarryRamp), 0f, 0f);
                    if (
                        Vector3.Dot(Flat(m_BackFrom - Pelvis()), Heading())
                            < Math.Max(
                                CarryBackDistance - CarryBackSpeed * k_BackCoast,
                                0.5f * CarryBackDistance
                            )
                        && m_StepTime < k_TurnMax
                    )
                        return false;
                    m_Anchor = m_BackFrom - Heading() * CarryBackDistance; // from here on the body holds the back-off spot
                    m_Restoring = false;
                    break;
                default:
                    if (m_StepTime < k_ApCoast)
                        return false;
                    Log.Info(
                        $"[G1] backed off {Vector3.Dot(Flat(m_BackFrom - Pelvis()), Heading()):F2} m from the conveyor (aim {CarryBackDistance:F2})"
                    );
                    return true;
            }
            NextStep();
            return false;
        }

        // Carrying to the table: lift the box (k_CarrySpeed, at most k_MaxRaise over CarryOffset) while its measured bottom is under the
        // table top + k_TableClear, so walking in never knocks it on the table's edge.
        private void RaiseOverTable(float ts)
        {
            if (
                BoxCenter(m_Box).Y - m_HalfH < m_Slot.Y + k_TableClear
                && m_Center.Y < CarryOffset.Y + k_MaxRaise
            )
                m_Center.Y += k_CarrySpeed * ts;
        }

        // Walker at zero (the caller's) until the box stops swinging in the hands - spin under k_SteadySpin - or k_SteadyMax has passed.
        private bool BoxSteady()
        {
            m_Mujoco!.GetBodyPoseMj(m_Boxes[m_Box].Body, out _, out _, out _, out Vector3 w);
            if (w.Length() >= k_SteadySpin && m_StepTime < k_SteadyMax)
                return false;
            Log.Info(
                $"[G1] box steady in the hands after {m_StepTime:F2} s in {m_Phase} (spin {w.Length():F2} rad/s)"
            );
            return true;
        }

        // Carrying: wait for the box to stop swinging, then walk straight at the stand spot - forward at CarryWalkSpeed (ramped in), steering onto
        // it - until k_WalkCoast short of it (at k_FwdCmd; with the square of the speed when faster: the body coasts on), then coast to a stop. True when stopped.
        private bool CarryWalk()
        {
            Vector3 to = Flat(m_Stand - Pelvis());
            m_ToGo = to.Length();
            if (m_Step == 0)
            {
                if (!m_Restoring && BoxSteady())
                    NextStep();
                return false;
            }
            if (m_Step == 1)
            {
                Drive(
                    CarryWalkSpeed * Math.Min(1f, m_StepTime / k_CarryRamp),
                    0f,
                    Mathf.Clamp(
                        Mathf.WrapToPi(YawOf(to) - YawOf(Heading())) * k_YawGain,
                        -k_MaxYawRate,
                        k_MaxYawRate
                    )
                );
                if (
                    Vector3.Dot(to, Heading())
                        > k_WalkCoast * (CarryWalkSpeed / k_FwdCmd) * (CarryWalkSpeed / k_FwdCmd)
                    && m_StepTime < WalkTimeout
                )
                    return false;
                m_Anchor = m_Stand; // from here on (coast, FaceTable) the body holds the stand spot
                m_Restoring = false;
                NextStep();
                return false;
            }
            if (m_StepTime < k_ApCoast)
                return false;
            Log.Info($"[G1] walked to the table: {m_ToGo:F2} m off the stand spot");
            return true;
        }

        // Turn in place on the walker with the box (the rotator's take-over lurches the body): settle and wait for the box to stop swinging; if more than tol off, yaw at CarryTurnCmd
        // (eased in over CarryTurnRamp, eased down to k_TurnFloor of it over the last k_TurnEase) until k_TurnStop short (it coasts on), ramp the command out over
        // k_CarryRamp, coast to a stop and check again (MaxTurnTries). True when facing yaw, or out of
        // tries (it goes on from there: the box is never let go over a turn).
        private bool WalkerTurn(float yaw, float tol)
        {
            if (m_Step == 0)
                m_TurnYaw = yaw; // aim fixed once the turn starts, so body sway doesn't move it
            float err = Mathf.WrapToPi(m_TurnYaw - YawOf(Heading()));
            switch (m_Step)
            {
                case 0:
                    if (m_StepTime < k_Settle || m_Restoring || !BoxSteady())
                        return false;
                    if (Math.Abs(err) < tol)
                        return true;
                    if (++m_TurnTries > MaxTurnTries)
                    {
                        Log.Warn(
                            $"[G1] turn still {err * Mathf.Rad2Deg:F0} deg off after {MaxTurnTries} tries -> going on from here"
                        );
                        return true;
                    }
                    Log.Info(
                        $"[G1] turning {err * Mathf.Rad2Deg:F0} deg in place on the walker ({m_TurnTries}/{MaxTurnTries})"
                    );
                    break;
                case 1:
                    float ramp = Math.Min(1f, m_StepTime / CarryTurnRamp);
                    m_TurnCmd =
                        Math.Sign(err)
                        * CarryTurnCmd
                        * Math.Min(
                            ramp * ramp * (3f - 2f * ramp),
                            Mathf.Clamp(Math.Abs(err) / k_TurnEase, k_TurnFloor, 1f)
                        );
                    Drive(0f, 0f, m_TurnCmd);
                    if (
                        Math.Abs(err) > k_TurnStop
                        && !Stalled(Math.Abs(err), k_TurnProgress, PhaseTimeout)
                    )
                        return false;
                    break;
                default:
                    Drive(0f, 0f, m_TurnCmd * Math.Max(0f, 1f - m_StepTime / k_CarryRamp));
                    if (m_StepTime < k_ApCoast)
                        return false;
                    m_Step = 0; // coasted to a stop: check again
                    m_StepTime = 0f;
                    ResetWatch();
                    return false;
            }
            NextStep();
            return false;
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
            m_Robot!.SetDrivenJoints(slot, m_WaistLocked ? s_LegsOnly : s_LegsAndWaistOnly);
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
            m_TurnTries = 0;
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
        // retry = go back for the next box afterwards, else stop for good. A box being carried is never let go: the body stands holding it.
        private void Fail(string reason, bool retry = true, bool skip = false)
        {
            Log.Error($"[G1] failed in {m_Phase}: {reason}");
            if (m_Rec.FailReason == "")
                m_Rec.FailReason = $"{m_Phase}: {reason}"; // the episode's first failure, for its sidecar
            if (m_Carrying)
            {
                Log.Warn("[G1] still carrying the box -> standing still holding it");
                Advance(Phase.Holding);
                return;
            }
            if (retry && m_Box >= 0 && (skip || ++m_Fails[m_Box] >= MaxBoxAttempts))
            {
                m_Fails[m_Box] = MaxBoxAttempts;
                Log.Warn($"[G1] skipping box {m_Box} until it moves");
            }
            m_Retry = retry;
            m_Holder = null;
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

        // Squeeze force per hand that holds the box: factor * m*g / (2*mu), boosted while slipping, capped so a palm never presses past MaxPalmForce.
        private float RequiredForce() =>
            Math.Min(
                GripSafetyFactor
                    * m_Mujoco!.GetBodyMass(m_Boxes[m_Box].Body)
                    * 9.81f
                    / (2f * BoxFriction)
                    * m_Boost,
                MaxPalmForce / k_SqueezeMargin
            );

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

        // The same in the carry frame (TrackCarryFrame); equal to the pelvis frame when not carrying.
        private Vector3 CarryDir(Vector3 b)
        {
            Vector3 h = new Vector3((float)Math.Cos(m_CarryYaw), 0f, -(float)Math.Sin(m_CarryYaw));
            return h * b.X + Vector3.Up * b.Y + LeftOf(h) * b.Z;
        }

        private Vector3 CarryPoint(Vector3 b) => m_CarryPos + CarryDir(b);

        // Carry frame: the pelvis position and heading low-passed over CarrySmoothing with their own smoothed velocity fed forward, so a steady
        // walk or turn isn't lagged while gait bounce and sway never reach the arms (the box swinging on them would push back into the walker).
        private void TrackCarryFrame(float ts)
        {
            Vector3 p = Pelvis();
            float yaw = YawOf(Heading());
            if (!m_Carrying || CarrySmoothing <= 0f)
            {
                m_CarryPos = p;
                m_CarryYaw = yaw;
                m_CarryVel = Vector3.Zero;
                m_CarryRate = 0f;
            }
            else
            {
                float k = 1f - (float)Math.Exp(-ts / CarrySmoothing);
                m_CarryVel += ((p - m_LastPelvis) * (1f / ts) - m_CarryVel) * k;
                m_CarryRate += (Mathf.WrapToPi(yaw - m_LastYaw) / ts - m_CarryRate) * k;
                m_CarryPos += (p + m_CarryVel * CarrySmoothing - m_CarryPos) * k;
                m_CarryYaw = Mathf.WrapToPi(
                    m_CarryYaw + Mathf.WrapToPi(yaw + m_CarryRate * CarrySmoothing - m_CarryYaw) * k
                );
            }
            m_LastPelvis = p;
            m_LastYaw = yaw;
        }

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
