using Hazel;

namespace G1Pipe
{
    // Inspector fields: every tunable setting, grouped as in the editor ([Group]).
    public partial class G1BoxGrasp
    {
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
    }
}
