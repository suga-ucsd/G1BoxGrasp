using Hazel;

namespace G1Pipe
{
    // Fixed constants: joint lists, policy command IDs, rates, gains and margins.
    public partial class G1BoxGrasp
    {
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
    }
}
