using System.Collections.Generic;
using Hazel;

namespace G1Pipe
{
    // Runtime state of the controller (reset between recorded episodes).
    public partial class G1BoxGrasp
    {
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

        private readonly bool[] m_WaistIK = new bool[3]; // the waist actuators' IK flags before the lock (given back with the waist)

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

        private float m_DragLeft; // m the grip still has to be dragged in (fixed when the drag starts)

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
    }
}
