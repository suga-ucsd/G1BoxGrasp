using System.Collections.Generic;
using Hazel;

namespace G1Sort
{
    /// <summary>
    /// Runtime state of G1BoxGrasp: phase enums, constants, and every private field
    /// the phase files share. Nothing here is visible in the Inspector.
    /// </summary>
    public partial class G1BoxGrasp
    {
        // ---- Walker command slots (RobotControllerComponent.SetFloat) -----
        private const uint k_SetVx = 1u;
        private const uint k_SetVy = 2u;
        private const uint k_SetYawRate = 3u;
        private const float k_G = 9.81f;

        /// <summary>Top-level phases. Only Advance() (Recovery.cs) changes m_Phase.</summary>
        private enum Phase { WalkTo, Wait, PreGrasp, CatchReady, Catch, Grasp, Lift, Holding, SetDown, Release, ScoopRequired, Recover, Failed }

        /// <summary>Sub-stages of Phase.Grasp (Grasp.cs).</summary>
        private enum GraspStage { Approach, Contact, Close }

        // ---- Components and robot -----------------------------------------
        [HideFromEditor] private RobotControllerComponent? m_Robot;
        [HideFromEditor] private MujocoSceneComponent? m_Mujoco;
        [HideFromEditor] private uint m_PelvisBodyId = uint.MaxValue;

        [HideFromEditor] private ArmChain m_Left = new ArmChain("left", 1f);
        [HideFromEditor] private ArmChain m_Right = new ArmChain("right", -1f);
        [HideFromEditor] private BoxTracker m_Tracker = new BoxTracker();
        [HideFromEditor] private Identifier m_DurationId, m_StartId;

        // ---- Phase bookkeeping ----------------------------------------------
        [HideFromEditor] private Phase m_Phase = Phase.WalkTo;
        [HideFromEditor] private float m_PhaseElapsed;       // reset by Advance()
        [HideFromEditor] private float m_TimeSinceStart;
        [HideFromEditor] private int m_Retries;
        [HideFromEditor] private bool m_RecoverCounts;       // this Recover counts as a retry
        [HideFromEditor] private bool m_LoggedThisPhase;     // Fail() logs once per phase
        [HideFromEditor] private string m_LastFailure = "";
        [HideFromEditor] private bool m_PreGraspAchieved;
        [HideFromEditor] private bool m_IdleChecked;

        // ---- Force tare (ContactSensing.cs) ---------------------------------
        [HideFromEditor] private Dictionary<uint, float> m_Base = new Dictionary<uint, float>(); // geom id -> baseline force
        [HideFromEditor] private bool m_Tared;

        // ---- External perception estimate (PushBoxEstimate) -----------------
        [HideFromEditor] private bool m_ExtValid;
        [HideFromEditor] private float m_ExtAge = 1000f;
        [HideFromEditor] private Vector3 m_ExtCenter;
        [HideFromEditor] private Quaternion m_ExtRot = Quaternion.Identity;
        [HideFromEditor] private Vector3 m_ExtSize;

        // ---- Fold pose frame (HandTargeting.cs) -----------------------------
        [HideFromEditor] private Vector3 m_PlanMid;
        [HideFromEditor] private Vector3 m_PlanLeft = new Vector3(0f, 0f, -1f);

        // ---- Approach direction (BoxModel.cs / Locomotion.cs) ---------------
        [HideFromEditor] private bool m_ApproachLatched;
        [HideFromEditor] private Vector3 m_ApproachDir = new Vector3(1f, 0f, 0f);
        [HideFromEditor] private bool m_InFinalApproach;

        // ---- Live box face frame (UpdateFaceFrame, every frame) -------------
        [HideFromEditor] private bool m_FaceValid;
        [HideFromEditor] private Vector3 m_FaceCenter;
        [HideFromEditor] private Vector3 m_FaceNL = new Vector3(0f, 0f, -1f);  // left (lateral) axis
        [HideFromEditor] private Vector3 m_FaceNF = new Vector3(1f, 0f, 0f);   // forward (approach) axis
        [HideFromEditor] private float m_FaceHalfW;
        [HideFromEditor] private float m_FaceHalfD;
        [HideFromEditor] private float m_FaceHalfH;

        // ---- Latched grasp frame (set once the arms commit) -----------------
        [HideFromEditor] private bool m_FrameLatched;
        [HideFromEditor] private Vector3 m_LatchedLat = new Vector3(0f, 0f, -1f);
        [HideFromEditor] private Vector3 m_LatchedFwd = new Vector3(1f, 0f, 0f);

        // ---- Target sending / timers ----------------------------------------
        [HideFromEditor] private Vector3 m_LastCommandedL;
        [HideFromEditor] private Vector3 m_LastCommandedR;
        [HideFromEditor] private float m_RetargetTimer;
        [HideFromEditor] private float m_HoldTime;
        [HideFromEditor] private float m_LogTimer;
        [HideFromEditor] private float m_WalkLogTimer;

        // ---- Grasp phase (Grasp.cs) -----------------------------------------
        [HideFromEditor] private GraspStage m_GraspStage;
        [HideFromEditor] private float m_GraspStageElapsed;
        [HideFromEditor] private float m_GraspHoldTime;
        [HideFromEditor] private bool m_Moving;              // grasp came from a moving-box catch
        [HideFromEditor] private Vector3 m_GraspCenter;       // latched box centre for hand targets

        // ---- Moving-box catch (MovingCatch.cs) ------------------------------
        [HideFromEditor] private Vector3 m_CatchDir = new Vector3(1f, 0f, 0f);
        [HideFromEditor] private float m_StopperSide = 1f;   // +1 left hand is the stopper, -1 right
        [HideFromEditor] private Vector3 m_CatchCenter;

        // ---- Lift / hold / set down (LiftAndRelease.cs) ---------------------
        [HideFromEditor] private Vector3 m_LiftCenter;
        [HideFromEditor] private float m_LiftH;
        [HideFromEditor] private float m_BoxY0;
        [HideFromEditor] private float m_PalmY0;
        [HideFromEditor] private float m_ForceBoost = 1f;
        [HideFromEditor] private float m_SlipTimer;
        [HideFromEditor] private float m_LiftSettle;
        [HideFromEditor] private float m_LastSlip;
        [HideFromEditor] private bool m_NeedScoop;
        [HideFromEditor] private string m_SetDownReason = "";

        /// <summary>Joints the walking policy drives. Arms stay free for IK / direct drive.</summary>
        private static readonly string[] s_LegsAndWaistOnly = {
            "left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint",
            "left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
            "right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint",
            "right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
            "waist_yaw_joint","waist_roll_joint","waist_pitch_joint",
        };

        // ---- Small accessors ------------------------------------------------

        /// <summary>0 = left arm, 1 = right arm.</summary>
        private ArmChain Arm(int i) => i == 0 ? m_Left : m_Right;

        /// <summary>+1 = left arm, -1 = right arm.</summary>
        private ArmChain ArmBySide(float side) => side > 0f ? m_Left : m_Right;

        /// <summary>Target frame lateral axis: the box's real (snapped) face frame, or the latched one if the box is not tracked.</summary>
        private Vector3 CurLat => m_FaceValid ? m_FaceNL : m_LatchedLat;

        /// <summary>Target frame forward axis (see CurLat).</summary>
        private Vector3 CurFwd => m_FaceValid ? m_FaceNF : m_LatchedFwd;
    }

}
