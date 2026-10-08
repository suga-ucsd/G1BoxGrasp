using System;
using Hazel;

namespace G1Sort
{
    /// <summary>
    /// G1 dual-arm corner-wrap box grasp.
    ///
    /// GRIP LAYOUT
    ///   thumb                  -> front face (the face toward the G1)
    ///   palm + index + middle  -> that hand's side face
    ///   wrists locked; arms place the hands; fingers close.
    ///
    /// PHASES (see State.cs for the enum)
    ///   Static box: WalkTo -> PreGrasp -> Grasp(Approach, Contact, Close) -> Lift -> Holding
    ///   Moving box: WalkTo(station) -> Wait -> CatchReady -> Catch -> Grasp(Contact, Close) -> Lift -> Holding
    ///   Slip at max squeeze: SetDown -> Release -> ScoopRequired (terminal)
    ///   Other failures:      Recover -> WalkTo (retry) | Failed
    ///
    /// FILE MAP (all files are this same partial class unless noted)
    ///   G1BoxGrasp.cs                 this file: public API, OnCreate, OnUpdate
    ///   G1BoxGrasp.Settings.cs        Inspector fields + read-only status properties
    ///   G1BoxGrasp.State.cs           enums, runtime state, small accessors
    ///   G1BoxGrasp.Setup.cs           finger poses, hand geom resolution
    ///   G1BoxGrasp.RobotFrame.cs      pelvis frame, walk command, heading control
    ///   G1BoxGrasp.BoxModel.cs        box pose/size/mass, face frame, force target
    ///   G1BoxGrasp.ContactSensing.cs  raw/tared forces, idle contact check
    ///   G1BoxGrasp.Locomotion.cs      WalkTo, Wait, arrival decision
    ///   G1BoxGrasp.HandTargeting.cs   fold pose, hand IK targets, thumb placement
    ///   G1BoxGrasp.PreGrasp.cs        PreGrasp phase
    ///   G1BoxGrasp.MovingCatch.cs     CatchReady + Catch phases
    ///   G1BoxGrasp.Grasp.cs           Grasp phase and grip control loops
    ///   G1BoxGrasp.LiftAndRelease.cs  Lift, Holding, SetDown, Release
    ///   G1BoxGrasp.Recovery.cs        Fail, Recover, Advance
    ///   G1BoxGrasp.Diagnostics.cs     console status lines
    ///   Helpers/Geo.cs, Helpers/BoxTracker.cs, Robot/Finger.cs, Robot/ArmChain.cs  (own classes)
    ///
    /// Attach to the robot root entity (RobotControllerComponent + MujocoSceneComponent),
    /// assign TargetObject (a MuJoCo box), press Play.
    /// </summary>
    public partial class G1BoxGrasp : Entity
    {
        // ==================================================================
        // Public API (callable from spawner / sorter / perception scripts)
        // ==================================================================

        /// <summary>
        /// Switch to a new target box. Resets tracker, retries and approach latch.
        /// If called mid-task, the robot folds its arms first (Recover, not counted as a retry).
        /// </summary>
        public void SetTarget(Entity? box)
        {
            TargetObject = box;
            m_Tracker.Reset();
            m_Retries = 0;
            m_ApproachLatched = false;
            m_PreGraspAchieved = false;
            if (m_Phase != Phase.WalkTo && m_Phase != Phase.Recover)
                EnterRecover("new target", false);
        }

        /// <summary>Same as SetTarget(box), also sets the box size (box-local X, Y = height, Z).</summary>
        public void SetTarget(Entity? box, Vector3 sizeLocal)
        {
            BoxSize = sizeLocal;
            SetTarget(box);
        }

        /// <summary>
        /// Perception entry point. Used only when UseExternalEstimate is on.
        /// The estimate goes stale after ExternalEstimateTimeout.
        /// </summary>
        public void PushBoxEstimate(Vector3 center, Quaternion rotation, Vector3 sizeLocal)
        {
            m_ExtCenter = center;
            m_ExtRot = rotation;
            m_ExtSize = sizeLocal;
            m_ExtValid = true;
            m_ExtAge = 0f;
        }

        // ==================================================================
        // Engine callbacks
        // ==================================================================

        /// <summary>
        /// Runs once at scene start:
        ///  1. get components and pelvis body id,
        ///  2. bind both arms, set finger poses, resolve hand geoms (Setup.cs),
        ///  3. activate the walker on legs + waist only,
        ///  4. configure IK inputs and joint IK flags, report missing bindings,
        ///  5. send the folded arm pose.
        /// </summary>
        protected override void OnCreate()
        {
            m_Robot = GetComponent<RobotControllerComponent>();
            m_Mujoco = GetComponent<MujocoSceneComponent>();
            m_PelvisBodyId = m_Mujoco != null ? m_Mujoco.GetBodyID(PelvisBodyName) : uint.MaxValue;

            m_DurationId = new Identifier(DurationInput);
            m_StartId = new Identifier(StartInput);

            if (m_Mujoco != null)
            {
                m_Left.Bind(m_Mujoco, LeftInputPrefix);
                m_Right.Bind(m_Mujoco, RightInputPrefix);
                ConfigureFingerPoses();
                ResolveGeoms();
            }

            m_Robot?.SetPolicyActive(WalkerSlotId, true);
            m_Robot?.SetDrivenJoints(WalkerSlotId, s_LegsAndWaistOnly);

            for (int i = 0; i < 2; i++)
            {
                ArmChain a = Arm(i);
                m_Robot?.SetInputBool(a.SolveOriId, false);
                m_Robot?.SetInputBool(a.HighAccId, true);
                a.ConfigureIK();
                for (int f = 0; f < a.Fingers.Length; f++)
                    if (!a.Fingers[f].Bound)
                        Log.Error("[G1BoxGrasp] " + a.Name + " " + a.Fingers[f].Label + ": finger actuator(s) not found - this finger cannot move.");
                if (a.Fingers.Length > 0 && a.Thumb.DistalBody == uint.MaxValue)
                    Log.Error("[G1BoxGrasp] " + a.Name + ": thumb_2_link not found - thumb placement/closing disabled for this hand.");
            }

            m_Tracker.Reset();
            BuildFoldPlan();
            CommandFold(PreBendDuration);
        }

        /// <summary>
        /// Runs every frame, in this order:
        ///  1. box estimate -> approach latch -> face frame   (BoxModel.cs)
        ///  2. one-time idle contact check                     (ContactSensing.cs)
        ///  3. joint drive: wrists locked, fingers from Curl, elbow folded in non-arm phases
        ///  4. walk diagnostics (optional)
        ///  5. the current phase's Step method
        ///  6. grasp status log (every 0.5 s in arm phases)
        /// </summary>
        protected override void OnUpdate(float ts)
        {
            if (m_Robot == null || m_Mujoco == null)
                return;
            m_PhaseElapsed += ts;
            m_TimeSinceStart += ts;
            UpdateEstimate(ts);
            LatchApproachIfNeeded();
            UpdateFaceFrame();

            if (IdleContactCheck && !m_IdleChecked && m_TimeSinceStart >= IdleCheckTime)
            {
                m_IdleChecked = true;
                if (m_Phase == Phase.WalkTo || m_Phase == Phase.Wait)
                    RunIdleContactCheck();
            }

            // Arms are "active" (IK-driven toward the box) in every phase except these.
            bool armsActive = m_Phase != Phase.WalkTo && m_Phase != Phase.Wait
                && m_Phase != Phase.Recover && m_Phase != Phase.Failed;

            for (int i = 0; i < 2; i++)
            {
                ArmChain a = Arm(i);
                a.LockWrist(WristRollDeg, WristPitchDeg, WristYawDeg, WristLockRateDegPerSec, ts);
                a.DriveFingers(MaxFingerJointRateDegPerSec, ts);
                if (!armsActive)
                    a.SetElbowDeg(ElbowCommandDeg(a), MaxJointRateDegPerSec, ts);
            }

            if (LogWalkStatus && (m_Phase == Phase.WalkTo || m_Phase == Phase.Wait))
            {
                m_WalkLogTimer += ts;
                if (m_WalkLogTimer >= 1.0f) { m_WalkLogTimer = 0f; LogWalkDiagnostics(); }
            }

            switch (m_Phase)
            {
                case Phase.WalkTo: StepWalkTo(); break;                                  // Locomotion.cs
                case Phase.Wait: Walk(0f, 0f, 0f); StepWait(); break;                   // Locomotion.cs
                case Phase.PreGrasp:                                                          // PreGrasp.cs
                    if (m_Tracker.Teleported && Fail("target teleported during pre-grasp")) break;
                    Walk(0f, 0f, FaceYawRate());
                    StepPreGrasp(ts);
                    break;
                case Phase.CatchReady: Walk(0f, 0f, 0f); StepCatchReady(ts); break;           // MovingCatch.cs
                case Phase.Catch: Walk(0f, 0f, 0f); StepCatch(); break;                  // MovingCatch.cs
                case Phase.Grasp:                                                             // Grasp.cs
                    if (m_Tracker.Teleported && Fail("target teleported during grasp")) break;
                    Walk(0f, 0f, m_Moving ? 0f : FaceYawRate());
                    StepGrasp(ts);
                    break;
                case Phase.Lift: Walk(0f, 0f, 0f); StepLift(ts); break;                 // LiftAndRelease.cs
                case Phase.Holding: Walk(0f, 0f, 0f); StepHolding(ts); break;              // LiftAndRelease.cs
                case Phase.SetDown: Walk(0f, 0f, 0f); StepSetDown(ts); break;              // LiftAndRelease.cs
                case Phase.Release: Walk(0f, 0f, 0f); StepRelease(ts); break;              // LiftAndRelease.cs
                case Phase.ScoopRequired: Walk(0f, 0f, 0f); break;                            // terminal
                case Phase.Recover:                                                           // Recovery.cs
                    Walk(0f, 0f, 0f);
                    if (m_PhaseElapsed >= RecoverDuration) FinishRecover();
                    break;
                case Phase.Failed: Walk(0f, 0f, 0f); break;                               // terminal
            }

            if (LogGraspStatus && armsActive && m_Phase != Phase.ScoopRequired)
            {
                m_LogTimer += ts;
                if (m_LogTimer >= 0.5f) { m_LogTimer = 0f; LogGraspDiagnostics(); }
            }
        }
    }
}

