using Hazel;

namespace G1Sort
{
    /// <summary>
    /// CatchReady and Catch phases (moving box on the belt).
    ///
    /// The DOWNSTREAM hand becomes a stopper: its palm waits at the box's downstream face
    /// at the catch point. The UPSTREAM hand hovers above the box's path. When the belt
    /// pushes the box into the stopper, the upstream hand lowers behind it and the normal
    /// Grasp phase starts at its Contact stage.
    /// </summary>
    public partial class G1BoxGrasp
    {
        /// <summary>Point on the box's path closest to the robot centreline: box + v*dot(pelvis - box, v).</summary>
        private Vector3 CatchPoint()
        {
            Vector3 box = m_Tracker.Position;
            float s = Geo.Dot(Geo.Flat(GetPelvis() - box), m_CatchDir);
            return box + m_CatchDir * s;
        }

        /// <summary>Lift that puts the upstream hand UpstreamRaiseClearance above the box top.</summary>
        private float UpstreamRaiseLift() =>
            (1f - ContactHeightFraction) * m_FaceHalfH - HandHeightOffset + UpstreamRaiseClearance;

        /// <summary>
        /// Latches the frame, checks the box fits, picks the stopper hand from the belt
        /// direction (downstream side), activates arms and sends stopper / raised targets.
        /// </summary>
        private void EnterCatchReady()
        {
            Vector3 fwd = GripForward();
            m_LatchedFwd = fwd;
            m_LatchedLat = Geo.LeftOf(fwd);
            m_FrameLatched = true;
            UpdateFaceFrame();
            if (!CheckBoxFits()) return;

            m_CatchDir = Geo.FlatDir(m_Tracker.Velocity, BeltDir());
            m_StopperSide = Geo.Dot(m_CatchDir, CurLat) >= 0f ? 1f : -1f;
            ActivateArms();
            ArmBySide(m_StopperSide).Gap = CatchStopperGap;
            m_CatchCenter = CatchPoint();
            SendCatchTargets(ApproachDuration);
            Log.Info("[G1BoxGrasp] moving box: " + (m_StopperSide > 0f ? "left" : "right") + " hand is the stopper, width="
                + (2f * m_FaceHalfW).ToString("F3") + " speed=" + Geo.Flat(m_Tracker.Velocity).Length().ToString("F3"));
            Advance(Phase.CatchReady);
        }

        /// <summary>Stopper hand at the downstream face; upstream hand raised above the box.</summary>
        private void SendCatchTargets(float duration)
        {
            Vector3 tStop = HandIkTarget(m_CatchCenter, m_StopperSide, CatchStopperGap, 0f, MinSqueezeHalfGap);
            Vector3 tUp = HandIkTarget(m_CatchCenter, -m_StopperSide, HandStandoff, UpstreamRaiseLift(), MinHalfGap);
            SendHandTargets(m_StopperSide > 0f ? tStop : tUp, m_StopperSide > 0f ? tUp : tStop, duration);
        }

        /// <summary>
        /// Phase.CatchReady step: track the catch point, tare once after ApproachDuration,
        /// place thumbs, wait for tared stopper contact. Fails if the box passes or on timeout.
        /// </summary>
        private void StepCatchReady(float ts)
        {
            if (!HasBox) return;
            m_CatchCenter = CatchPoint();

            if (m_PhaseElapsed >= ApproachDuration)
            {
                if (!m_Tared) Tare();
                ServoInset(m_Left, ts);
                ServoInset(m_Right, ts);
            }

            Vector3 tStop = HandIkTarget(m_CatchCenter, m_StopperSide, CatchStopperGap, 0f, MinSqueezeHalfGap);
            Vector3 tUp = HandIkTarget(m_CatchCenter, -m_StopperSide, HandStandoff, UpstreamRaiseLift(), MinHalfGap);
            RetargetIfDrifted(m_StopperSide > 0f ? tStop : tUp, m_StopperSide > 0f ? tUp : tStop,
                RetargetInterval, RetargetThreshold, RetargetMoveDuration, ts);

            if (m_Tared && InContactEff(ArmBySide(m_StopperSide).SideGeomIds))
            {
                EnterCatch();
                return;
            }

            float passed = Geo.Dot(Geo.Flat(m_Tracker.Position - m_CatchCenter), m_CatchDir);
            if (passed > m_FaceHalfW)
            {
                Fail("moving box passed the catch point without touching the stopper");
                return;
            }
            if (m_PhaseElapsed >= CatchTimeout)
                Fail("catch timeout");
        }

        /// <summary>Latches the stopped box centre and lowers the upstream hand behind it.</summary>
        private void EnterCatch()
        {
            m_CatchCenter = m_Tracker.Position;
            Vector3 tStop = HandIkTarget(m_CatchCenter, m_StopperSide, CatchStopperGap, 0f, MinSqueezeHalfGap);
            Vector3 tUp = HandIkTarget(m_CatchCenter, -m_StopperSide, HandStandoff, 0f, MinSqueezeHalfGap);
            SendHandTargets(m_StopperSide > 0f ? tStop : tUp, m_StopperSide > 0f ? tUp : tStop, CatchLowerDuration);
            Log.Info("[G1BoxGrasp] box stopped against stopper hand - lowering upstream hand");
            Advance(Phase.Catch);
        }

        /// <summary>Phase.Catch step: after CatchLowerDuration, start the grasp.</summary>
        private void StepCatch()
        {
            if (m_PhaseElapsed >= CatchLowerDuration)
                EnterGrasp(true);
        }
    }

}
