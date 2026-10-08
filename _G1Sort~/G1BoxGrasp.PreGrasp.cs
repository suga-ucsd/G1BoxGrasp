using Hazel;

namespace G1Sort
{
    /// <summary>
    /// PreGrasp phase (static box).
    ///
    /// Hands move to the front corners, HandStandoff outside each side face, fingers open.
    /// After ApproachDuration the inset servo places each open thumb in front of the front face.
    /// Confirms (held for HoldTimeToConfirm) when both hand positions are within
    /// PositionTolerance AND both thumbs are placed, then enters Grasp.
    /// </summary>
    public partial class G1BoxGrasp
    {
        /// <summary>Activates the arms, unlatches the frame, sends standoff targets, enters PreGrasp.</summary>
        private void EnterPreGrasp()
        {
            ActivateArms();
            m_HoldTime = 0f;
            m_FrameLatched = false;
            SendHandTargets(HandIkTarget(m_FaceCenter, 1f, HandStandoff, 0f, MinHalfGap),
                HandIkTarget(m_FaceCenter, -1f, HandStandoff, 0f, MinHalfGap), ApproachDuration);
            Advance(Phase.PreGrasp);
        }

        /// <summary>Phase.PreGrasp step: thumb placement, follow the box, confirm or time out.</summary>
        private void StepPreGrasp(float ts)
        {
            if (!m_FaceValid || m_Mujoco == null) return;

            // Thumb placement only after the arms have had time to arrive.
            if (m_PhaseElapsed >= ApproachDuration)
            {
                ServoInset(m_Left, ts);
                ServoInset(m_Right, ts);
            }

            Vector3 tL = HandIkTarget(m_FaceCenter, 1f, HandStandoff, 0f, MinHalfGap);
            Vector3 tR = HandIkTarget(m_FaceCenter, -1f, HandStandoff, 0f, MinHalfGap);
            RetargetIfDrifted(tL, tR, RetargetInterval, RetargetThreshold, RetargetMoveDuration, ts);

            float errL = (m_Left.HandPoint(m_Mujoco, HandReach) - tL).Length();
            float errR = (m_Right.HandPoint(m_Mujoco, HandReach) - tR).Length();
            bool posOk = errL <= PositionTolerance && errR <= PositionTolerance;
            bool thumbsOk = m_PhaseElapsed >= ApproachDuration && ThumbPlaced(m_Left) && ThumbPlaced(m_Right);

            if (posOk && thumbsOk) m_HoldTime += ts; else m_HoldTime = 0f;

            if (m_HoldTime >= HoldTimeToConfirm)
            {
                Log.Info("[G1BoxGrasp] pre-grasp stable: inset L=" + m_Left.Inset.ToString("F3") + " R=" + m_Right.Inset.ToString("F3")
                    + " thumbErr L=" + m_Left.ThumbAlongErr.ToString("F3") + " R=" + m_Right.ThumbAlongErr.ToString("F3"));
                EnterGrasp(false);
                return;
            }

            if (m_PhaseElapsed >= PreGraspTimeout)
                Fail("pre-grasp timeout - posErr L=" + errL.ToString("F3") + " R=" + errR.ToString("F3")
                    + " thumbErr L=" + m_Left.ThumbAlongErr.ToString("F3") + " R=" + m_Right.ThumbAlongErr.ToString("F3")
                    + " inset L=" + m_Left.Inset.ToString("F3") + " R=" + m_Right.Inset.ToString("F3"));
        }
    }

}
