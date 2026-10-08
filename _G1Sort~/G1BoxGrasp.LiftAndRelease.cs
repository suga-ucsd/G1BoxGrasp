using System;
using Hazel;

namespace G1Sort
{
    /// <summary>
    /// Lift, Holding, SetDown and Release phases.
    ///
    /// Lift is done in an object frame: hand targets come from the latched m_LiftCenter
    /// plus m_LiftH while the grip loops keep running. Slip is detected as
    ///   slip = (palm rise) - (box rise)
    /// Slipping pauses the lift and boosts the force target; slipping at max squeeze or a
    /// drop sets the box down and ends in ScoopRequired.
    /// </summary>
    public partial class G1BoxGrasp
    {
        // ==================================================================
        // Lift / Holding
        // ==================================================================

        /// <summary>Latches lift centre and starting heights of box and palms, enters Lift.</summary>
        private void EnterLift()
        {
            if (m_Mujoco == null) return;
            m_LiftCenter = m_GraspCenter;
            m_BoxY0 = BoxPos().Y;
            m_PalmY0 = AvgPalmY();
            m_LiftH = 0f;
            m_SlipTimer = 0f;
            m_LiftSettle = 0f;
            Log.Info("[G1BoxGrasp] grip confirmed: " + HandSummary(m_Left) + " | " + HandSummary(m_Right) + " -> lifting");
            Advance(Phase.Lift);
        }

        /// <summary>Average palm (wrist_yaw_link) height of both hands.</summary>
        private float AvgPalmY()
        {
            if (m_Mujoco == null) return 0f;
            return (m_Left.PalmPosition(m_Mujoco).Y + m_Right.PalmPosition(m_Mujoco).Y) * 0.5f;
        }

        /// <summary>Palm rise minus box rise since EnterLift (+ = box slipping down).</summary>
        private float MeasureSlip() => (AvgPalmY() - m_PalmY0) - (BoxPos().Y - m_BoxY0);

        /// <summary>Box lost, slipped more than DropTolerance, or drifted sideways more than DropTolerance.</summary>
        private bool BoxDropped(float slip) =>
            !HasBox || slip > DropTolerance || Geo.Flat(BoxPos() - m_LiftCenter).Length() > DropTolerance;

        /// <summary>Both hands at (98% of) maximum squeeze depth.</summary>
        private bool BothAtSqueezeCap() =>
            m_Left.Gap <= -MaxSqueezeDepth * 0.98f && m_Right.Gap <= -MaxSqueezeDepth * 0.98f;

        /// <summary>
        /// Shared by Lift and Holding: grip loops, slip and drop checks, force boost, lift targets.
        /// Returns true while slipping; <paramref name="abort"/> = the phase changed (SetDown).
        /// </summary>
        private bool StepHoldGrip(float ts, out bool abort)
        {
            abort = false;
            UpdateHandGrip(m_Left, ts);
            UpdateHandGrip(m_Right, ts);

            float slip = MeasureSlip();
            m_LastSlip = slip;
            if (BoxDropped(slip))
            {
                EnterSetDown("box dropped (slip=" + slip.ToString("F3") + ")", true);
                abort = true;
                return true;
            }

            bool slipping = slip > SlipTolerance;
            if (slipping)
            {
                m_ForceBoost = Math.Min(m_ForceBoost + SlipBoostRate * ts, MaxForceBoost);
                m_SlipTimer += ts;
                if (BothAtSqueezeCap() && m_SlipTimer >= SlipGiveUpTime)
                {
                    EnterSetDown("slipping at max squeeze - friction insufficient", true);
                    abort = true;
                    return true;
                }
            }
            else
            {
                m_SlipTimer = 0f;
            }

            RetargetGrasp(m_LiftCenter, m_LiftH, ts);
            return slipping;
        }

        /// <summary>Phase.Lift step: ramp LiftH to LiftHeight (paused while slipping), settle -> Holding.</summary>
        private void StepLift(float ts)
        {
            bool abort;
            bool slipping = StepHoldGrip(ts, out abort);
            if (abort) return;

            if (!slipping)
                m_LiftH = Math.Min(m_LiftH + LiftSpeed * ts, LiftHeight);

            if (m_LiftH >= LiftHeight && !slipping)
            {
                m_LiftSettle += ts;
                if (m_LiftSettle >= LiftSettleTime)
                {
                    Log.Info("[G1BoxGrasp] lifted " + LiftHeight.ToString("F3") + " m - holding");
                    Advance(Phase.Holding);
                }
            }
            else
            {
                m_LiftSettle = 0f;
            }
        }

        /// <summary>Phase.Holding step: keep gripping and watching for slip.</summary>
        private void StepHolding(float ts)
        {
            bool abort;
            StepHoldGrip(ts, out abort);
        }

        // ==================================================================
        // SetDown / Release
        // ==================================================================

        /// <summary>Records why the box is being set down and whether a scoop grasp is needed, enters SetDown.</summary>
        private void EnterSetDown(string reason, bool needScoop)
        {
            m_SetDownReason = reason;
            m_NeedScoop = needScoop;
            m_LastFailure = reason;
            Log.Error("[G1BoxGrasp] set down: " + reason);
            Advance(Phase.SetDown);
        }

        /// <summary>Phase.SetDown step: lower LiftH to 0 while still gripping, then Release.</summary>
        private void StepSetDown(float ts)
        {
            UpdateHandGrip(m_Left, ts);
            UpdateHandGrip(m_Right, ts);
            m_LiftH = Math.Max(m_LiftH - LiftSpeed * ts, 0f);
            RetargetGrasp(m_LiftCenter, m_LiftH, ts);
            if (m_LiftH <= 0f)
                Advance(Phase.Release);
        }

        /// <summary>
        /// Phase.Release step: open fingers (2x FingerBackoffRate), then back the palms out
        /// to HandStandoff. After ReleaseDuration: ScoopRequired (slip/drop) or Recover.
        /// </summary>
        private void StepRelease(float ts)
        {
            bool fingersOpen = true;
            for (int i = 0; i < 2; i++)
            {
                ArmChain a = Arm(i);
                for (int f = 0; f < a.Fingers.Length; f++)
                    a.Fingers[f].Curl = Math.Max(a.Fingers[f].Curl - FingerBackoffRate * 2f * ts, 0f);
                fingersOpen &= a.FingersOpen();
            }
            if (fingersOpen)
                for (int i = 0; i < 2; i++)
                    Arm(i).Gap = Math.Min(Arm(i).Gap + MaxSqueezeRate * ts, HandStandoff);
            RetargetGrasp(m_LiftCenter, 0f, ts);

            bool opened = fingersOpen && m_Left.Gap >= HandStandoff - 0.0005f && m_Right.Gap >= HandStandoff - 0.0005f;
            if (opened && m_PhaseElapsed >= ReleaseDuration)
            {
                if (m_NeedScoop)
                {
                    Log.Error("[G1BoxGrasp] " + m_SetDownReason + " - scoop grasp required (not implemented).");
                    Advance(Phase.ScoopRequired);
                }
                else
                {
                    EnterRecover(m_SetDownReason, true);
                }
            }
        }
    }

}
