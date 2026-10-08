using Hazel;

namespace G1Sort
{
    /// <summary>
    /// Phase switching and failure handling.
    ///
    ///  - Advance(): the ONLY place m_Phase changes.
    ///  - Fail():    logs once per phase; with EnableFailureHandling enters Recover.
    ///  - EnterRecover / FinishRecover: fold arms, reset per-grasp state, retry or give up.
    /// </summary>
    public partial class G1BoxGrasp
    {
        /// <summary>
        /// Reports a failure. Returns true if it switched to Recover (caller should stop
        /// its step). With EnableFailureHandling off it only logs and returns false.
        /// </summary>
        private bool Fail(string reason)
        {
            m_LastFailure = reason;
            if (!m_LoggedThisPhase)
            {
                m_LoggedThisPhase = true;
                Log.Error("[G1BoxGrasp] " + reason + " (phase " + m_Phase + ", retry " + m_Retries + ")");
            }
            if (!EnableFailureHandling)
                return false;
            EnterRecover(reason, true);
            return true;
        }

        /// <summary>
        /// Folds the arms (elbow IK off) and resets gaps, insets, curls, thumb state, tare,
        /// latches and force boost. <paramref name="countRetry"/> = count it against MaxRetries.
        /// </summary>
        private void EnterRecover(string reason, bool countRetry)
        {
            m_RecoverCounts = countRetry;
            for (int i = 0; i < 2; i++)
            {
                ArmChain a = Arm(i);
                a.ElbowRecoverFrom = a.ElbowDeg;
                a.SetElbowIK(false);
                a.SetCurlAll(0f);
                a.ResetThumbState();
                a.Gap = HandStandoff;
                a.Inset = FrontInset;
                a.Contacted = false;
            }
            m_Tared = false;
            m_FrameLatched = false;
            m_Moving = false;
            m_ForceBoost = 1f;
            m_NeedScoop = false;
            BuildFoldPlan();
            CommandFold(PreBendDuration);
            Advance(Phase.Recover);
        }

        /// <summary>After RecoverDuration: counts the retry, then WalkTo again or Failed after MaxRetries.</summary>
        private void FinishRecover()
        {
            if (m_RecoverCounts)
            {
                m_Retries++;
                if (m_Retries > MaxRetries)
                {
                    Log.Error("[G1BoxGrasp] out of retries: " + m_LastFailure);
                    Advance(Phase.Failed);
                    return;
                }
            }
            m_ApproachLatched = false;
            m_PreGraspAchieved = false;
            Advance(Phase.WalkTo);
        }

        /// <summary>Switches phase: resets phase timer, retarget timer and the once-per-phase log flag.</summary>
        private void Advance(Phase next)
        {
            m_Phase = next;
            m_PhaseElapsed = 0f;
            m_RetargetTimer = 0f;
            m_LoggedThisPhase = false;
            if (next == Phase.WalkTo) m_InFinalApproach = false;
            Log.Info("[G1BoxGrasp] -> " + next);
        }
    }

}
