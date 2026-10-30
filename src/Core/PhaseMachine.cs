using Hazel;

namespace G1Pipe
{
    // Phase changes, sub-steps, the progress watchdog, Fail and Recover.
    public partial class G1BoxGrasp
    {
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
    }
}
