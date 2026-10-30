using System;
using Hazel;

namespace G1Pipe
{
    // Walking and turning through the walker/rotator policies, and the waist lock.
    public partial class G1BoxGrasp
    {
        // LockWaist: while the box is held (grip confirmed until Release) the walker drives the legs only and the waist holds the angles
        // it had then, so the torso can't twist on the pelvis and swing the arms and box around (given back while placing, the walker
        // swung a heavy box loose: yaw +-10 deg, pitch 8-15 deg).
        private void HoldWaist()
        {
            bool lockIt = LockWaist && m_Phase > Phase.Squeeze && m_Phase < Phase.Release;
            if (lockIt != m_WaistLocked)
            {
                m_Robot!.SetDrivenJoints(m_Active, lockIt ? s_LegsOnly : s_LegsAndWaistOnly);
                for (int i = 0; i < 3; i++)
                {
                    m_WaistQ[i] = (float)m_Mujoco!.GetJointPosition(m_WaistAdr[i]);
                    if (lockIt)
                        m_WaistIK[i] = m_Waist[i].IK;
                    m_Waist[i].IK = !lockIt && m_WaistIK[i]; // script-owned while locked: else its ctrl stayed 0 and the waist snapped straight (6 deg pitch in 0.1 s)
                }
                m_WaistLocked = lockIt;
                Log.Info(
                    $"[G1] waist {(lockIt ? $"locked at (yaw, roll, pitch) [{Degs(m_WaistAdr)}] deg in {m_Phase}" : $"back to the walker in {m_Phase}")}"
                );
            }
            if (m_WaistLocked)
                for (int i = 0; i < 3; i++)
                    m_Waist[i].Value = m_WaistQ[i];
        }

        // Closed-loop walk to a point: steer at it while far (if allowed); otherwise step straight onto it (forward + sideways, no turning,
        // slowing near it) so the robot never circles it. True within tol.
        private bool WalkTo(Vector3 goal, float tol, bool steer)
        {
            Vector3 to = Flat(goal - Pelvis()),
                fwd = Heading();
            float err = Mathf.WrapToPi(YawOf(to) - YawOf(fwd));
            m_ToGo = to.Length();
            if (m_ToGo <= tol)
                return true;
            if (steer && m_ToGo > k_NearRadius)
                Drive(
                    Math.Abs(err) < 0.6f ? WalkSpeed : 0f,
                    0f,
                    Mathf.Clamp(err * k_YawGain, -k_MaxYawRate, k_MaxYawRate)
                );
            else
            {
                float v = Mathf.Clamp(m_ToGo * k_StepGain, k_MinStepSpeed, WalkSpeed) / m_ToGo;
                Drive(v * Vector3.Dot(to, fwd), v * Vector3.Dot(to, LeftOf(fwd)), 0f);
            }
            return false;
        }

        // Walk made no progress: turn toward the goal again and resume (MaxWalkRetries), else fail the attempt.
        private void WalkStalled(Phase turn)
        {
            if (++m_WalkRetries > MaxWalkRetries)
            {
                Fail($"walk stalled {m_ToGo:F2} m from the goal after {MaxWalkRetries} re-turns");
                return;
            }
            Log.Warn(
                $"[G1] walk stalled {m_ToGo:F2} m from the goal -> turning toward it again ({m_WalkRetries}/{MaxWalkRetries})"
            );
            Advance(turn);
        }

        // Turn in place on the walker with the box (the rotator's take-over lurches the body): settle and wait for the box to stop swinging; if more than tol off, yaw at CarryTurnCmd
        // (eased in over CarryTurnRamp, eased down to k_TurnFloor of it over the last k_TurnEase) until k_TurnStop short (it coasts on), ramp the command out over
        // k_CarryRamp, coast to a stop and check again (MaxTurnTries). True when facing yaw, or out of
        // tries (it goes on from there: the box is never let go over a turn).
        private bool WalkerTurn(float yaw, float tol)
        {
            if (m_Step == 0)
                m_TurnYaw = yaw; // aim fixed once the turn starts, so body sway doesn't move it
            float err = Mathf.WrapToPi(m_TurnYaw - YawOf(Heading()));
            switch (m_Step)
            {
                case 0:
                    if (m_StepTime < k_Settle || m_Restoring || !BoxSteady())
                        return false;
                    if (Math.Abs(err) < tol)
                        return true;
                    if (++m_TurnTries > MaxTurnTries)
                    {
                        Log.Warn(
                            $"[G1] turn still {err * Mathf.Rad2Deg:F0} deg off after {MaxTurnTries} tries -> going on from here"
                        );
                        return true;
                    }
                    Log.Info(
                        $"[G1] turning {err * Mathf.Rad2Deg:F0} deg in place on the walker ({m_TurnTries}/{MaxTurnTries})"
                    );
                    break;
                case 1:
                    float ramp = Math.Min(1f, m_StepTime / CarryTurnRamp);
                    m_TurnCmd =
                        Math.Sign(err)
                        * CarryTurnCmd
                        * Math.Min(
                            ramp * ramp * (3f - 2f * ramp),
                            Mathf.Clamp(Math.Abs(err) / k_TurnEase, k_TurnFloor, 1f)
                        );
                    Drive(0f, 0f, m_TurnCmd);
                    if (
                        Math.Abs(err) > k_TurnStop
                        && !Stalled(Math.Abs(err), k_TurnProgress, PhaseTimeout)
                    )
                        return false;
                    break;
                default:
                    Drive(0f, 0f, m_TurnCmd * Math.Max(0f, 1f - m_StepTime / k_CarryRamp));
                    if (m_StepTime < k_ApCoast)
                        return false;
                    m_Step = 0; // coasted to a stop: check again
                    m_StepTime = 0f;
                    ResetWatch();
                    return false;
            }
            NextStep();
            return false;
        }

        // Turn in place with the rotator to the heading given when the turn starts: settle, swap in, ramped P-turn (at least MinTurnRate,
        // doubled on a retry after a stall), stop, swap back, settle (G9). True when done; a turn still stalled after MaxTurnTries fails.
        private bool Turn(float yaw)
        {
            if (m_Step == 0)
                m_TurnYaw = yaw; // aim fixed once the turn starts, so body sway doesn't move it
            float err = Mathf.WrapToPi(m_TurnYaw - YawOf(Heading()));
            switch (m_Step)
            {
                case 0: // settle on the walker; no turn needed if already facing
                    if (m_StepTime < k_Settle)
                        return false;
                    if (Math.Abs(err) < TurnTolerance)
                        return true;
                    SetLocomotion(RotatorSlotId);
                    m_TurnTries = 1;
                    break;
                case 1: // turn, yaw rate ramped in from standstill
                    Drive(
                        0f,
                        0f,
                        Math.Sign(err)
                            * Mathf.Clamp(
                                Math.Abs(err) * k_YawGain,
                                Math.Min(MinTurnRate * m_TurnTries, k_TurnRate),
                                k_TurnRate
                            )
                            * Math.Min(1f, m_StepTime / k_TurnRampIn)
                    );
                    if (Math.Abs(err) <= TurnTolerance)
                        break;
                    if (!Stalled(Math.Abs(err), k_TurnProgress, PhaseTimeout))
                        return false;
                    if (m_TurnTries >= MaxTurnTries)
                    {
                        Fail(
                            $"turn stalled {err * Mathf.Rad2Deg:F0} deg short after {MaxTurnTries} tries"
                        );
                        return false;
                    }
                    Log.Warn(
                        $"[G1] turn stalled {err * Mathf.Rad2Deg:F0} deg short -> turning again harder ({++m_TurnTries}/{MaxTurnTries})"
                    );
                    m_StepTime = 0f;
                    ResetWatch();
                    return false;
                case 2: // zero yaw while the rotator is still active, then hand back to the walker
                    if (m_StepTime < k_DecelTime)
                        return false;
                    SetLocomotion(WalkerSlotId);
                    break;
                default: // settle on the walker
                    return m_StepTime >= k_Settle;
            }
            NextStep();
            return false;
        }

        // Hands the legs + waist to the walker or the rotator, switching the other off; the arms stay with the IK (G8).
        private void SetLocomotion(uint slot)
        {
            m_Robot!.SetDrivenJoints(slot, m_WaistLocked ? s_LegsOnly : s_LegsAndWaistOnly);
            m_Robot.SetPolicyActive(slot == WalkerSlotId ? RotatorSlotId : WalkerSlotId, false);
            m_Robot.SetPolicyActive(slot, true);
            m_Active = slot;
        }

        // Sends (vx, vy, yaw rate) to the active locomotion policy.
        private void Drive(float vx, float vy, float yawRate)
        {
            m_Robot!.SetFloat(m_Active, k_SetVx, vx);
            m_Robot.SetFloat(m_Active, k_SetVy, vy);
            m_Robot.SetFloat(m_Active, k_SetYawRate, yawRate);
            m_Cmd = new Vector3(vx, vy, yawRate);
        }
    }
}
