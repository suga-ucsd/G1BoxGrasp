using System;
using Hazel;

namespace G1Pipe
{
    // Carry: back off, turn, walk to the table, station hold and the carry frame.
    public partial class G1BoxGrasp
    {
        // The pelvis spot to hold from now on (standing with the box).
        private void SetAnchor()
        {
            m_Anchor = Pelvis();
            m_Restoring = false;
        }

        // Standing with the box: once its weight has walked the body more than k_DriftTol off m_Anchor, and the body has been still for
        // StillTime, the walker steps it back at the slowest command it steps at, along one direction only - forward / back first, sideways
        // once that is within k_DriftDone - until that direction is within k_DriftDone (an overshoot ends it; the next waits for stillness).
        // Never during a turn: a held box made combined commands stumble.
        private void HoldStation()
        {
            Vector3 d = Flat(m_Anchor - Pelvis());
            float fwd = Vector3.Dot(d, Heading()),
                side = Vector3.Dot(d, LeftOf(Heading()));
            bool turning = m_Cmd.Z != 0f;
            if (!m_Restoring && !turning && d.Length() > k_DriftTol && m_Still >= StillTime)
            {
                m_RestoreDir =
                    Math.Abs(fwd) > k_DriftDone
                        ? new Vector2(Math.Sign(fwd), 0f)
                        : new Vector2(0f, Math.Sign(side)); // one axis, one way
                m_Restoring = true;
                Log.Info(
                    $"[G1] the box walked the body {d.Length():F3} m off its spot in {m_Phase} ({-fwd:F3} fwd, {-side:F3} left) -> stepping {(m_RestoreDir.X > 0f ? "forward" : m_RestoreDir.X < 0f ? "back" : m_RestoreDir.Y > 0f ? "left" : "right")} onto it"
                );
            }
            if (!m_Restoring)
                return;
            float left = m_RestoreDir.X != 0f ? m_RestoreDir.X * fwd : m_RestoreDir.Y * side; // still to go along the chosen direction
            if (turning || left < k_DriftDone)
            {
                m_Restoring = false;
                Log.Info(
                    $"[G1] {(turning ? "correction stopped for the turn" : "correction done")} in {m_Phase} ({d.Length():F3} m off its spot)"
                );
                return;
            }
            Drive(
                m_RestoreDir.X > 0f ? k_FwdCmd
                    : m_RestoreDir.X < 0f ? -k_BackCmd
                    : 0f,
                m_RestoreDir.Y * k_SideCmd,
                0f
            );
        }

        // Picks the nearest free table slot and where to stand for it, then backs off the conveyor, turns to face the table, walks onto that spot and squares up.
        private void EnterTurnToTable()
        {
            m_Slot = TableSpot(m_Anchor - Heading() * CarryBackDistance); // nearest to where it turns, after backing off
            m_Reach = PlaceStandDistance;
            m_WalkRetries = 0;
            m_Stand = StandFor(PlaceTarget!, m_Slot, m_Reach, FaceOf(PlaceTarget!));
            Log.Info(
                $"[G1] placing at {m_Slot}, standing at {m_Stand}, {Vector3.Dot(m_Slot - m_Stand, FaceOf(PlaceTarget!)):F2} m from the slot"
            );
            Advance(Phase.BackOff);
        }

        // Carrying, first move: settle, back straight away from the conveyor (ramped in) until its coast short of CarryBackDistance behind the start (at least half way) (the sway
        // after stopping goes either way, ~4 cm), then coast to a stop. True when stopped.
        private bool StepBackOff()
        {
            switch (m_Step)
            {
                case 0:
                    if (m_StepTime < k_Settle)
                        return false;
                    m_BackFrom = m_Anchor; // from where it gripped: any drift counts toward the back-off
                    break;
                case 1:
                    Drive(-CarryBackSpeed * Math.Min(1f, m_StepTime / k_CarryRamp), 0f, 0f);
                    if (
                        Vector3.Dot(Flat(m_BackFrom - Pelvis()), Heading())
                            < Math.Max(
                                CarryBackDistance - CarryBackSpeed * k_BackCoast,
                                0.5f * CarryBackDistance
                            )
                        && m_StepTime < k_TurnMax
                    )
                        return false;
                    m_Anchor = m_BackFrom - Heading() * CarryBackDistance; // from here on the body holds the back-off spot
                    m_Restoring = false;
                    break;
                default:
                    if (m_StepTime < k_ApCoast)
                        return false;
                    Log.Info(
                        $"[G1] backed off {Vector3.Dot(Flat(m_BackFrom - Pelvis()), Heading()):F2} m from the conveyor (aim {CarryBackDistance:F2})"
                    );
                    return true;
            }
            NextStep();
            return false;
        }

        // Carrying to the table: lift the box (k_CarrySpeed, at most k_MaxRaise over CarryOffset) while its measured bottom is under the
        // table top + k_TableClear, so walking in never knocks it on the table's edge.
        private void RaiseOverTable(float ts)
        {
            if (
                BoxCenter(m_Box).Y - m_HalfH < m_Slot.Y + k_TableClear
                && m_Center.Y < CarryOffset.Y + k_MaxRaise
            )
                m_Center.Y += k_CarrySpeed * ts;
        }

        // Walker at zero (the caller's) until the box stops swinging in the hands - spin under k_SteadySpin - or k_SteadyMax has passed.
        private bool BoxSteady()
        {
            m_Mujoco!.GetBodyPoseMj(m_Boxes[m_Box].Body, out _, out _, out _, out Vector3 w);
            if (w.Length() >= k_SteadySpin && m_StepTime < k_SteadyMax)
                return false;
            Log.Info(
                $"[G1] box steady in the hands after {m_StepTime:F2} s in {m_Phase} (spin {w.Length():F2} rad/s)"
            );
            return true;
        }

        // Carrying: wait for the box to stop swinging, then walk straight at the stand spot - forward at CarryWalkSpeed (ramped in), steering onto
        // it - until k_WalkCoast short of it (at k_FwdCmd; with the square of the speed when faster: the body coasts on), then coast to a stop. True when stopped.
        private bool CarryWalk()
        {
            Vector3 to = Flat(m_Stand - Pelvis());
            m_ToGo = to.Length();
            if (m_Step == 0)
            {
                if (!m_Restoring && BoxSteady())
                    NextStep();
                return false;
            }
            if (m_Step == 1)
            {
                Drive(
                    CarryWalkSpeed * Math.Min(1f, m_StepTime / k_CarryRamp),
                    0f,
                    Mathf.Clamp(
                        Mathf.WrapToPi(YawOf(to) - YawOf(Heading())) * k_YawGain,
                        -k_MaxYawRate,
                        k_MaxYawRate
                    )
                );
                if (
                    Vector3.Dot(to, Heading())
                        > k_WalkCoast * (CarryWalkSpeed / k_FwdCmd) * (CarryWalkSpeed / k_FwdCmd)
                    && m_StepTime < WalkTimeout
                )
                    return false;
                m_Anchor = m_Stand; // from here on (coast, FaceTable) the body holds the stand spot
                m_Restoring = false;
                NextStep();
                return false;
            }
            if (m_StepTime < k_ApCoast)
                return false;
            Log.Info($"[G1] walked to the table: {m_ToGo:F2} m off the stand spot");
            return true;
        }

        // The same in the carry frame (TrackCarryFrame); equal to the pelvis frame when not carrying.
        private Vector3 CarryDir(Vector3 b)
        {
            Vector3 h = new Vector3((float)Math.Cos(m_CarryYaw), 0f, -(float)Math.Sin(m_CarryYaw));
            return h * b.X + Vector3.Up * b.Y + LeftOf(h) * b.Z;
        }

        private Vector3 CarryPoint(Vector3 b) => m_CarryPos + CarryDir(b);

        // Carry frame: the pelvis position and heading low-passed over CarrySmoothing with their own smoothed velocity fed forward, so a steady
        // walk or turn isn't lagged while gait bounce and sway never reach the arms (the box swinging on them would push back into the walker).
        private void TrackCarryFrame(float ts)
        {
            Vector3 p = Pelvis();
            float yaw = YawOf(Heading());
            if (!m_Carrying || CarrySmoothing <= 0f)
            {
                m_CarryPos = p;
                m_CarryYaw = yaw;
                m_CarryVel = Vector3.Zero;
                m_CarryRate = 0f;
            }
            else
            {
                float k = 1f - (float)Math.Exp(-ts / CarrySmoothing);
                m_CarryVel += ((p - m_LastPelvis) * (1f / ts) - m_CarryVel) * k;
                m_CarryRate += (Mathf.WrapToPi(yaw - m_LastYaw) / ts - m_CarryRate) * k;
                m_CarryPos += (p + m_CarryVel * CarrySmoothing - m_CarryPos) * k;
                m_CarryYaw = Mathf.WrapToPi(
                    m_CarryYaw + Mathf.WrapToPi(yaw + m_CarryRate * CarrySmoothing - m_CarryYaw) * k
                );
            }
            m_LastPelvis = p;
            m_LastYaw = yaw;
        }
    }
}
