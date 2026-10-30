using System;
using Hazel;

namespace G1Pipe
{
    // Place, Release and Retract, and where on the table the box goes.
    public partial class G1BoxGrasp
    {
        // Settle and switch the grip to world coords; a box not yet over the table comes in over the table spot first; then it goes
        // straight down where it is until the table holds it (or the hands can't go lower) and is let go there.
        private void StepPlace(float ts)
        {
            if (m_PhaseTime <= 1.5f * ts)
                SetAnchor();
            if (!Hold(ts))
                return;
            HoldStation();
            Vector3 c = BoxCenter(m_Box);
            if (m_Step == 0)
            {
                if (m_StepTime < k_Settle)
                    return;
                m_Center = CarryPoint(m_Center);
                m_Lat = CarryDir(m_Lat);
                m_Fwd = CarryDir(m_Fwd);
                m_Carrying = false;
                NextStep();
            }
            else if (m_Step == 1)
            {
                if (
                    OverTable(c, k_OverTableInset)
                    || MoveTowards(
                        ref m_Center,
                        new Vector3(m_Slot.X, m_Center.Y, m_Slot.Z),
                        k_CarrySpeed * ts
                    )
                )
                    NextStep();
            }
            else
            {
                m_Center = m_Center - Vector3.Up * (k_PlaceLowerSpeed * ts);
                float above = c.Y - m_HalfH - TopOf(PlaceTarget!).Y; // box bottom over the table top
                if (above > 0.01f && m_Center.Y > c.Y - 0.02f)
                    return; // still coming down
                Log.Info($"[G1] releasing {above:F3} m over the table");
                Advance(Phase.Release);
            }
        }

        // Open both palms away from the box, then start lifting the hands clear.
        private void StepRelease(float ts)
        {
            foreach (Arm a in m_Arms)
                a.Gap = Math.Min(a.Gap + k_OpenSpeed * ts, k_ReleaseGap);
            SendHands(HandTarget, false);
            if (m_PhaseTime < k_ReleaseTime)
                return;
            m_Lift = k_RetractUp;
            SendHands(HandTarget, true);
            Advance(Phase.Retract);
        }

        // Hands up first, then back to the fold pose - never forward over the placed box (G21) - then look for the next box.
        private void StepRetract()
        {
            if (m_Step == 0)
            {
                SendHands(HandTarget, false);
                if (Gliding())
                    return;
                SendHands(Fold, true);
                NextStep();
            }
            else
            {
                SendHands(Fold, false);
                if (Gliding())
                    return;
                Log.Info(
                    $"[G1] placed {Flat(BoxCenter(m_Box) - m_Slot).Length():F3} m from its spot"
                );
                Advance(Repeat ? Phase.FindBox : Phase.Done);
            }
        }

        // The table spot nearest from: on the line PlaceEdgeInset in from the table top's near edge, PlaceEdgeInset in from its ends (where
        // the robot walks to; the box goes down wherever it is over the table, no free-slot search for now).
        private Vector3 TableSpot(Vector3 from)
        {
            TransformComponent t = PlaceTarget!.Transform;
            Vector3 face = FaceOf(PlaceTarget),
                along = LeftOf(face);
            float halfLen = 0.5f * Math.Max(t.WorldScale.X, t.WorldScale.Z) - PlaceEdgeInset,
                halfWidth = 0.5f * Math.Min(t.WorldScale.X, t.WorldScale.Z);
            Vector3 line = TopOf(PlaceTarget) - face * (halfWidth - PlaceEdgeInset);
            return line
                + along * Mathf.Clamp(Vector3.Dot(Flat(from - line), along), -halfLen, halfLen);
        }

        // Whether point p is over the table top, at least inset in from its edges.
        private bool OverTable(Vector3 p, float inset)
        {
            TransformComponent t = PlaceTarget!.Transform;
            Vector3 face = FaceOf(PlaceTarget),
                d = Flat(p - TopOf(PlaceTarget));
            return Math.Abs(Vector3.Dot(d, LeftOf(face)))
                    <= 0.5f * Math.Max(t.WorldScale.X, t.WorldScale.Z) - inset
                && Math.Abs(Vector3.Dot(d, face))
                    <= 0.5f * Math.Min(t.WorldScale.X, t.WorldScale.Z) - inset;
        }
    }
}
