using System;
using Hazel;

namespace G1Pipe
{
    // DragIn and Lift: the box is pulled toward the body and raised.
    public partial class G1BoxGrasp
    {
        // How far ahead of the pelvis the grasp sites' grip spot is (m).
        private float GripAhead() =>
            Vector3.Dot(Flat(m_Center - Pelvis()), Heading()) - m_HalfD + FrontInset;

        // Drag in: still squeezing, the grip slides toward the body at k_DragSpeed (the box sliding on the belt) by the distance that
        // put it k_DragTo ahead when the drag began, then Lift (a fixed distance: the body stepping back must not keep the pull going).
        // A box trailing the hands by more than k_DragLag is lifted from where it is.
        private void StepDragIn(float ts)
        {
            if (m_Step == 0)
            {
                m_DragLeft = GripAhead() - k_DragTo; // measured once: a body stepping back while dragging must not keep the pull going
                NextStep();
            }
            if (!Hold(ts))
                return;
            float lag = Vector3.Dot(Flat(BoxCenter(m_Box) - m_Center), m_Fwd);
            if (lag > k_DragLag)
            {
                Log.Warn(
                    $"[G1] box trails the hands by {lag:F3} m while dragging -> lifting from here"
                );
                m_Center = BoxCenter(m_Box);
            }
            else if (m_DragLeft > 0f)
            {
                float d = Math.Min(k_DragSpeed * ts, m_DragLeft);
                m_Center -= m_Fwd * d;
                m_DragLeft -= d;
                return;
            }
            Log.Info($"[G1] dragged in: grip {GripAhead():F2} m ahead | {ArmAngles()}");
            m_Slot = m_Center - Vector3.Up * m_HalfH;
            Advance(Phase.Lift);
        }

        // Lift in pelvis coords from the start, so the box follows the body (a body pushed back by the box's weight never stretches the
        // arms after it): up k_ClearLift, then up the rest of LiftHeight and in toward CarryOffset together, so a heavy box comes close to
        // the chest early. No station hold while picking up: its correction steps rocked the body to and fro and flung a heavy box. Next
        // carry it, or put it back down where it was picked.
        private void StepLift(float ts)
        {
            if (m_Step == 0)
            {
                SetAnchor();
                m_Center = ToBody(m_Center - Pelvis());
                m_Lat = ToBody(m_Lat);
                m_Fwd = ToBody(m_Fwd);
                m_Carrying = true;
                m_LiftTo = new Vector3(
                    Math.Min(m_Center.X, CarryX()),
                    m_Center.Y + LiftHeight,
                    m_Center.Z
                );
                NextStep();
            }
            if (!Hold(ts))
                return;
            float up = Math.Min(m_Center.Y + k_LiftSpeed * ts, m_LiftTo.Y),
                fwd = up >= m_LiftTo.Y - LiftHeight + k_ClearLift
                    ? Math.Max(m_Center.X - k_CarrySpeed * ts, m_LiftTo.X)
                    : m_Center.X;
            m_Center = new Vector3(fwd, up, m_Center.Z);
            if (up < m_LiftTo.Y || fwd > m_LiftTo.X)
                return;
            Advance(
                CarryToTable ? Phase.PullIn
                : PlaceBox ? Phase.Place
                : Phase.Holding
            );
        }

        // How far ahead of the pelvis the box is carried: CarryOffset, or - if its top can reach head height anywhere in the carry
        // (raised over the table included) - far enough that its near face stays HeadClearance in front of the head.
        private float CarryX() =>
            CarryOffset.Y + k_MaxRaise + m_HalfH > k_HeadBottom
                ? Math.Max(CarryOffset.X, k_HeadFront + HeadClearance + m_HalfD)
                : CarryOffset.X;
    }
}
