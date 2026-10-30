using System;
using Hazel;

namespace G1Pipe
{
    // Hand targets and poses: grip frame, wrist steering, finger poses, IK targets and glides.
    public partial class G1BoxGrasp
    {
        // Grip frame from the box as it is now: centre, its own horizontal axis closest to near (the heading, then the last forward so it
        // never flips) as forward, squeeze axis to its left, half extents.
        private void SetGripFrame(Vector3 near)
        {
            m_Mujoco!.GetGeomPoseMj(m_Boxes[m_Box].Geom, out _, out Quaternion q);
            Quaternion r = m_MjToHz * q;
            float best = 0f;
            m_Fwd = near;
            foreach (
                Vector3 axis in new[] { r * Vector3.Right, r * Vector3.Up, r * Vector3.Forward }
            )
            {
                float d = Vector3.Dot(Flat(axis), near); // unnormalised, so a near-vertical axis never wins
                if (Math.Abs(d) <= best)
                    continue;
                best = Math.Abs(d);
                m_Fwd = FlatDir(axis) * (d > 0f ? 1f : -1f);
            }
            m_Lat = LeftOf(m_Fwd);
            m_Center = BoxCenter(m_Box);
            m_HalfW = HalfExtent(m_Box, m_Lat);
            m_HalfD = HalfExtent(m_Box, m_Fwd);
            m_HalfH = HalfExtent(m_Box, Vector3.Up);
        }

        // Hand target at its front-bottom box corner: palm PalmOffset + Gap outside the side face, grasp site FrontInset behind the front face
        // (straight thumb across the front face, fingers along the side) + m_Slide (- = in front of the box; not for a holding hand), GripHeight up, + m_Lift; while
        // reaching for the box, never lower than GripHeight over the conveyor surface under it.
        private Vector3 HandTarget(Arm a)
        {
            Vector3 c = m_Carrying ? CarryPoint(m_Center) : m_Center;
            Vector3 lat = m_Carrying ? CarryDir(m_Lat) : m_Lat,
                fwd = m_Carrying ? CarryDir(m_Fwd) : m_Fwd;
            Vector3 t =
                c
                + lat * (a.Side * (m_HalfW + PalmOffset + a.Gap))
                + fwd * (FrontInset - m_HalfD + (a == m_Holder ? 0f : m_Slide))
                + Vector3.Up * (GripHeight(a) - m_HalfH + m_Lift);
            if (a == m_Holder)
                return t + lat * m_HoldOff.X + fwd * m_HoldOff.Y + Vector3.Up * m_HoldOff.Z;
            if (!m_Carrying && m_Phase is Phase.PreGrasp or Phase.Squeeze)
                t.Y = Math.Max(t.Y, SurfaceY(t) + GripHeight(a));
            return t;
        }

        // Grasp-site height over the box bottom that leaves BeltClearance under the palm's lower edge (the left hand 2 x k_ThumbStagger higher),
        // plus any raise from the reach plan.
        private float GripHeight(Arm a) =>
            BeltClearance + PalmBelow(a) + (1f + a.Side) * k_ThumbStagger + m_GripLift;

        // How far the palm's lower edge hangs below the grasp site (hand upright): left 0.034 m, right 0.054 m.
        private static float PalmBelow(Arm a) => k_PalmHalf - a.Side * k_SiteOffset;

        // Front-bottom corner of the box on one side (world, from the grip frame).
        private Vector3 Corner(float side) =>
            m_Center + m_Lat * (side * m_HalfW) - m_Fwd * m_HalfD - Vector3.Up * m_HalfH;

        // Thumbs flared back (ThumbOpenDeg) except while gripping, when they swing to ThumbGripDeg past straight with the tip bent ThumbTipDeg; index + middle follow a.Curl while
        // gripping (Squeeze curls them in) and ease straight otherwise. Everything moves at a set rate - nothing snaps into the box.
        private void PoseHands(float ts)
        {
            bool grip = Gripping();
            foreach (Arm a in m_Arms)
            {
                if (a.Hand.Length < 4)
                    continue;
                float open = Math.Clamp(ThumbOpenDeg, 0f, 41.5f) * Mathf.Deg2Rad,
                    shut = Math.Clamp(ThumbGripDeg, 0f, 60f) * Mathf.Deg2Rad,
                    tip = Math.Clamp(ThumbTipDeg, 0f, 100f) * Mathf.Deg2Rad;
                a.Thumb += Mathf.Clamp(
                    (grip ? -shut : open) - a.Thumb,
                    -k_ThumbRate * ts,
                    k_ThumbRate * ts
                );
                a.Tip += Mathf.Clamp(
                    (grip ? tip : 0f) - a.Tip,
                    -k_ThumbRate * ts,
                    k_ThumbRate * ts
                );
                if (!grip)
                    a.Curl = Math.Max(0f, a.Curl - 2f * k_CurlRate * ts);
                a.Hand[0].Value = -a.Side * a.Thumb;
                a.Hand[1].Value = a.Side * a.Tip;
                a.Hand[2].Value = a.Hand[3].Value = -a.Side * a.Curl;
            }
        }

        // Height of the conveyor's top plane under point p.
        private float SurfaceY(Vector3 p) =>
            m_ChuteTop.Y
            - (m_ChuteN.X * (p.X - m_ChuteTop.X) + m_ChuteN.Z * (p.Z - m_ChuteTop.Z)) / m_ChuteN.Y;

        // Wrist servo: turns each hand toward its zero-pose frame on the grip axes - fingers (X) along forward, Y along the squeeze axis, so the
        // left palm (-Y) and right palm (+Y) face the box; heading axes when not gripping (and for the catch's parked hand). Closed on the measured hand frame, whatever the IK does.
        // A hand still climbing out from low down keeps its wrist still (turning there would swing the fingers into the belt); LockWrists keeps both still always.
        // Once the grip is confirmed (until Release) the box fixes the hands' orientation, so the wrists hold the angles they had then: steering on
        // only winds the command up to the joint limit, pinning the actuator at its torque limit with no stiffness or damping left to stop the box swinging.
        private void OrientWrists(float ts)
        {
            bool grip = m_Phase >= Phase.Ready && m_Phase <= Phase.Release,
                held = m_Phase > Phase.Squeeze && m_Phase < Phase.Release;
            foreach (Arm a in m_Arms)
            {
                if (held && !a.WristHeld && a.Joints.Length == 7)
                    for (int i = 0; i < 3; i++)
                        a.Joints[4 + i].Value = a.WristQ[i] = m_Mujoco!.GetJointPosition(
                            a.WristAdr[i]
                        );
                a.WristHeld = held;
                if (a.Joints.Length < 7 || a.Rising || SiteWorld(a.Site).Y < a.Cmd.Y - k_Snap)
                    continue;
                Vector3 x = grip && !Parked(a) ? (m_Carrying ? BodyDir(m_Fwd) : m_Fwd) : Heading(),
                    y = LeftOf(x),
                    z = Vector3.Cross(x, y);
                Quaternion r = BodyRot(a.WristLink[2]);
                Vector3 cy = r * s_Axes[1],
                    w =
                        (
                            Vector3.Cross(r * s_Axes[0], x)
                            + Vector3.Cross(cy, y)
                            + Vector3.Cross(r * s_Axes[2], z)
                        ) * 0.5f; // rotation error
                a.PalmErr = Mathf.Acos(Mathf.Clamp(Vector3.Dot(cy, y), -1f, 1f)) * Mathf.Rad2Deg;
                if (LockWrists || held)
                    continue;
                for (int i = 0; i < 3; i++)
                {
                    m_Mujoco!.GetActuatorCtrlRange(
                        (uint)a.Joints[4 + i].ActuatorID,
                        out float lo,
                        out float hi
                    );
                    a.WristQ[i] = Mathf.Clamp(
                        a.WristQ[i]
                            + Mathf.Clamp(
                                k_WristGain * Vector3.Dot(BodyRot(a.WristLink[i]) * s_Axes[i], w),
                                -k_WristRate,
                                k_WristRate
                            ) * ts,
                        lo,
                        hi
                    );
                    a.Joints[4 + i].Value = a.WristQ[i];
                }
            }
        }

        // Folded hand target in front of the body.
        private Vector3 Fold(Arm a) => BodyPoint(Mirror(FoldOffset, a.Side));

        // Writes both hand targets and elbow poles. While gripping, or within k_Snap, a hand target is tracked directly. A farther one is reached
        // along a safe path of straight segments, each a min-jerk move (eases in and out) at HandMoveSpeed on average: a short move across goes
        // straight; a longer one first climbs straight up to m_SafeY if low, then crosses above the belt and boxes, then comes straight down.
        // newMove re-plans from the live hands and restarts the graph's blend (Start) from the current pose. No-op until the pelvis is known.
        private void SendHands(Func<Arm, Vector3> target, bool newMove)
        {
            if (m_PelvisId == uint.MaxValue)
                return;
            bool grip = Gripping();
            foreach (Arm a in m_Arms)
            {
                Vector3 goal = target(a);
                if (newMove)
                {
                    a.Cmd = SiteWorld(a.Site);
                    a.T = a.Tau = 0f;
                }
                if (
                    !grip
                    && !newMove
                    && a.Tau >= a.T
                    && (target.Method.Name == nameof(Fold) || Parked(a))
                )
                {
                    // Folded: follow the body's fold point with a k_FoldSmooth lag - drift is followed, bounces never reach the arms (and never
                    // come back through them into the walker), and no path is re-planned.
                    a.Cmd += (goal - a.Cmd) * (1f - (float)Math.Exp(-m_Dt / k_FoldSmooth));
                    a.Direct = true;
                    a.Rising = false;
                    SendTarget(a);
                    continue;
                }
                bool direct = grip || (a.Tau >= a.T && (goal - a.Cmd).Length() <= k_Snap);
                if (m_Phase == Phase.PreGrasp && direct != a.Direct)
                    Log.Info(
                        $"[G1] PG {a.Name} -> {(direct ? $"tracking its goal directly ({(goal - a.Cmd).Length():F3} m away)" : $"path to its goal ({(goal - a.Cmd).Length():F3} m away)")}"
                    );
                a.Direct = direct;
                if (direct)
                {
                    a.Cmd = goal;
                    a.T = a.Tau = 0f;
                    a.Rising = false;
                }
                else
                {
                    if (a.Tau >= a.T)
                        PlanSegment(a, goal);
                    a.Tau = Math.Min(a.Tau + m_Dt, a.T);
                    float u = a.Tau / a.T;
                    a.Cmd = a.From + (a.To - a.From) * (u * u * u * (10f + u * (6f * u - 15f))); // min-jerk position profile
                }
                SendTarget(a);
            }
            if (!newMove)
                return;
            if (m_Phase == Phase.PreGrasp)
                Log.Info(
                    $"[G1] PG IK re-seed (Start, {k_SeedBlend:F1} s blend from the current pose), paths re-planned from the live hands"
                );
            m_Robot!.SetInputFloat(s_Duration, k_SeedBlend);
            m_Robot.SetInputTrigger(s_Start);
        }

        // Sends a hand's IK target and elbow pole. Cap: a target beyond MaxReach of the shoulder is pulled in toward it (logged when the cap
        // starts and stops).
        private void SendTarget(Arm a)
        {
            Vector3 t = a.Cmd,
                d = t - BodyPos(a.Shoulder);
            float max = MaxReach * k_ArmLength;
            if (d.Length() > max != a.Capped)
                Log.Warn(
                    $"[G1] {a.Name} IK target {(d.Length() > max ? $"{d.Length() / k_ArmLength * 100f:F0}% of the arm away -> pulled in to {MaxReach * 100f:F0}%" : "back within reach")} in {m_Phase}"
                );
            a.Capped = d.Length() > max;
            m_Robot!.SetInputVector3(a.Target, a.Capped ? t - d * (1f - max / d.Length()) : t);
            Vector3 pole = Mirror(PoleOffset + new Vector3(0f, 0f, m_PoleOut), a.Side);
            m_Robot.SetInputVector3(a.Pole, m_Carrying ? CarryPoint(pole) : BodyPoint(pole));
        }

        // Next straight segment toward goal. Pure vertical moves go straight. Near a box (NearBox) and below m_SafeY, a long move across or a big climb first rises
        // straight up to m_SafeY (wrist held). A move that ends lower first crosses at the current height, then the next segment comes straight
        // down. Anything else (short local moves, climbs above m_SafeY) goes straight.
        private void PlanSegment(Arm a, Vector3 goal)
        {
            float across = Flat(goal - a.Cmd).Length();
            a.Rising =
                across > 0.02f
                && (NearBox(a.Cmd) || NearBox(goal)) // away from the boxes a move just goes straight
                && a.Cmd.Y < m_SafeY - 0.005f
                && (across > k_LocalMove || goal.Y > a.Cmd.Y + k_LocalMove);
            a.From = a.Cmd;
            a.To =
                across <= 0.02f ? goal
                : a.Rising ? new Vector3(a.Cmd.X, m_SafeY, a.Cmd.Z)
                : goal.Y < a.Cmd.Y - 0.02f ? new Vector3(goal.X, a.Cmd.Y, goal.Z)
                : goal;
            a.T = Math.Max(k_MinSegment, (a.To - a.From).Length() / HandMoveSpeed);
            a.Tau = 0f;
            if (m_Phase == Phase.PreGrasp)
                Log.Info(
                    $"[G1] PG {a.Name} new segment ({(across <= 0.02f ? "straight to the goal (vertical)" : a.Rising ? "straight up to the travel height" : a.To.Y > goal.Y + 0.001f ? "across at this height, down next" : "straight to the goal")}): {F(a.From)} -> {F(a.To)}, {(a.To - a.From).Length():F3} m in {a.T:F2} s; goal {F(goal)} {(goal - a.To).Length():F3} m beyond"
                );
        }

        // Point within k_BoxNear (horizontally) of a box on the conveyor.
        private bool NearBox(Vector3 p)
        {
            for (int i = 0; i < m_Boxes.Count; i++)
                if (Math.Abs(ChuteGap(i)) < 0.05f && Flat(BoxCenter(i) - p).Length() < k_BoxNear)
                    return true;
            return false;
        }

        // A hand is still running a path segment.
        private bool Gliding() => m_Arms[0].Tau < m_Arms[0].T || m_Arms[1].Tau < m_Arms[1].T;

        // Safe travel height for the hands: k_SafeMargin over the conveyor top and the boxes on it.
        private void SetSafeY()
        {
            m_SafeY = m_ChuteTop.Y;
            for (int i = 0; i < m_Boxes.Count; i++)
                if (Math.Abs(ChuteGap(i)) < 0.05f)
                    m_SafeY = Math.Max(m_SafeY, BoxCenter(i).Y + HalfExtent(i, Vector3.Up));
            m_SafeY += k_SafeMargin;
        }
    }
}
