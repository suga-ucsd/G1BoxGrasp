using System;
using Hazel;

namespace G1Pipe
{
    // Boxes and the conveyor: finding boxes, which one to take, box pose and speed.
    public partial class G1BoxGrasp
    {
        // Nearest box that rests on the conveyor, can be reached from outside its edge, and hasn't failed MaxBoxAttempts times; -1 if none.
        private int RestingBox()
        {
            int best = -1;
            float bestDist = float.MaxValue;
            for (int i = 0; i < m_Boxes.Count; i++)
            {
                float speed = BoxSpeed(i),
                    dist = Flat(BoxCenter(i) - Pelvis()).Length();
                if (speed > 0.2f)
                    m_Fails[i] = 0; // a box on the move (dropped, respawned) gets fresh attempts
                if (
                    Math.Abs(ChuteGap(i)) < 0.02f
                    && speed < k_RestSpeed
                    && (
                        !WalkToBox || ReachShort(ChuteEntity!, BoxCenter(i), StandDistance) <= 0.05f
                    )
                    && m_Fails[i] < MaxBoxAttempts
                    && dist < bestDist
                )
                {
                    best = i;
                    bestDist = dist;
                }
            }
            return best;
        }

        // Nearest box over the conveyor still uphill of the pelvis (and not failed MaxBoxAttempts times): the next to catch; -1 if none.
        private int IncomingBox()
        {
            int best = -1;
            float bestAt = float.MinValue;
            for (int i = 0; i < m_Boxes.Count; i++)
            {
                float at = Vector3.Dot(BoxCenter(i) - Pelvis(), m_Downhill); // - = uphill of the pelvis
                if (
                    OverChute(BoxCenter(i))
                    && at < 0f
                    && at > bestAt
                    && m_Fails[i] < MaxBoxAttempts
                )
                {
                    best = i;
                    bestAt = at;
                }
            }
            return best;
        }

        // Fastest a box can speed up sliding down the belt (frictionless): g sin(tilt).
        private float SlideAccel() => 9.81f * Math.Abs(m_Downhill.Y);

        // Point p lies over the conveyor's top face.
        private bool OverChute(Vector3 p)
        {
            TransformComponent t = ChuteEntity!.Transform;
            Vector3 d = t.WorldRotationQuat.Conjugate * (p - t.WorldTranslation);
            return d.Y > 0f
                && Math.Abs(d.X) < 0.5f * t.WorldScale.X
                && Math.Abs(d.Z) < 0.5f * t.WorldScale.Z;
        }

        // Free-moving boxes: scene bodies (named lucky_body_<entity id>) with a box geom and a free joint.
        private void FindBoxes()
        {
            m_Boxes.Clear();
            for (uint g = 0; g < m_Mujoco!.ModelNgeom; g++)
            {
                if (
                    !m_Mujoco.GetGeomDescriptor(
                        g,
                        out int body,
                        out int type,
                        out _,
                        out _,
                        out _,
                        out Vector3 half
                    )
                    || type != 6
                )
                    continue; // 6 = mjGEOM_BOX
                uint b = (uint)body;
                if (
                    (m_Mujoco.GetBodyNameByIndex(b) ?? "").StartsWith("lucky_body_")
                    && m_Mujoco.GetBodyFreeJointIndex(b) < m_Mujoco.ModelNjnt
                )
                    m_Boxes.Add((b, g, half));
            }
        }

        // Box i from MuJoCo: centre (world), speed, clearance under it on the conveyor, and half extent along a world direction (support function of its box geom).
        private Vector3 BoxCenter(int i)
        {
            m_Mujoco!.GetGeomPoseMj(m_Boxes[i].Geom, out Vector3 p, out _);
            return m_MjToHz * p + m_MjOffset;
        }

        private float BoxSpeed(int i)
        {
            m_Mujoco!.GetBodyPoseMj(m_Boxes[i].Body, out _, out _, out Vector3 v, out _);
            return v.Length();
        }

        private float ChuteGap(int i) =>
            Vector3.Dot(BoxCenter(i) - m_ChuteTop, m_ChuteN) - HalfExtent(i, m_ChuteN);

        private float HalfExtent(int i, Vector3 dir)
        {
            m_Mujoco!.GetGeomPoseMj(m_Boxes[i].Geom, out _, out Quaternion q);
            Quaternion r = m_MjToHz * q;
            Vector3 h = m_Boxes[i].Half;
            return Math.Abs(Vector3.Dot(dir, r * Vector3.Right)) * h.X
                + Math.Abs(Vector3.Dot(dir, r * Vector3.Up)) * h.Y
                + Math.Abs(Vector3.Dot(dir, r * Vector3.Forward)) * h.Z;
        }
    }
}
