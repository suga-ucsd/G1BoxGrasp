using System;
using Hazel;

namespace G1Pipe
{
    // Frames and geometry helpers: MuJoCo poses, pelvis frame, surfaces, small vector maths.
    public partial class G1BoxGrasp
    {
        // Surfaces (conveyor, table) are unit-cube meshes: centre of the top face, and the direction that faces one square
        // (across its long horizontal axis) from the robot's side.
        private static Vector3 TopOf(Entity e) =>
            e.Transform.WorldTranslation
            + e.Transform.WorldRotationQuat * Vector3.Up * (0.5f * e.Transform.WorldScale.Y);

        private Vector3 FaceOf(Entity e)
        {
            TransformComponent t = e.Transform;
            Vector3 across = FlatDir(
                t.WorldRotationQuat
                    * (t.WorldScale.X > t.WorldScale.Z ? Vector3.Forward : Vector3.Right)
            );
            return Vector3.Dot(across, t.WorldTranslation - Pelvis()) >= 0f ? across : across * -1f;
        }

        // Pelvis spot to work on point p of a surface: reach behind p along dir (horizontal), pushed back so the pelvis stays at least
        // EdgeClearance outside the surface's near edge and the legs never hit it.
        private Vector3 StandFor(Entity surface, Vector3 p, float reach, Vector3 dir)
        {
            Vector3 face = FaceOf(surface),
                stand = p - dir * reach;
            float clear =
                Vector3.Dot(surface.Transform.WorldTranslation - stand, face)
                - 0.5f * Math.Min(surface.Transform.WorldScale.X, surface.Transform.WorldScale.Z);
            return clear >= EdgeClearance ? stand : stand - face * (EdgeClearance - clear);
        }

        // How much farther than reach the surface edge forces the robot to stand from p (> 0: p is out of reach by that much).
        private float ReachShort(Entity surface, Vector3 p, float reach) =>
            Vector3.Dot(p - StandFor(surface, p, reach, FaceOf(surface)), FaceOf(surface)) - reach;

        // MuJoCo poses in Hazel world coordinates: axis swap + scene translation, as the reference project did (PLAN.md G22).
        private Vector3 SiteWorld(uint site) =>
            m_MjToHz * m_Mujoco!.GetSitePositionMj(site) + m_MjOffset;

        private Vector3 BodyPos(uint body)
        {
            m_Mujoco!.GetBodyPoseMj(body, out Vector3 p, out _, out _, out _);
            return m_MjToHz * p + m_MjOffset;
        }

        private Quaternion BodyRot(uint body)
        {
            m_Mujoco!.GetBodyPoseMj(body, out _, out Quaternion q, out _, out _);
            return m_MjToHz * q;
        }

        // Pelvis position (world) and horizontal heading, from MuJoCo.
        private Vector3 Pelvis() => BodyPos(m_PelvisId);

        // How fast the body carries a hand held out in front: pelvis speed + its turn (and sway) rate x k_HandLever (m/s).
        private float BodyMotion()
        {
            m_Mujoco!.GetBodyPoseMj(m_PelvisId, out _, out _, out Vector3 v, out Vector3 w);
            return v.Length() + k_HandLever * w.Length();
        }

        private Vector3 Heading() => FlatDir(BodyRot(m_PelvisId) * Vector3.Right);

        // Pelvis coords (forward, up, left) <-> world, using the pelvis heading only so the grip ignores body sway.
        private Vector3 BodyDir(Vector3 b) =>
            Heading() * b.X + Vector3.Up * b.Y + LeftOf(Heading()) * b.Z;

        private Vector3 BodyPoint(Vector3 b) => Pelvis() + BodyDir(b);

        private Vector3 ToBody(Vector3 v) =>
            new Vector3(Vector3.Dot(v, Heading()), v.Y, Vector3.Dot(v, LeftOf(Heading())));

        // Engine is Y-up: yaw = atan2(-z, x), left of a heading = (z, 0, -x); "flat" drops the height.
        private static float YawOf(Vector3 d) => Mathf.Atan2(-d.Z, d.X);

        private static Vector3 LeftOf(Vector3 f) => new Vector3(f.Z, 0f, -f.X);

        private static Vector3 Flat(Vector3 v) => new Vector3(v.X, 0f, v.Z);

        private static Vector3 FlatDir(Vector3 v) => Flat(v).Normalized();

        // (forward, up, outward) offset of one arm: the right arm mirrors the outward part.
        private static Vector3 Mirror(Vector3 o, float side) => new Vector3(o.X, o.Y, side * o.Z);

        // Moves p toward goal by at most step; true once there.
        private static bool MoveTowards(ref Vector3 p, Vector3 goal, float step)
        {
            Vector3 d = goal - p;
            float len = d.Length();
            p = len <= step ? goal : p + d * (step / len);
            return len <= step;
        }
    }
}
