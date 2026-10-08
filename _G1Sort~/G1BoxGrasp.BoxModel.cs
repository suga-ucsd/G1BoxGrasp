using Hazel;

namespace G1Sort
{
    /// <summary>
    /// Everything the controller knows about the box:
    ///  - where it is (ground truth or PushBoxEstimate), tracked by BoxTracker,
    ///  - which vertical face the robot approaches (snapped to real face normals),
    ///  - its extents along the grip frame (the "face frame"),
    ///  - whether it fits the arm span, its mass and the required grip force.
    /// </summary>
    public partial class G1BoxGrasp
    {
        /// <summary>Override to supply box size from elsewhere (collider, spawner). Default: BoxSize.</summary>
        protected virtual Vector3 ResolveBoxSize() => BoxSize;

        // ---- Box source -----------------------------------------------------

        /// <summary>True if a box is available: TargetObject set, or a fresh external estimate.</summary>
        private bool HasBox => UseExternalEstimate ? (m_ExtValid && m_ExtAge <= ExternalEstimateTimeout) : TargetObject != null;

        /// <summary>Box world centre.</summary>
        private Vector3 BoxPos()
        {
            if (UseExternalEstimate) return m_ExtCenter;
            Entity? t = TargetObject;
            return t != null ? t.Transform.WorldTranslation : m_Tracker.Position;
        }

        /// <summary>Box world rotation.</summary>
        private Quaternion BoxRot()
        {
            if (UseExternalEstimate) return m_ExtRot;
            Entity? t = TargetObject;
            return t != null ? t.Transform.WorldRotationQuat : Quaternion.Identity;
        }

        /// <summary>Box size in box-local axes (X, Y = height, Z). Falls back to reference dims if invalid.</summary>
        private Vector3 BoxSizeLocal()
        {
            Vector3 s = UseExternalEstimate ? m_ExtSize : ResolveBoxSize();
            if (s.X > 0.001f && s.Y > 0.001f && s.Z > 0.001f) return s;
            return new Vector3(ReferenceDepth, ReferenceHeight, ReferenceWidth);
        }

        /// <summary>Ages the external estimate and feeds the current box position into the tracker.</summary>
        private void UpdateEstimate(float ts)
        {
            if (UseExternalEstimate) m_ExtAge += ts;
            if (!HasBox) return;
            m_Tracker.Update(BoxPos(), ts, VelocityFilterTime, MaxBoxSpeed);
        }

        // ---- Face selection -------------------------------------------------

        /// <summary>Horizontal direction of the box's local +X axis.</summary>
        private Vector3 BoxAxisX() => Geo.FlatDir(BoxRot() * new Vector3(1f, 0f, 0f), new Vector3(1f, 0f, 0f));

        /// <summary>The box face normal (of the four vertical faces) closest to <paramref name="reference"/>.</summary>
        private Vector3 SnapToBoxFace(Vector3 reference)
        {
            Vector3 aX = BoxAxisX();
            Vector3 aP = Geo.LeftOf(aX);
            float dx = Geo.Dot(aX, reference);
            float dp = Geo.Dot(aP, reference);
            if (Geo.Abs(dx) >= Geo.Abs(dp)) return dx >= 0f ? aX : aX * -1f;
            return dp >= 0f ? aP : aP * -1f;
        }

        /// <summary>
        /// WalkTo only (non-station, AlignToBoxFaces): picks the face normal closest to the
        /// robot's bearing as the approach direction. Keeps it while the bearing stays within
        /// 60 degrees of it (hysteresis).
        /// </summary>
        private void LatchApproachIfNeeded()
        {
            if (!AlignToBoxFaces || StationEntity != null || !HasBox || m_Phase != Phase.WalkTo || !m_Tracker.HasSample)
                return;
            Vector3 b = Geo.FlatDir(m_Tracker.Position - GetPelvis(), GetHorizontalForward());
            if (m_ApproachLatched && Geo.Dot(m_ApproachDir, b) >= 0.5f)
                return;
            m_ApproachDir = SnapToBoxFace(b);
            m_ApproachLatched = true;
        }

        /// <summary>Reference forward: latched grasp frame > approach direction > pelvis forward.</summary>
        private Vector3 GripForward()
        {
            if (m_FrameLatched) return m_LatchedFwd;
            if (m_ApproachLatched && (AlignToBoxFaces || StationEntity != null)) return m_ApproachDir;
            return GetHorizontalForward();
        }

        // ---- Face frame -----------------------------------------------------

        /// <summary>
        /// Exact face extents: width across the hands, depth along the approach, height.
        /// Uses the snapped face, so a yawed box still gives true face dimensions.
        /// </summary>
        private void Dims(out float width, out float depth, out float height)
        {
            width = ReferenceWidth;
            depth = ReferenceDepth;
            height = ReferenceHeight;
            if (!HasBox) return;

            Vector3 size = BoxSizeLocal();
            Vector3 g = SnapToBoxFace(GripForward());
            bool alongX = Geo.Abs(Geo.Dot(g, BoxAxisX())) >= 0.5f;
            depth = alongX ? size.X : size.Z;
            width = alongX ? size.Z : size.X;
            height = size.Y;
        }

        /// <summary>
        /// Writes the face frame every frame: centre, forward (NF), left (NL), half extents.
        /// All hand targets read this (via CurFwd / CurLat and m_FaceHalf*).
        /// </summary>
        private void UpdateFaceFrame()
        {
            m_FaceValid = false;
            if (!HasBox || !m_Tracker.HasSample)
                return;
            Vector3 g = SnapToBoxFace(GripForward());
            float w, d, h;
            Dims(out w, out d, out h);
            m_FaceCenter = m_Tracker.Position;
            m_FaceNF = g;
            m_FaceNL = Geo.LeftOf(g);
            m_FaceHalfW = w * 0.5f;
            m_FaceHalfD = d * 0.5f;
            m_FaceHalfH = h * 0.5f;
            m_FaceValid = true;
        }

        // ---- Size / mass / force -------------------------------------------

        /// <summary>Fails if the box is too wide for the arm span or narrower than MinSqueezeHalfGap.</summary>
        private bool CheckBoxFits()
        {
            if (m_FaceHalfW + HandStandoff > MaxHalfGap)
            {
                Fail("box too wide for arm span (width=" + (2f * m_FaceHalfW).ToString("F3") + " m, max="
                    + (2f * (MaxHalfGap - HandStandoff)).ToString("F3") + " m)");
                return false;
            }
            if (m_FaceHalfW < MinSqueezeHalfGap)
            {
                Fail("box too narrow (width=" + (2f * m_FaceHalfW).ToString("F3") + " m)");
                return false;
            }
            return true;
        }

        /// <summary>Box mass: density * true volume (yaw independent), or BoxMass.</summary>
        private float BoxMassKg()
        {
            if (!UseDensityForMass) return BoxMass;
            Vector3 s = BoxSizeLocal();
            return BoxDensity * s.X * s.Y * s.Z;
        }

        /// <summary>
        /// Required side-face force per hand:
        ///   F = GripSafetyFactor * m * g / (2 * max(mu, 0.05)) * m_ForceBoost
        /// m_ForceBoost rises above 1 while the box slips during lift.
        /// </summary>
        private float RequiredForce()
        {
            float mu = BoxFriction > 0.05f ? BoxFriction : 0.05f;
            return GripSafetyFactor * BoxMassKg() * k_G / (2f * mu) * m_ForceBoost;
        }
    }

}
