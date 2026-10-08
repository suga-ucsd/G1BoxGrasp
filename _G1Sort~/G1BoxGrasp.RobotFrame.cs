using Hazel;

namespace G1Sort
{
    /// <summary>
    /// Robot-body frame helpers and base motion commands:
    ///  - pelvis position / heading,
    ///  - body-relative points (pole targets, shoulder approximations),
    ///  - walk command and heading P-controllers.
    /// </summary>
    public partial class G1BoxGrasp
    {
        /// <summary>pelvis + forward*f + up*u + left*(lateral*side). side = +1 left, -1 right.</summary>
        private Vector3 BodyRelative(float forward, float up, float lateral, float side)
        {
            return GetPelvis() + GetHorizontalForward() * forward + GetHorizontalLeft() * (lateral * side) + new Vector3(0f, up, 0f);
        }

        /// <summary>Sends (vx, vy, yaw rate) to the walking policy slot.</summary>
        private void Walk(float vx, float vy, float yawRate)
        {
            m_Robot?.SetFloat(WalkerSlotId, k_SetVx, vx);
            m_Robot?.SetFloat(WalkerSlotId, k_SetVy, vy);
            m_Robot?.SetFloat(WalkerSlotId, k_SetYawRate, yawRate);
        }

        /// <summary>
        /// Yaw rate that holds the body facing the box: toward the latched approach
        /// direction if there is one, otherwise toward the box centre.
        /// </summary>
        private float FaceYawRate()
        {
            if (!HasBox) return 0f;
            if (m_ApproachLatched && (AlignToBoxFaces || StationEntity != null))
                return Mathf.Clamp(Mathf.WrapToPi(Geo.YawFromDir(m_ApproachDir) - GetPelvisYaw()) * YawGain, -MaxYawRate, MaxYawRate);
            return Mathf.Clamp(HeadingErrorTo(BoxPos()) * YawGain, -MaxYawRate, MaxYawRate);
        }

        /// <summary>Signed heading error (rad) from the pelvis toward a world point.</summary>
        private float HeadingErrorTo(Vector3 worldTarget)
        {
            Vector3 pelvis = GetPelvis();
            return Mathf.WrapToPi(Mathf.Atan2(-(worldTarget.Z - pelvis.Z), worldTarget.X - pelvis.X) - GetPelvisYaw());
        }

        /// <summary>Pelvis world position (falls back to this entity's position).</summary>
        private Vector3 GetPelvis()
        {
            if (m_Mujoco == null || m_PelvisBodyId == uint.MaxValue)
                return Transform.WorldTranslation;
            return m_Mujoco.GetPosition(m_PelvisBodyId);
        }

        /// <summary>Pelvis yaw (rad).</summary>
        private float GetPelvisYaw()
        {
            Vector3 fwd = GetHorizontalForward();
            return Mathf.Atan2(-fwd.Z, fwd.X);
        }

        /// <summary>Horizontal unit forward of the pelvis (pelvis local +X).</summary>
        private Vector3 GetHorizontalForward()
        {
            Vector3 fallback = new Vector3(1f, 0f, 0f);
            if (m_Mujoco == null || m_PelvisBodyId == uint.MaxValue)
                return fallback;
            return Geo.FlatDir(m_Mujoco.GetOrientation(m_PelvisBodyId) * fallback, fallback);
        }

        /// <summary>Horizontal unit left of the pelvis.</summary>
        private Vector3 GetHorizontalLeft() => Geo.LeftOf(GetHorizontalForward());
    }

}
