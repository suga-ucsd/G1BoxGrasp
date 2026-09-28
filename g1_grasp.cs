using System;
using Hazel;

namespace G1Pipe
{


    public class G1BoxGrasp : Entity
    {

        private enum Phase { Walk, Grasp };

        private static readonly string[] s_LegsAndWaistOnly = {
            "left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint",
            "left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
            "right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint",
            "right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
            "waist_yaw_joint","waist_roll_joint","waist_pitch_joint",
        };

        // Walker policy input IDs: SetFloat(slot, id, value) writes one command channel.
        private const uint k_SetVx = 1u;       // forward speed (m/s)
        private const uint k_SetVy = 2u;       // sideways speed (m/s)
        private const uint k_SetYawRate = 3u;  // turn rate (rad/s)

        [Group("Walk Position")]
        public Vector3 position = new Vector3(0f, 0f, 0f);

        [Group("Walk")]
        public float WalkSpeed = 0.112f;        // m/s
        [Group("Walk")]
        public float YawGain = 2.0f;            // turn rate per radian of heading error
        [Group("Walk")]
        public float MaxYawRate = 1.0f;         // rad/s
        [Group("Walk")]
        public float ArriveTolerance = 0.05f;   // m, horizontal distance that counts as arrived

        [Group("Policy")]
        public uint WalkerSlotId = 1u;
        [Group("Policy")]
        public string PelvisBodyName = "pelvis";

        [HideFromEditor] private RobotControllerComponent? m_Robot;
        [HideFromEditor] private MujocoSceneComponent? m_Mujoco;
        [HideFromEditor] private uint m_PelvisBodyId = uint.MaxValue;
        [HideFromEditor] private Phase m_Phase = Phase.Walk;



        /// <summary>Send a velocity command to the walking policy.</summary>
        private void Walk(float vx, float vy, float yawRate)
        {
            m_Robot!.SetFloat(WalkerSlotId, k_SetVx, vx);
            m_Robot.SetFloat(WalkerSlotId, k_SetVy, vy);
            m_Robot.SetFloat(WalkerSlotId, k_SetYawRate, yawRate);
        }

        private void WalkTo()
        {
            // Where the pelvis is and which way it faces.
            Vector3 pelvis = m_Mujoco!.GetPosition(m_PelvisBodyId);
            Vector3 fwd = m_Mujoco.GetOrientation(m_PelvisBodyId) * new Vector3(1f, 0f, 0f);  // pelvis local +X
            float yaw = Mathf.Atan2(-fwd.Z, fwd.X);   // engine is Y-up, so yaw lives in the X/Z plane

            // Horizontal vector to the goal (ignore height).
            Vector3 toGoal = new Vector3(position.X - pelvis.X, 0f, position.Z - pelvis.Z);
            float dist = toGoal.Length();
            Log.Info($"{dist}");

            if (dist <= ArriveTolerance)
            {
                Walk(0f, 0f, 0f);
                m_Phase = Phase.Grasp;
                Log.Info("[G1] arrived");
                return;
            }

            // Heading error in (-pi, pi]: how far we need to turn to face the goal.
            float headingErr = Mathf.WrapToPi(Mathf.Atan2(-toGoal.Z, toGoal.X) - yaw);
            float yawRate = Mathf.Clamp(headingErr * YawGain, -MaxYawRate, MaxYawRate);

            float speed = dist < 0.3f ? WalkSpeed * 0.5f : WalkSpeed;
            // Turn in place until roughly facing the goal, then walk; slow down near the end.
            bool facing = headingErr > -0.6f && headingErr < 0.6f;
            Walk(facing ? speed : 0f, 0f, yawRate);
        }

        protected override void OnCreate()
        {

            m_Robot = GetComponent<RobotControllerComponent>();
            m_Mujoco = GetComponent<MujocoSceneComponent>();

            Log.Info("Walking");
            if (m_Mujoco != null)
                m_PelvisBodyId = m_Mujoco.GetBodyID(PelvisBodyName);
            m_Robot?.SetPolicyActive(WalkerSlotId, true);
            m_Robot?.SetDrivenJoints(WalkerSlotId, s_LegsAndWaistOnly);

            if (m_PelvisBodyId == uint.MaxValue)
                Log.Info($"[G1] body '{PelvisBodyName}' not found");

        }
        protected override void OnUpdate(float ts)
        {

            if (m_Mujoco == null || m_Robot == null || m_PelvisBodyId == uint.MaxValue)
                return;
            switch (m_Phase)
            {
                case Phase.Walk:
                    WalkTo();
                    Log.Info("Walking");
                    break;
                case Phase.Grasp: Walk(0f, 0f, 0f); break;   // TODO: arms
            }
        }

    }

}
