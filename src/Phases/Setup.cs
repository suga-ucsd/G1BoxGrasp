using System;
using System.Collections.Generic;
using Hazel;

namespace G1Pipe
{
    // Begin: finds the robot, arms, boxes, conveyor and table, and sets the start pose.
    public partial class G1BoxGrasp
    {
        // First frames (MuJoCo isn't ready in OnCreate): find the pelvis and boxes, set up the arm IK (failing loudly on a missing part), read the conveyor, fold the arms.
        private void Begin()
        {
            m_MjToHz = m_Mujoco!.GetAxisHzToMj().Conjugate;
            m_MjOffset = m_Mujoco.GetSceneTranslation();
            m_PelvisId = m_Mujoco.GetBodyID("pelvis");
            m_TorsoId = m_Mujoco.GetBodyID("torso_link");
            m_WaistAdr = Array.ConvertAll(
                s_LegsAndWaistOnly[^3..],
                j => m_Mujoco.GetJointQposAdr(m_Mujoco.GetJointID(j))
            );
            m_Waist = Array.ConvertAll(s_LegsAndWaistOnly[^3..], j => m_Mujoco.GetActuator(j)!);
            if (ChuteEntity == null || PlaceTarget == null)
            {
                Fail("assign ChuteEntity (conveyor) and PlaceTarget (table top)", false);
                return;
            }
            FindBoxes();
            if (m_PelvisId == uint.MaxValue || m_Boxes.Count == 0)
            {
                if (m_PhaseTime > 2f)
                    Fail(
                        m_PelvisId == uint.MaxValue
                            ? "no 'pelvis' body in the MuJoCo model"
                            : "no free-moving MuJoCo boxes in the scene",
                        false
                    );
                return; // scene bodies can join the model a few frames after start: retry
            }
            foreach (var b in m_Boxes)
                m_Mujoco.UnweldBody(b.Body); // clear welds left by earlier versions of this script (welds default to persistent)
            m_Fails = new int[m_Boxes.Count];
            foreach (Arm a in m_Arms)
            {
                // The IK owns the shoulders and the elbow; the script owns the wrists (OrientWrists) and the thumb and finger bases (PoseHands).
                var acts = new List<MujocoActuator>();
                foreach (
                    string j in new[]
                    {
                        "_shoulder_pitch_joint",
                        "_shoulder_roll_joint",
                        "_shoulder_yaw_joint",
                        "_elbow_joint",
                        "_wrist_roll_joint",
                        "_wrist_pitch_joint",
                        "_wrist_yaw_joint",
                        "_hand_thumb_1_joint",
                        "_hand_thumb_2_joint",
                        "_hand_index_0_joint",
                        "_hand_middle_0_joint",
                    }
                )
                {
                    MujocoActuator? act = m_Mujoco.GetActuator(a.Name + j);
                    if (act == null)
                    {
                        Fail($"no actuator '{a.Name + j}'", false);
                        return;
                    }
                    act.IK = !j.StartsWith("_wrist") && !j.StartsWith("_hand");
                    acts.Add(act);
                }
                // Start tucked: the shoulder and elbow start pose written straight into the sim, so the spawn's forearms-forward pose
                // never swings out over the conveyor while the body drops onto its feet.
                float[] tuck =
                {
                    StartShoulderDeg.X,
                    a.Side * StartShoulderDeg.Y,
                    a.Side * StartShoulderDeg.Z,
                    StartElbowDeg,
                };
                string[] arm =
                {
                    "_shoulder_pitch_joint",
                    "_shoulder_roll_joint",
                    "_shoulder_yaw_joint",
                    "_elbow_joint",
                };
                for (int i = 0; i < 4; i++)
                {
                    float q = tuck[i] * Mathf.Deg2Rad;
                    m_Mujoco.SetQposByIndex(
                        m_Mujoco.GetJointQposAdr(m_Mujoco.GetJointID(a.Name + arm[i])),
                        q
                    );
                    acts[i].Value = q;
                }
                a.ArmAdr = Array.ConvertAll(
                    arm,
                    j => m_Mujoco.GetJointQposAdr(m_Mujoco.GetJointID(a.Name + j))
                );
                a.Joints = acts.GetRange(0, 7).ToArray();
                a.Hand = acts.GetRange(7, 4).ToArray();
                a.WristQ = new[]
                {
                    (float)a.Joints[4].Value,
                    (float)a.Joints[5].Value,
                    (float)a.Joints[6].Value,
                };
                a.WristLink = Array.ConvertAll(
                    new[] { "_wrist_roll_link", "_wrist_pitch_link", "_wrist_yaw_link" },
                    l => m_Mujoco.GetBodyID(a.Name + l)
                );
                a.WristAdr = Array.ConvertAll(
                    new[] { "_wrist_roll_joint", "_wrist_pitch_joint", "_wrist_yaw_joint" },
                    j => m_Mujoco.GetJointQposAdr(m_Mujoco.GetJointID(a.Name + j))
                );
                a.Site = m_Mujoco.GetSiteID(a.Name + "_grasp_site");
                a.Shoulder = m_Mujoco.GetBodyID(a.Name + "_shoulder_pitch_link");
                if (
                    a.Site == uint.MaxValue
                    || a.Shoulder == uint.MaxValue
                    || Array.IndexOf(a.WristLink, uint.MaxValue) >= 0
                )
                {
                    Fail(
                        $"no site '{a.Name}_grasp_site' or body '{a.Name}_shoulder_pitch_link' / '{a.Name}_wrist_*_link'",
                        false
                    );
                    return;
                }
                m_Robot!.SetInputBool(new Identifier(a.Input + "Solve_Orientation"), false); // position only
                m_Robot.SetInputBool(new Identifier(a.Input + "High_Accuracy"), true);
                m_Robot.SetInputFloat(new Identifier(a.Input + "Pole_Weight"), 1f);
                HandGeoms(a);
                a.Base = new float[a.Geoms.Length];
                a.Touch = new bool[a.Geoms.Length];
            }
            m_ChuteN = ChuteEntity.Transform.WorldRotationQuat * Vector3.Up;
            m_ChuteTop = TopOf(ChuteEntity);
            m_Downhill = m_ChuteN * m_ChuteN.Y - Vector3.Up; // gravity along the belt plane: length = sin(tilt)
            if (CatchBox)
            {
                if (m_Downhill.Length() < 0.01f)
                {
                    Fail(
                        "CatchBox needs the conveyor tilted (> 0.6 deg along its length) so boxes slide down it",
                        false
                    );
                    return;
                }
                m_Downhill = m_Downhill.Normalized();
                Log.Info(
                    $"[G1] catch: belt tilted {Mathf.Asin(Math.Abs(m_Downhill.Y)) * Mathf.Rad2Deg:F1} deg, boxes slide toward {F(m_Downhill)} (frictionless: {SlideAccel():F2} m/s^2)"
                );
            }
            SetSafeY();
            if (ApproachGap > 0.1f)
                Log.Warn(
                    $"[G1] ApproachGap is {ApproachGap:F2} m: each palm aims that far outside the box (arms flung sideways) - expected 0.02-0.05"
                );
            Log.Info(
                $"[G1] ready: {m_Boxes.Count} boxes, hand geoms L={m_Arms[0].Geoms.Length} R={m_Arms[1].Geoms.Length}, pelvis at {Pelvis()}"
            );
            SendHands(Fold, true);
            foreach (Arm a in m_Arms)
            {
                a.Cmd = Fold(a); // first IK targets at the fold, not where the hands were before the tuck
                SendTarget(a);
            }
            Advance(Phase.FindBox);
        }
    }
}
