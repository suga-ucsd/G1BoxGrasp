using Hazel;

namespace G1Pipe
{
    // Walker test mode (WalkerTest): measures how the body moves for each command.
    public partial class G1BoxGrasp
    {
        private int m_TestIdx; // walker test: command being measured

        private Vector3 m_TestFrom,
            m_TestFootL,
            m_TestFootR; // walker test: pelvis and feet (ankle roll links) when the command started

        private float m_TestYaw; // walker test: heading when the command started

        // Walker test commands: (rotator?, vx, vy, yaw rate, s sent). Short bursts at the smallest commands that step (each followed by its
        // reverse), right turns with the walker (alone and with a side step), then left turns in place at three rates.
        private static readonly (bool Rot, float Vx, float Vy, float Yaw, float T)[] s_WalkerTests =
        {
            (false, -0.4f, 0f, 0f, 0.25f),
            (false, 0.6f, 0f, 0f, 0.25f),
            (false, -0.4f, 0f, 0f, 0.5f),
            (false, 0.6f, 0f, 0f, 0.5f),
            (false, 0f, -0.4f, 0f, 0.25f),
            (false, 0f, 0.4f, 0f, 0.25f),
            (false, 0f, -0.4f, 0f, 0.5f),
            (false, 0f, 0.4f, 0f, 0.5f),
            (false, 0f, 0f, -1.5f, 2f),
            (false, 0f, 0f, -2f, 2f),
            (false, 0f, -0.4f, -1f, 1f),
            (false, 0f, 0.4f, -1f, 1f),
            (false, 0f, 0f, 0.6f, 2f),
            (false, 0f, 0f, 1f, 2f),
            (false, 0f, 0f, 1.5f, 2f),
        };

        private const float k_TestSettle = 2f,
            k_TestCoast = 1.5f,
            k_TestWait = 10f; // walker test: s still before each command (waited for at most k_TestWait), s to coast to a stop after it

        private Vector3 m_TestEnd; // walker test: pelvis when the command stopped

        private float m_TestEndYaw; // walker test: heading when the command stopped

        // Walker test, one command per pass: settle (still body), note pose, send the command for its time (rotator swapped in for its
        // commands), zero it (rotator: while still active, then back to the walker), coast, then log the real move against the commanded one.
        private void StepWalkerTest()
        {
            SendHands(Fold, false);
            var c = s_WalkerTests[m_TestIdx];
            switch (m_Step)
            {
                case 0:
                    if (
                        m_StepTime < k_TestSettle
                        || (m_Still < StillTime && m_StepTime < k_TestWait)
                    )
                        return;
                    m_TestFrom = Pelvis();
                    m_TestYaw = YawOf(Heading());
                    m_TestFootL = BodyPos(m_Mujoco!.GetBodyID("left_ankle_roll_link"));
                    m_TestFootR = BodyPos(m_Mujoco.GetBodyID("right_ankle_roll_link"));
                    if (c.Rot)
                        SetLocomotion(RotatorSlotId);
                    NextStep();
                    return;
                case 1:
                    Drive(c.Vx, c.Vy, c.Yaw);
                    if ((int)(m_StepTime / 0.25f) != (int)((m_StepTime - m_Dt) / 0.25f))
                    {
                        m_Mujoco!.GetBodyPoseMj(
                            m_PelvisId,
                            out _,
                            out _,
                            out Vector3 v,
                            out Vector3 w
                        );
                        Vector3 hv = m_MjToHz * v;
                        Log.Info(
                            $"[G1] WT   t={m_StepTime:F1} s: pelvis moving fwd {Vector3.Dot(hv, Heading()):F2} left {Vector3.Dot(hv, LeftOf(Heading())):F2} m/s, turning {w.Length():F2} rad/s, height {Pelvis().Y:F3} m"
                        );
                    }
                    if (m_StepTime >= c.T)
                    {
                        m_TestEnd = Pelvis();
                        m_TestEndYaw = YawOf(Heading());
                        NextStep();
                    }
                    return;
                default:
                    if (m_Active == RotatorSlotId)
                    {
                        if (m_StepTime < k_DecelTime)
                            return;
                        SetLocomotion(WalkerSlotId);
                    }
                    if (m_StepTime < k_TestCoast)
                        return;
                    Vector3 fwd = new Vector3(Mathf.Cos(m_TestYaw), 0f, -Mathf.Sin(m_TestYaw)),
                        d = Flat(Pelvis() - m_TestFrom),
                        dc = Flat(m_TestEnd - m_TestFrom);
                    Log.Info(
                        $"[G1] WT {m_TestIdx + 1}/{s_WalkerTests.Length} {(c.Rot ? "rotator" : "walker")} vx {c.Vx:F2} vy {c.Vy:F2} yaw {c.Yaw:F2} for {c.T:F2} s"
                            + $" -> while commanded: fwd {Vector3.Dot(dc, fwd):F3} m, left {Vector3.Dot(dc, LeftOf(fwd)):F3} m, turned {Mathf.WrapToPi(m_TestEndYaw - m_TestYaw) * Mathf.Rad2Deg:F1} deg"
                            + $" | total after coasting {k_TestCoast:F1} s: fwd {Vector3.Dot(d, fwd):F3} m, left {Vector3.Dot(d, LeftOf(fwd)):F3} m, turned {Mathf.WrapToPi(YawOf(Heading()) - m_TestYaw) * Mathf.Rad2Deg:F1} deg"
                            + $" (asked fwd {c.Vx * c.T:F2} m, left {c.Vy * c.T:F2} m, turn {c.Yaw * c.T * Mathf.Rad2Deg:F0} deg)"
                            + $" | feet moved L {Flat(BodyPos(m_Mujoco!.GetBodyID("left_ankle_roll_link")) - m_TestFootL).Length():F3} m, R {Flat(BodyPos(m_Mujoco.GetBodyID("right_ankle_roll_link")) - m_TestFootR).Length():F3} m"
                    );
                    if (++m_TestIdx < s_WalkerTests.Length)
                        Advance(Phase.WalkerTest);
                    else
                    {
                        Log.Info("[G1] walker test done");
                        Advance(Phase.Done);
                    }
                    return;
            }
        }
    }
}
