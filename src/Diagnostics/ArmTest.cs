using System;
using System.Collections.Generic;
using Hazel;

namespace G1Pipe
{
    // Arm test mode (ArmTest): IK reach and tracking trials.
    public partial class G1BoxGrasp
    {
        private System.Random m_Rng = new System.Random();

        private int m_ArmIdx; // arm test: trial

        private readonly Vector3[] m_ArmGoal = new Vector3[2]; // arm test: world targets of the trial (left, right)

        private string m_ArmPlan = ""; // arm test: the trial's targets, for its result line

        private float m_ArmReached,
            m_JointMax,
            m_BodyMax; // arm test: s to arrive; deg/s fastest arm joint and m/s body motion while holding

        private readonly float[] m_ErrSum = new float[2],
            m_ErrSq = new float[2],
            m_ErrMax = new float[2];

        private readonly float[][] m_PrevQ = { new float[4], new float[4] };

        private int m_ErrN;

        private bool m_ArmCapped;

        private readonly List<(
            float Err,
            float Jit,
            float Speed,
            string Line,
            string Key
        )> m_ArmResults = new List<(float, float, float, string, string)>();

        private string m_ArmKey = ""; // arm test: sweep and value of the trial (sweep mode)

        // Arm test sweeps: (sweep, centre ahead, centre over the pelvis, centre left, hands apart, slide back to this far ahead (0 = no slide); m,
        // pelvis coords). One value changes at a time from a usual grip (0.32 ahead, 0.05 up, centred, 0.32 apart), well past the arms' limits
        // (no reach filter); "far" stretches both arms straight ahead to full reach, "pull" then slides the hands back like dragging a far box in.
        private static readonly (
            string Sweep,
            float F,
            float Up,
            float C,
            float Apart,
            float PullTo
        )[] s_ArmSweep =
        {
            ("height", 0.32f, -0.10f, 0f, 0.32f, 0f),
            ("height", 0.32f, -0.05f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.05f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.10f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.15f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.20f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.25f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.30f, 0f, 0.32f, 0f),
            ("height", 0.32f, 0.35f, 0f, 0.32f, 0f),
            ("ahead", 0.20f, 0.05f, 0f, 0.32f, 0f),
            ("ahead", 0.25f, 0.05f, 0f, 0.32f, 0f),
            ("ahead", 0.30f, 0.05f, 0f, 0.32f, 0f),
            ("ahead", 0.35f, 0.05f, 0f, 0.32f, 0f),
            ("ahead", 0.40f, 0.05f, 0f, 0.32f, 0f),
            ("ahead", 0.45f, 0.05f, 0f, 0.32f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.16f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.20f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.24f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.28f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.32f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.36f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.40f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.44f, 0f),
            ("apart", 0.32f, 0.05f, 0f, 0.48f, 0f),
            ("offset", 0.32f, 0.05f, -0.15f, 0.32f, 0f),
            ("offset", 0.32f, 0.05f, -0.10f, 0.32f, 0f),
            ("offset", 0.32f, 0.05f, -0.05f, 0.32f, 0f),
            ("offset", 0.32f, 0.05f, 0f, 0.32f, 0f),
            ("offset", 0.32f, 0.05f, 0.05f, 0.32f, 0f),
            ("offset", 0.32f, 0.05f, 0.10f, 0.32f, 0f),
            ("offset", 0.32f, 0.05f, 0.15f, 0.32f, 0f),
            ("far", 0.40f, 0.05f, 0f, 0.32f, 0f),
            ("far", 0.43f, 0.05f, 0f, 0.32f, 0f),
            ("far", 0.46f, 0.05f, 0f, 0.32f, 0f),
            ("far", 0.49f, 0.05f, 0f, 0.32f, 0f),
            ("far", 0.40f, 0.10f, 0f, 0.32f, 0f),
            ("far", 0.43f, 0.10f, 0f, 0.32f, 0f),
            ("far", 0.46f, 0.10f, 0f, 0.32f, 0f),
            ("far", 0.49f, 0.10f, 0f, 0.32f, 0f),
            ("pull", 0.46f, 0.05f, 0f, 0.32f, 0.28f),
            ("pull", 0.49f, 0.05f, 0f, 0.32f, 0.28f),
            ("pull", 0.46f, 0.05f, 0f, 0.36f, 0.28f),
            ("pull", 0.49f, 0.10f, 0f, 0.36f, 0.28f),
        };

        private int ArmTrials => ArmTestSweep ? s_ArmSweep.Length : ArmTestTrials;

        private Vector4 m_ArmPull; // arm test trial slide: (slide back to this far ahead, 0 = none; up; centre left; half separation)

        private const float k_PullSpeed = 0.08f; // m/s the arm test slides the hands back (dragging a far box in)

        // Both arms' IK joints (deg): shoulder pitch / roll / yaw and elbow, with how far the elbow is from straight (0 = straight).
        private string ArmAngles()
        {
            string A(Arm a) =>
                $"{a.Name[0]} shoulder {a.Joints[0].Value * Mathf.Rad2Deg:F0}/{a.Joints[1].Value * Mathf.Rad2Deg:F0}/{a.Joints[2].Value * Mathf.Rad2Deg:F0} elbow {a.Joints[3].Value * Mathf.Rad2Deg:F0} (bent {90f - a.Joints[3].Value * Mathf.Rad2Deg:F0})";
            return $"angles {A(m_Arms[0])}, {A(m_Arms[1])} deg";
        }

        // Resets the arm test error / joint-speed / body statistics.
        private void ResetArmStats()
        {
            m_ErrN = 0;
            m_JointMax = m_BodyMax = 0f;
            m_ArmCapped = false;
            for (int s = 0; s < 2; s++)
            {
                m_ErrSum[s] = m_ErrSq[s] = m_ErrMax[s] = 0f;
                for (int i = 0; i < 4; i++)
                    m_PrevQ[s][i] = (float)m_Arms[s].Joints[i].Value;
            }
        }

        // One frame of arm test statistics: each hand's error against its reference point (target, or the path point while sliding).
        private void ArmStats(Func<Arm, Vector3> reference)
        {
            for (int s = 0; s < 2; s++)
            {
                Arm a = m_Arms[s];
                float e = (SiteWorld(a.Site) - reference(a)).Length();
                m_ErrSum[s] += e;
                m_ErrSq[s] += e * e;
                m_ErrMax[s] = Math.Max(m_ErrMax[s], e);
                for (int i = 0; i < 4; i++)
                {
                    float q = (float)a.Joints[i].Value;
                    m_JointMax = Math.Max(
                        m_JointMax,
                        Math.Abs(q - m_PrevQ[s][i]) / m_Dt * Mathf.Rad2Deg
                    );
                    m_PrevQ[s][i] = q;
                }
                m_ArmCapped |= a.Capped;
            }
            m_ErrN++;
            m_BodyMax = Math.Max(m_BodyMax, BodyMotion());
        }

        private const float k_ArmSettle = 1f,
            k_ArmHold = 2f,
            k_ArmMoveMax = 6f; // arm test: s still before a trial, s held at the targets, s a move may take

        // Arm test trial target pair, in pelvis coords (forward, up, left) when it starts. Random: centre 0.25-0.40 m ahead, 0-0.30 m over the
        // pelvis, up to 0.08 m off centre; hands 0.10-0.22 m either side of it (boxes ~0.16-0.40 m wide), resampled until both are within
        // PlanReach. Sweep (ArmTestSweep): the next s_ArmSweep entry, whatever its reach.
        private bool PickArmTrial()
        {
            float R(float lo, float hi) => lo + (float)m_Rng.NextDouble() * (hi - lo);
            for (int k = 0; k < 50; k++)
            {
                (string Sweep, float F, float Up, float C, float Apart, float PullTo) sw =
                    ArmTestSweep
                        ? s_ArmSweep[m_ArmIdx]
                        : (
                            "",
                            R(0.25f, 0.40f),
                            R(0f, 0.30f),
                            R(-0.08f, 0.08f),
                            2f * R(0.10f, 0.22f),
                            0f
                        );
                float f = sw.F,
                    up = sw.Up,
                    c = sw.C,
                    w = 0.5f * sw.Apart;
                Vector3 l = BodyPoint(new Vector3(f, up, c + w)),
                    r = BodyPoint(new Vector3(f, up, c - w));
                float rl = (l - BodyPos(m_Arms[0].Shoulder)).Length() / k_ArmLength,
                    rr = (r - BodyPos(m_Arms[1].Shoulder)).Length() / k_ArmLength;
                if (!ArmTestSweep && Math.Max(rl, rr) > PlanReach)
                    continue;
                float v =
                    sw.Sweep == "height" ? up
                    : sw.Sweep == "ahead" ? f
                    : sw.Sweep == "apart" ? sw.Apart
                    : sw.Sweep == "offset" ? c
                    : f;
                m_ArmKey = ArmTestSweep ? $"{sw.Sweep} {v:+0.00;-0.00} m" : "";
                m_ArmGoal[0] = l;
                m_ArmGoal[1] = r;
                m_ArmPull = new Vector4(sw.PullTo, up, c, w);
                m_ArmPlan =
                    $"centre {f:F2} m ahead, {up:F2} m over the pelvis, {c:+0.00;-0.00} m left | hands {2f * w:F2} m apart | reach L {rl * 100f:F0}% R {rr * 100f:F0}%";
                return true;
            }
            return false;
        }

        private Vector3 ArmGoal(Arm a) => m_ArmGoal[a.Side > 0f ? 0 : 1];

        // Arm test, one trial per pass: still body, new target pair, both hands move there together (same paths, IK re-seed, wrist servo
        // and cap as a reach), hold k_ArmHold s measuring each hand's error (mean, max, jitter), the fastest arm joint and the body's motion,
        // log the trial, fold again; after the last trial a summary of the worst ones.
        private void StepArmTest()
        {
            switch (m_Step)
            {
                case 0:
                    SendHands(Fold, false);
                    if (m_StepTime < k_ArmSettle || (m_Still < StillTime && m_StepTime < 10f))
                        return;
                    if (!PickArmTrial())
                    {
                        Log.Warn(
                            "[G1] arm test: no reachable target pair found from here - move the robot"
                        );
                        Advance(Phase.Done);
                        return;
                    }
                    Log.Info($"[G1] AT {m_ArmIdx + 1}/{ArmTrials} go: {m_ArmPlan}");
                    SendHands(ArmGoal, true);
                    NextStep();
                    return;
                case 1:
                    SendHands(ArmGoal, false);
                    if (Gliding() && m_StepTime < k_ArmMoveMax)
                        return;
                    m_ArmReached = m_StepTime;
                    ResetArmStats();
                    NextStep();
                    return;
                case 2:
                    SendHands(ArmGoal, false);
                    ArmStats(ArmGoal);
                    if (m_StepTime < k_ArmHold)
                        return;
                    float Mean(int s) => m_ErrSum[s] / m_ErrN;
                    float Jit(int s) =>
                        (float)Math.Sqrt(Math.Max(0f, m_ErrSq[s] / m_ErrN - Mean(s) * Mean(s)));
                    string line =
                        $"{m_ArmPlan} | {(m_ArmReached >= k_ArmMoveMax ? "move timed out" : $"arrived in {m_ArmReached:F1} s")}"
                        + $" | held {k_ArmHold:F0} s: error L mean {Mean(0):F3} max {m_ErrMax[0]:F3} jitter {Jit(0):F3}, R mean {Mean(1):F3} max {m_ErrMax[1]:F3} jitter {Jit(1):F3} m"
                        + $" | fastest arm joint {m_JointMax:F0} deg/s | body {m_BodyMax:F2} m/s at the hands{(m_ArmCapped ? " | IK target capped" : "")} | {ArmAngles()}";
                    Log.Info($"[G1] AT {m_ArmIdx + 1}/{ArmTrials} result: {line}");
                    m_ArmResults.Add(
                        (
                            Math.Max(Mean(0), Mean(1)),
                            Math.Max(Jit(0), Jit(1)),
                            m_JointMax,
                            line,
                            $"{m_ArmKey}: error L {Mean(0):F3} R {Mean(1):F3} m (max {Math.Max(m_ErrMax[0], m_ErrMax[1]):F3}), jitter {Math.Max(Jit(0), Jit(1)):F3} m, fastest joint {m_JointMax:F0} deg/s, body {m_BodyMax:F2} m/s{(m_ArmCapped ? ", capped" : "")} | {ArmAngles()}"
                        )
                    );
                    if (m_ArmPull.X > 0f)
                    {
                        // Slide: both hands straight back to PullTo ahead at k_PullSpeed (one fixed path), tracking measured against it.
                        m_ArmGoal[0] = BodyPoint(
                            new Vector3(m_ArmPull.X, m_ArmPull.Y, m_ArmPull.Z + m_ArmPull.W)
                        );
                        m_ArmGoal[1] = BodyPoint(
                            new Vector3(m_ArmPull.X, m_ArmPull.Y, m_ArmPull.Z - m_ArmPull.W)
                        );
                        foreach (Arm a in m_Arms)
                        {
                            a.From = a.Cmd;
                            a.To = ArmGoal(a);
                            a.T = Math.Max(k_MinSegment, (a.To - a.From).Length() / k_PullSpeed);
                            a.Tau = 0f;
                            a.Rising = false;
                        }
                        ResetArmStats();
                        Log.Info(
                            $"[G1] AT {m_ArmIdx + 1}/{ArmTrials} sliding the hands back to {m_ArmPull.X:F2} m ahead at {k_PullSpeed:F2} m/s"
                        );
                        NextStep();
                        return;
                    }
                    SendHands(Fold, true);
                    m_Step = 3; // no slide: straight to folding
                    NextStep();
                    return;
                case 3:
                    SendHands(ArmGoal, false);
                    ArmStats(a => a.Cmd); // tracking error along the slide path
                    if (Gliding() || m_StepTime < m_Arms[0].T + 1f)
                        return;
                    float pm = Math.Max(m_ErrSum[0], m_ErrSum[1]) / m_ErrN;
                    string pull =
                        $"slide back to {m_ArmPull.X:F2} m ahead: tracking error L mean {m_ErrSum[0] / m_ErrN:F3} max {m_ErrMax[0]:F3}, R mean {m_ErrSum[1] / m_ErrN:F3} max {m_ErrMax[1]:F3} m"
                        + $" | end error L {(SiteWorld(m_Arms[0].Site) - m_ArmGoal[0]).Length():F3} R {(SiteWorld(m_Arms[1].Site) - m_ArmGoal[1]).Length():F3} m"
                        + $" | fastest arm joint {m_JointMax:F0} deg/s | body {m_BodyMax:F2} m/s{(m_ArmCapped ? " | capped" : "")} | {ArmAngles()}";
                    Log.Info($"[G1] AT {m_ArmIdx + 1}/{ArmTrials} {pull}");
                    m_ArmResults.Add((pm, 0f, m_JointMax, pull, $"{m_ArmKey} {pull}"));
                    SendHands(Fold, true);
                    NextStep();
                    return;
                default:
                    SendHands(Fold, false);
                    if (Gliding() && m_StepTime < k_ArmMoveMax)
                        return;
                    if (++m_ArmIdx < ArmTrials)
                    {
                        Advance(Phase.ArmTest);
                        return;
                    }
                    int missed = 0,
                        shaky = 0;
                    foreach (var r in m_ArmResults)
                    {
                        missed += r.Err > 0.03f ? 1 : 0;
                        shaky += r.Jit > 0.005f || r.Speed > 120f ? 1 : 0;
                    }
                    Log.Info(
                        $"[G1] arm test done: {m_ArmResults.Count} trials, {missed} missed (mean error > 3 cm), {shaky} shaky (jitter > 5 mm or a joint > 120 deg/s)"
                    );
                    if (ArmTestSweep)
                        foreach (var r in m_ArmResults)
                            Log.Info($"[G1] AT sweep {r.Key}");
                    m_ArmResults.Sort((x, y) => y.Err.CompareTo(x.Err));
                    for (int i = 0; i < Math.Min(5, m_ArmResults.Count); i++)
                        Log.Info($"[G1] AT worst error #{i + 1}: {m_ArmResults[i].Line}");
                    m_ArmResults.Sort((x, y) => y.Jit.CompareTo(x.Jit));
                    for (int i = 0; i < Math.Min(3, m_ArmResults.Count); i++)
                        Log.Info($"[G1] AT most jitter #{i + 1}: {m_ArmResults[i].Line}");
                    Advance(Phase.Done);
                    return;
            }
        }
    }
}
