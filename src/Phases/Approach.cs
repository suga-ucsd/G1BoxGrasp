using System;
using Hazel;

namespace G1Pipe
{
    // Approach to a resting box and the reach check before the grasp (Approach, Ready).
    public partial class G1BoxGrasp
    {
        // Approach: line the body up with the box (Ready follows).
        private void EnterApproach()
        {
            m_Moves = 0;
            Advance(Phase.Approach);
        }

        // Approach, one correction per cycle: wait for a still body, measure the box in its own frame, then make the first correction needed -
        // turn in place to face its front face square, side-step until the pelvis-to-box line points straight ahead, step forward / back
        // until the front face is FaceDistance ahead (never closer than EdgeClearance to the conveyor edge) - then coast to a stop and
        // measure again. Lined up (or out of corrections): Ready, whose reach check decides.
        private void StepApproach()
        {
            SendHands(Fold, false);
            if (m_Step == 0)
            {
                if (AwaitStill(m_StepTime, "before lining up with the box"))
                    return;
                SetGripFrame(Heading());
                Vector3 rel = Flat(m_Center - Pelvis());
                float off = Mathf.WrapToPi(YawOf(m_Fwd) - YawOf(Heading())),
                    side = Vector3.Dot(rel, m_Lat),
                    face = Vector3.Dot(rel, m_Fwd) - m_HalfD,
                    room = ChuteRoom();
                string state =
                    $"facing {off * Mathf.Rad2Deg:F0} deg off the box, its centre {side:F2} m left of straight ahead, front face {face:F2} m ahead (aim {FaceDistance:F2}), {room:F2} m of room before the conveyor edge";
                string why;
                if (m_Moves >= k_MaxMoves)
                {
                    Log.Warn($"[G1] approach: {state} - out of corrections, reaching from here");
                    EnterReady();
                    return;
                }
                if (Math.Abs(off) > AlignYawDeg * Mathf.Deg2Rad)
                {
                    m_ApTurnTo = YawOf(m_Fwd);
                    m_ApCmd = new Vector3(0f, 0f, Math.Sign(off) * k_TurnCmd);
                    m_ApTime = k_TurnMax;
                    why = $"turning in place {(off > 0f ? "left" : "right")}";
                }
                else if (Math.Abs(side) > AlignSide)
                {
                    m_ApCmd = new Vector3(0f, Math.Sign(side) * k_SideCmd, 0f);
                    m_ApTime = Math.Abs(side) > 0.2f ? k_LongBurst : k_Burst;
                    why = $"side-stepping {(side > 0f ? "left" : "right")} ({m_ApTime:F2} s burst)";
                }
                else if (face > FaceDistance + AlignForward && room > 0.1f)
                {
                    m_ApCmd = new Vector3(k_FwdCmd, 0f, 0f);
                    m_ApTime = k_Burst;
                    why = $"stepping forward ({m_ApTime:F2} s burst)";
                }
                else if (face < FaceDistance - AlignForward)
                {
                    m_ApCmd = new Vector3(-k_BackCmd, 0f, 0f);
                    m_ApTime = face < FaceDistance - 0.15f ? k_LongBurst : k_Burst;
                    why = $"stepping back ({m_ApTime:F2} s burst)";
                }
                else
                {
                    Log.Info($"[G1] lined up with the box after {m_Moves} corrections: {state}");
                    EnterReady();
                    return;
                }
                m_Moves++;
                Log.Info($"[G1] approach {m_Moves}/{k_MaxMoves}: {state} -> {why}");
                NextStep();
            }
            else if (m_Step == 1)
            {
                Drive(m_ApCmd.X, m_ApCmd.Y, m_ApCmd.Z);
                bool turned =
                    m_ApCmd.Z != 0f
                    && Math.Abs(Mathf.WrapToPi(m_ApTurnTo - YawOf(Heading()))) < k_TurnStop; // stop early: it coasts on
                if (m_StepTime >= m_ApTime || turned)
                    NextStep();
            }
            else if (m_StepTime >= k_ApCoast)
            {
                m_Step = 0; // coasted to a stop: measure again
                m_StepTime = 0f;
                ResetWatch();
            }
        }

        // How far the pelvis may still step toward the conveyor before it comes within EdgeClearance of its near edge.
        private float ChuteRoom() =>
            Vector3.Dot(ChuteEntity!.Transform.WorldTranslation - Pelvis(), FaceOf(ChuteEntity!))
            - 0.5f
                * Math.Min(ChuteEntity!.Transform.WorldScale.X, ChuteEntity!.Transform.WorldScale.Z)
            - EdgeClearance;

        // Grip frame squared to the box, then Ready: settle, plan the grip and check the reach, then reach (PreGrasp -> Squeeze).
        private void EnterReady()
        {
            SetGripFrame(Heading());
            m_Carrying = false;
            m_Lift = 0f;
            m_Boost = 1f;
            m_PickAt = m_BoxAt = BoxCenter(m_Box);
            SetSafeY();
            m_Slide = 0f;
            m_Holder = null;
            foreach (Arm a in m_Arms)
                a.Gap = ApproachGap;
            Advance(Phase.Ready);
        }

        // Farthest hand target of the reach (in front of the box and beside it) from its shoulder, as a share of the arm.
        private float ReachShare(out string which)
        {
            float worst = 0f;
            which = "";
            foreach (Arm a in m_Arms)
                for (int i = 0; i < 2; i++)
                {
                    float r =
                        ((i == 0 ? HandTarget(a) : GripPoint(a)) - BodyPos(a.Shoulder)).Length()
                        / k_ArmLength;
                    if (r <= worst)
                        continue;
                    worst = r;
                    which = $"{a.Name} hand {(i == 0 ? "in front of the box" : "beside the box")}";
                }
            return worst;
        }

        // Grip planned from the live shoulders, then the reach checked from where the robot stands: every hand target (pre-grip in front of the
        // box, and beside it) within PlanReach of its shoulder, grip points at least MinSide off the body's centre line (Approach has lined the body up as close as the conveyor edge allows), else the attempt
        // fails here (retried; the box is skipped after MaxBoxAttempts). Logs how the body stands to the box.
        private bool Reachable()
        {
            PlanGrip();
            m_Slide = -k_PreFront; // targets = pre-grip points (ReachShare checks them and the grip points)
            foreach (Arm a in m_Arms)
                a.Gap = ApproachGap + k_PreWiden;
            float side = float.MaxValue,
                up = 0f;
            foreach (Arm a in m_Arms)
            {
                side = Math.Min(
                    side,
                    a.Side * Vector3.Dot(Flat(GripPoint(a) - Pelvis()), LeftOf(Heading()))
                );
                up = Math.Max(up, GripPoint(a).Y - Pelvis().Y);
            }
            if (side < MinSide)
            {
                Fail(
                    $"box too narrow or off-centre for the arms: a grip point comes within {side:F2} m of the body's centre line (needs {MinSide:F2})"
                );
                return false;
            }
            if (up > MaxGripUp)
                Log.Warn(
                    $"[G1] grip points {up:F2} m over the pelvis (arms are accurate up to {MaxGripUp:F2}) - reaching anyway"
                );
            float off = Mathf.WrapToPi(YawOf(m_Fwd) - YawOf(Heading())),
                face = Vector3.Dot(Flat(m_Center - Pelvis()), m_Fwd) - m_HalfD,
                reach = ReachShare(out string which);
            string state =
                $"facing {off * Mathf.Rad2Deg:F0} deg off the box, front face {face:F2} m ahead, farthest hand target {reach * 100f:F0}% of the arm ({which})";
            if (reach > PlanReach)
            {
                Fail(
                    $"box out of reach from where the robot stands: {state} (limit {PlanReach * 100f:F0}%) - move the robot or the box, or lower EdgeClearance so it can stand closer"
                ); // retried; skipped after MaxBoxAttempts
                return false;
            }
            Log.Info($"[G1] body vs box: {state} -> reaching");
            return true;
        }

        // Waits for StillTime of unbroken stillness, saying so every k_StillLog s; after PhaseTimeout of waiting the attempt fails (no reach on
        // a moving body). True while still waiting (or just failed).
        private bool AwaitStill(float waited, string where)
        {
            if (m_Still >= StillTime)
                return false;
            if (waited > PhaseTimeout)
            {
                Fail(
                    $"body never settled {where}: {BodyMotion():F2} m/s at the hands after {waited:F0} s"
                );
                return true;
            }
            if ((int)(waited / k_StillLog) != (int)((waited - m_Dt) / k_StillLog))
                Log.Warn(
                    $"[G1] waiting for the body to settle {where}: {BodyMotion():F2} m/s at the hands, {waited:F0} s so far"
                );
            return true;
        }

        // Ready: once the body has been still for StillTime straight and every hand target is in reach (Reachable), both hands glide together
        // in one straight move from the fold to the pre-grip points - grip height, k_PreFront in front of the box, k_PreWiden wider - (PreGrasp).
        private void StepReady()
        {
            if (!Track(true))
                return;
            SendHands(Fold, false); // folded, body-relative, while settling
            if (AwaitStill(m_StepTime, "before reaching") || !Reachable())
                return; // arm moves on a swaying body make the walker step
            Log.Info(
                $"[G1] reach: {GripInfo()} | front-bottom corners L {Corner(1f)} R {Corner(-1f)} | palms touch at width + 2 x PalmOffset {2f * (m_HalfW + PalmOffset):F3} m"
            );
            GlideTo(HandMoveSpeed);
            Advance(Phase.PreGrasp);
        }
    }
}
