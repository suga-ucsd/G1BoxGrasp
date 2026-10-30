using System;
using Hazel;

namespace G1Pipe
{
    // Catch: the downhill hand waits in the sliding box's path, gives with it and centres it.
    public partial class G1BoxGrasp
    {
        // The downhill hand: the box slides toward it and stops against its palm.
        private Arm Stopper => Vector3.Dot(m_Downhill, m_Lat) > 0f ? m_Arms[0] : m_Arms[1];

        // A hand the catch keeps folded like while walking (smoothed fold point, wrist on the heading), so it stays still: the free hand, and the stopper until it goes.
        private bool Parked(Arm a) => m_Phase == Phase.Catch && (a != Stopper || m_Step == 0);

        // Catch pose: the stopper's palm on the stop spot of the box's downhill face (m_Center = the box centre there) once it goes; parked hands folded.
        private Vector3 CatchPose(Arm a) => Parked(a) ? Fold(a) : m_StopCmd;

        // The stopper's target: HandTarget with the palm centred on the box's mid-height (a hit above the centre of mass twists the box away),
        // its lower edge never under BeltClearance.
        private Vector3 StopTarget()
        {
            Arm s = Stopper;
            float up = Math.Max(m_HalfH + PalmBelow(s) - k_PalmHalf, BeltClearance + PalmBelow(s));
            return HandTarget(s) + Vector3.Up * (up - GripHeight(s));
        }

        // s the stopper takes to cover d to a still target: at CatchHandSpeed, plus its easing in.
        private float Travel(float d) => d / CatchHandSpeed + 2f * k_StopSmooth;

        // How far the box centre is past its stop spot along its path (- = still uphill of it).
        private float PastStop() =>
            Vector3.Dot(BoxCenter(m_Box) - m_Path, m_Downhill) - m_Ahead - m_Shift - m_Give;

        // Catch from the box's first frames: grip frame from the box and its path down the belt (through where it rests on the belt); the
        // robot catches from where it stands, so the belt must run across its front.
        private void EnterCatch()
        {
            m_Holder = null;
            SetGripFrame(Heading());
            m_Path = m_Center - m_ChuteN * ChuteGap(m_Box);
            m_Shift = m_Give = m_GiveV = m_PeakSpeed = m_PeakForce = m_PrevV = 0f;
            m_Accel = SlideAccel(); // frictionless until measured: the earliest the box can come
            m_Carrying = false;
            m_Lift = m_GripLift = 0f;
            m_Slide = -(FrontInset + k_CatchFront); // stopper waits in front of the path
            foreach (Arm a in m_Arms)
                a.Gap = a == Stopper ? 0f : ApproachGap;
            float across = Vector3.Dot(m_Downhill, LeftOf(Heading()));
            if (Math.Abs(across) < 0.5f)
            {
                Fail(
                    $"the belt doesn't run across the robot's front ({Mathf.Acos(Math.Abs(across)) * Mathf.Rad2Deg:F0} deg off) - turn the robot to face the conveyor",
                    false
                );
                return;
            }
            Log.Info(
                $"[G1] catching box {m_Box} at {F(m_Center)} ({BoxSpeed(m_Box):F2} m/s) with the {Stopper.Name} hand | {GripInfo()}"
            );
            Advance(Phase.Catch);
        }

        // Catch, every frame. Steps: 0 both hands folded while the body settles (the stop spot follows the pelvis: on the path with the palm
        // CatchPalmSide out from its centre line, so the arm's catch pose is the same for any box size), until it has been still for StillTime or the box is so close the stopper must go; 1 stopper to its waiting point in front of the
        // box's path (palm height, fingertips k_CatchFront clear of its front face); 2 slide in along the forward axis onto the stop spot; 3 in
        // place (contact tared); 4 give; 5 caught (then pushed up the belt, at most k_CentreMax, until its centre is on the body's centre line, whatever its size). From step 1 the spot is fixed in the world, except that once the stopper is at its waiting
        // point (and only there) it moves down the belt (never back up, at most CatchShiftMax) to k_CatchClear past where the box will be when the
        // palm gets there - predicted from its speed and its measured acceleration; the slide only starts with time to finish before the box
        // (+ k_SlideSpare), and from then the spot is fixed, so the palm only moves straight forward into the box's path, never along it.
        // Give: over the box's last k_GiveMatch before it reaches the palm, the spot speeds up so the gap closes at a steadily falling rate (they meet
        // with almost no speed between them); once the box is on the palm (contact, or within k_GiveTouch) it brakes to a stop over CatchGive, so the box is slowed by a
        // yielding palm instead of bouncing off a stiff one. The palm is centred on the box's mid-height (StopTarget). Caught once the give is over
        // and the box stays slower than CatchSpeed for CatchTime within k_CatchZone of the spot - or of the real palm, when the arm can't
        // quite reach its spot (from step 2 on: a box already resting on a late palm counts too); then it stays held there. Fails if it gets
        // past the stopper, falls off the belt, or never arrives.
        private void StepCatch(float ts)
        {
            Arm s = Stopper;
            Vector3 c = BoxCenter(m_Box);
            m_Mujoco!.GetBodyPoseMj(m_Boxes[m_Box].Body, out _, out _, out Vector3 vel, out _);
            float v = Vector3.Dot(m_MjToHz * vel, m_Downhill),
                speed = BoxSpeed(m_Box);
            ReadForces();
            LogContacts("CA");
            bool touch = m_Step >= 3 && s.Force + s.ThumbForce > TouchForce; // tared from step 3 on
            if (!touch && Math.Abs(ChuteGap(m_Box)) < 0.01f)
                m_Accel = Mathf.Clamp(
                    m_Accel
                        + ((v - m_PrevV) / ts - m_Accel)
                            * (1f - (float)Math.Exp(-ts / k_AccelSmooth)),
                    0f,
                    SlideAccel()
                ); // sliding freely on the belt: learn its acceleration
            m_PrevV = v;
            Vector3 goal = StopTarget(),
                spot = goal - m_Fwd * m_Slide; // the stopper's target now; its palm on the stop spot
            float err = (SiteWorld(s.Site) - goal).Length(),
                slide = Travel((spot - goal).Length()), // s the slide in takes
                hand =
                    (
                        m_Step <= 1
                            ? (err <= ReachTolerance ? 0f : Travel(err)) + slide
                            : Travel((SiteWorld(s.Site) - spot).Length())
                    ) + k_CatchLead; // s until the palm is on the stop spot
            if (m_Step == 0)
            {
                Vector3 left = LeftOf(Heading());
                m_Ahead =
                    (
                        Vector3.Dot(Pelvis() - m_Path, left)
                        + s.Side * (CatchPalmSide - m_HalfW - PalmOffset)
                    ) / Vector3.Dot(m_Downhill, left);
            }
            else if (m_Step == 1 && err <= ReachTolerance)
            {
                float then =
                    Vector3.Dot(c - m_Path, m_Downhill)
                    - m_Ahead
                    + v * hand
                    + 0.5f * m_Accel * hand * hand; // box centre past straight ahead when the palm gets there
                m_Shift = Math.Min(Math.Max(m_Shift, then + k_CatchClear), CatchShiftMax);
            }
            if (m_Step == 3)
            {
                float gap = Math.Max(0f, -PastStop()); // box centre short of the spot = its face short of the palm
                m_GiveV =
                    Math.Max(0f, v) * (1f - (float)Math.Sqrt(Math.Min(1f, gap / k_GiveMatch))); // relative speed falls at a steady rate: meets the box at its speed
                m_Give += m_GiveV * ts;
            }
            else if (m_Step == 4)
            {
                m_Give += m_GiveV * ts;
                m_GiveV = Math.Max(0f, m_GiveV - m_GiveDecel * ts);
            }
            m_Center = m_Path + m_Downhill * (m_Ahead + m_Shift + m_Give);
            float past = PastStop(),
                a = Math.Max(m_Accel, 1e-3f),
                eta = past >= 0f
                    ? 0f
                    : (-v + (float)Math.Sqrt(Math.Max(0f, v * v - 2f * a * past))) / a; // s until the box is at the spot
            if (m_Step == 0 && (m_Still >= StillTime || eta <= hand + k_CatchMargin))
            {
                Log.Info(
                    $"[G1] {s.Name} hand going: {(m_Still >= StillTime ? $"body still for {m_Still:F1} s" : $"box getting close (body still only {m_Still:F1} s)")}, box {-past:F3} m short of the stop spot, there in ~{eta:F2} s, the palm needs ~{hand:F2} s | stop spot fixed at {F(m_Center)}"
                );
                m_StopCmd = s.Cmd; // tracking starts from where the parked hand's command is
                s.T = s.Tau = 0f;
                s.Rising = false;
                NextStep();
            }
            else if (
                m_Step == 1
                && err <= ReachTolerance
                && eta > slide + k_CatchLead + k_SlideSpare
            ) // the slide in only starts with time to finish before the box comes
            {
                Log.Info(
                    $"[G1] {s.Name} hand in front of the box's path ({err:F3} m off) -> sliding in, box {-past:F3} m short of the stop spot, there in ~{eta:F2} s"
                );
                m_Slide = 0f;
                NextStep();
            }
            else if (m_Step == 2 && err <= ReachTolerance)
            {
                Tare();
                Log.Info(
                    $"[G1] {s.Name} palm in the box's path ({err:F3} m off the stop spot, {m_Shift:F3} m down the belt from straight ahead), box {-past:F3} m short, there in ~{eta:F2} s"
                );
                NextStep();
            }
            else if (m_Step == 3 && (touch || past >= -k_GiveTouch))
            {
                m_GiveDecel = m_GiveV * m_GiveV / (2f * Math.Max(CatchGive, 0.005f));
                Log.Info(
                    $"[G1] give: box on the {s.Name} palm ({(touch ? $"{s.Force + s.ThumbForce:F1} N" : $"{-past:F3} m short of it")}) at {v:F3} m/s, palm already moving {m_GiveV:F3} m/s after {m_Give:F3} m -> braking over {CatchGive:F3} m ({m_GiveDecel:F1} m/s^2)"
                );
                NextStep();
            }
            // The stopper's IK command tracks its target continuously - at up to CatchHandSpeed, easing in over k_StopSmooth near it, so a
            // moved target (sway, shift, slide in) is never a fresh fixed-length move; in the give it moves with the target directly.
            if (m_Step >= 1)
            {
                Vector3 d = StopTarget() - m_StopCmd;
                float len = d.Length(),
                    move = m_GiveV > 0f
                        ? len
                        : Math.Min(
                            len * (1f - (float)Math.Exp(-ts / k_StopSmooth)),
                            CatchHandSpeed * ts
                        );
                if (len > 1e-6f)
                    m_StopCmd += d * (move / len);
            }
            SendHands(CatchPose, false);
            m_PeakSpeed = Math.Max(m_PeakSpeed, speed);
            m_PeakForce = Math.Max(m_PeakForce, m_Step >= 3 ? s.Force + s.ThumbForce : 0f);
            if (PreGraspLogInterval > 0f && (m_DbgTimer += ts) >= PreGraspLogInterval)
            {
                m_DbgTimer = 0f;
                Log.Info(
                    $"[G1] CA t={m_PhaseTime:F2} step {m_Step} | box {F(c)} {v:F3} m/s down the belt (accel {m_Accel:F3} m/s^2), {-past:F3} m short of the stop spot, there in ~{eta:F2} s, belt gap {ChuteGap(m_Box):F3} m"
                        + $" | stop spot {F(m_Center)}: shift {m_Shift:F3} give {m_Give:F3} m ({m_GiveV:F3} m/s) | {s.Name} {err:F3} m off its target (palm on the spot in ~{hand:F2} s), reach {(SiteWorld(s.Site) - BodyPos(s.Shoulder)).Length() / k_ArmLength * 100f:F0}% / target {(goal - BodyPos(s.Shoulder)).Length() / k_ArmLength * 100f:F0}% of the arm{(s.Capped ? " (capped)" : "")}"
                        + $" | force palm {s.PalmForce:F1} hand {s.Force:F1} thumb {s.ThumbForce:F1} N | pelvis {F(Pelvis())} yaw {YawOf(Heading()) * Mathf.Rad2Deg:F0} deg"
                );
            }
            if (m_Step == 5)
            {
                float off =
                        s.Side * Vector3.Dot(Flat(BoxCenter(m_Box) - Pelvis()), LeftOf(Heading())), // box centre off the body's centre line, on the stopper's side
                    need = Math.Min(off, k_CentreMax + m_Give - m_Caught);
                if (need > k_Centred)
                {
                    if (m_Give == m_Caught)
                        Log.Info(
                            $"[G1] box centre {off:F3} m off the body's centre line -> {s.Name} hand pushes it up the belt to the centre"
                        );
                    m_Give -= Math.Min(k_CentreSpeed * ts, need);
                    m_StepTime = 0f;
                    return;
                }
                if (m_StepTime <= 1.5f * ts && m_Give != m_Caught)
                    Log.Info(
                        $"[G1] box centred after a {m_Caught - m_Give:F3} m push: centre {off:F3} m off the body's centre line"
                    );
                if (!AwaitStill(m_StepTime, "before grasping the caught box"))
                    EnterGrasp();
                return; // caught: pushed to the centre, held there until the body is still, then grasp
            }
            if (past > m_HalfW || ChuteGap(m_Box) < -0.05f || m_PhaseTime > PhaseTimeout)
            {
                Fail(
                    past > m_HalfW
                        ? $"box got past the {s.Name} hand ({past:F3} m beyond the stop spot, step {m_Step}{(m_Step <= 1 ? " - too late to slide the palm in" : "")}, hand {err:F3} m off its target)"
                    : ChuteGap(m_Box) < -0.05f ? "box fell off the belt"
                    : $"box never stopped at the {s.Name} hand: {-past:F3} m short of the stop spot, {Vector3.Dot(SiteWorld(s.Site) - c, m_Downhill) - PalmOffset - m_HalfW:F3} m off the real palm, {speed:F3} m/s after {m_PhaseTime:F0} s"
                );
                return;
            }
            float palmGap = Vector3.Dot(SiteWorld(s.Site) - c, m_Downhill) - PalmOffset - m_HalfW; // box's downhill face short of the real palm (the arm may not reach its spot)
            bool atHand = past >= -k_CatchZone || palmGap <= k_CatchZone;
            if (m_Step < 2 || m_GiveV > 0f || speed > CatchSpeed || !atHand)
                m_StepTime = 0f; // from here m_StepTime = time stopped at the hand (after the give, or resting on a palm that got there late)
            if (m_StepTime < CatchTime)
                return;
            Log.Info(
                $"[G1] caught box {m_Box} after {m_PhaseTime:F2} s{(m_Step < 4 ? $" (at rest on the palm in step {m_Step}, before the give)" : "")}: {-past:F3} m short of the stop spot ({palmGap:F3} m off the real palm), came at up to {m_PeakSpeed:F2} m/s, gave {m_Give:F3} m, hand force up to {m_PeakForce:F1} N, now {s.Force:F1} N | holding it there for the grasp"
            );
            m_Caught = m_Give;
            NextStep();
        }
    }
}
