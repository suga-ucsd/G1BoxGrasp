using System;
using Hazel;

namespace G1Pipe
{
    // Grip planning, PreGrasp (hands in front, slide in) and Squeeze.
    public partial class G1BoxGrasp
    {
        // Grip height from the live shoulders: raised up the box until each grasp site is within GripComfort of the arm's reach, up to TopMargin
        // under its top; if even that isn't enough, says by how much the box sits too low. Logs the plan.
        private void PlanGrip()
        {
            m_GripLift = 0f;
            float reach = GripComfort / 100f * k_ArmLength,
                need = 0f,
                maxLift = MaxGripLift();
            foreach (Arm a in m_Arms)
            {
                Vector3 d = GripPoint(a) - BodyPos(a.Shoulder);
                float across = Flat(d).Length();
                need = Math.Max(
                    need,
                    -d.Y - (float)Math.Sqrt(Math.Max(0f, reach * reach - across * across))
                ); // shoulder height over the grip beyond what the arm covers
            }
            m_GripLift = Math.Min(need, maxLift);
            Log.Info(
                $"[G1] reach plan: shoulders {BodyPos(m_Arms[0].Shoulder).Y:F2} m, grip L {GripPoint(m_Arms[0]).Y:F2} R {GripPoint(m_Arms[1]).Y:F2} m (box {m_Center.Y - m_HalfH:F2}-{m_Center.Y + m_HalfH:F2}), grip raised {m_GripLift:F3} m"
            );
            if (need > maxLift + 0.005f)
                Log.Warn(
                    $"[G1] box sits {need - maxLift:F2} m too low for the arms even gripping just under its top - raise the belt by that much (reaching anyway)"
                );
        }

        // Grasp-site target beside the box at grip height (palm ApproachGap clear, not raised, not lifted).
        private Vector3 GripPoint(Arm a) =>
            m_Center
            + m_Lat * (a.Side * (m_HalfW + PalmOffset + ApproachGap))
            + m_Fwd * (FrontInset - m_HalfD)
            + Vector3.Up * (GripHeight(a) - m_HalfH);

        // How far the grip can still rise before a grasp site comes within TopMargin of the box top.
        private float MaxGripLift() =>
            2f * m_HalfH
            - TopMargin
            - Math.Max(GripHeight(m_Arms[0]), GripHeight(m_Arms[1]))
            + m_GripLift;

        // A hand is beside the box, judged from its live grasp site in the box frame: palm outside its side face by no more than the closing
        // range, thumb within 3 cm of the front face, palm overlapping the side face above the belt. Outputs those three numbers for the log.
        private bool Placed(Arm a, out float side, out float front, out float up)
        {
            Vector3 p = SiteWorld(a.Site) - m_Center;
            side = a.Side * Vector3.Dot(p, m_Lat) - m_HalfW - PalmOffset;
            front = Vector3.Dot(p, m_Fwd) + m_HalfD - FrontInset;
            up = p.Y + m_HalfH;
            return side > -PlacedInside
                && side < ApproachGap + k_Widen
                && Math.Abs(front) < PlacedFront
                && up > PalmBelow(a)
                && up < 2f * m_HalfH - PlacedTop;
        }

        private bool BothPlaced() =>
            Placed(m_Arms[0], out _, out _, out _) && Placed(m_Arms[1], out _, out _, out _);

        // Inner loop, every frame of the reach: re-reads the box so the hands follow it (nudged, sliding) while follow is set, logs each
        // disturbance, and fails the attempt (next box) once the box has gone.
        private bool Track(bool follow)
        {
            Vector3 c = BoxCenter(m_Box);
            float gone = Flat(c - m_PickAt).Length(),
                moved = (c - m_BoxAt).Length();
            if (gone > LostDistance || ChuteGap(m_Box) > 0.05f)
            {
                Fail(
                    $"box left the grasp: {gone:F2} m from where the reach started, {ChuteGap(m_Box):F2} m off the conveyor"
                );
                return false;
            }
            if (moved > k_Disturb)
            {
                Log.Warn(
                    $"[G1] box moved {moved:F3} m (now at {c}){(follow ? " -> hands follow it" : " - hands already touching, holding the frame")}"
                );
                m_BoxAt = c;
            }
            if (!follow)
                return true;
            Vector3 c0 = m_Center,
                f0 = m_Fwd,
                g0 = HandTarget(m_Arms[0]),
                g1 = HandTarget(m_Arms[1]);
            SetGripFrame(m_Fwd);
            float dc = (m_Center - c0).Length(),
                turn = Mathf.Acos(Mathf.Clamp(Vector3.Dot(m_Fwd, f0), -1f, 1f)) * Mathf.Rad2Deg;
            if (m_Phase == Phase.PreGrasp && (dc > 0.002f || turn > 0.5f))
                Log.Warn(
                    $"[G1] PG frame re-read: box centre moved {dc:F3} m {F(m_Center - c0)}, forward turned {turn:F1} deg -> hand goals moved L {(HandTarget(m_Arms[0]) - g0).Length():F3} m, R {(HandTarget(m_Arms[1]) - g1).Length():F3} m (step {m_Step})"
                );
            return true;
        }

        // Caught -> grasp. The stopper becomes the holder: its target is pinned where its IK command is now (HoldHere), so it doesn't move -
        // later only its Gap (the squeeze), the lift and the carry move it. The other hand comes in as in any grasp: the grip frame from the
        // box as it rests on the holder, its pre-grip point in front of the box, its palm level with the holder's - or, on a box too narrow
        // for the straight thumbs to pass (under 2 x k_ThumbReach), 2 x k_ThumbStagger over it - and always all on the face, BeltClearance
        // over the box bottom and under its top; checked within PlanReach (a grip near the body's centre line only warns); then it glides
        // there from the fold and PreGrasp slides it in.
        private void EnterGrasp()
        {
            Arm h = Stopper,
                r = h == m_Arms[0] ? m_Arms[1] : m_Arms[0];
            m_Holder = h;
            SetGripFrame(m_Fwd);
            m_Carrying = false;
            m_Lift = 0f;
            m_Boost = 1f;
            m_PickAt = m_BoxAt = BoxCenter(m_Box);
            m_Reach = StandDistance;
            m_SqueezeTries = m_WalkRetries = 0;
            SetSafeY();
            h.Gap = 0f;
            r.Gap = ApproachGap + k_PreWiden;
            m_Slide = -k_PreFront;
            float bottom = m_Center.Y - m_HalfH,
                lowest = BeltClearance + k_PalmHalf, // palm centre range over the box bottom that keeps the palm all on the face
                palm = Mathf.Clamp(
                    h.Cmd.Y
                        - PalmBelow(h)
                        + k_PalmHalf
                        - bottom
                        + (m_HalfW < k_ThumbReach ? 2f * k_ThumbStagger : 0f),
                    lowest,
                    Math.Max(lowest, 2f * m_HalfH - k_PalmHalf)
                ), // reaching palm's centre over the box bottom
                low = BeltClearance + PalmBelow(r) + (1f + r.Side) * k_ThumbStagger; // its grip height with no raise
            m_GripLift = Math.Max(0f, palm + PalmBelow(r) - k_PalmHalf - low);
            HoldHere();
            Vector3 sh = BodyPos(r.Shoulder);
            float reach =
                    Math.Max((HandTarget(r) - sh).Length(), (GripPoint(r) - sh).Length())
                    / k_ArmLength,
                side = r.Side * Vector3.Dot(Flat(GripPoint(r) - Pelvis()), LeftOf(Heading()));
            if (reach > PlanReach)
            {
                Fail(
                    $"caught box out of the {r.Name} hand's reach: {reach * 100f:F0}% of the arm (limit {PlanReach * 100f:F0}%)"
                );
                return;
            }
            if (side < MinSide)
                Log.Warn(
                    $"[G1] {r.Name} grip point {side:F2} m off the body's centre line (MinSide {MinSide:F2}) - the box stopped off-centre, reaching anyway"
                );
            Log.Info(
                $"[G1] grasping the caught box: {h.Name} hand holds still, {r.Name} hand reaches in ({reach * 100f:F0}% of the arm, grip {GripHeight(r):F3} m over the box bottom, raised {m_GripLift:F3} m) | {GripInfo()}"
            );
            GlideTo(HandMoveSpeed);
            Advance(Phase.PreGrasp);
        }

        // Pins the holding hand where its IK command is now: from here its target is that point in the grip frame, moved only by its Gap,
        // the lift and the carry (HandTarget adds this offset).
        private void HoldHere()
        {
            Arm h = m_Holder!;
            m_HoldOff = new Vector3(0f, 0f, 0f);
            Vector3 d = h.Cmd - HandTarget(h);
            m_HoldOff = new Vector3(Vector3.Dot(d, m_Lat), Vector3.Dot(d, m_Fwd), d.Y);
        }

        // Both hands start one straight min-jerk move from their current IK targets to HandTarget, both taking the time the longer one needs at
        // speed (they arrive together); SendHands then runs it - no path planning, so nothing climbs over the box.
        private void GlideTo(float speed)
        {
            float t = k_MinSegment;
            foreach (Arm a in m_Arms)
                t = Math.Max(t, (HandTarget(a) - a.Cmd).Length() / speed);
            foreach (Arm a in m_Arms)
            {
                a.From = a.Cmd;
                a.To = HandTarget(a);
                a.T = t;
                a.Tau = 0f;
                a.Rising = false;
            }
        }

        // At the pre-grip points (in front of the box, once the body is still) contact sensing is tared and both hands slide in together along the
        // side faces to the grip points, ApproachGap clear of them (a hand that lands on the box slides outward). Grasp (Squeeze) once both are
        // within ReachTolerance - or beside the box faces (Placed) and no longer improving. Hands that stall short adapt (GraspOutOfReach).
        private void StepPreGrasp(float ts)
        {
            if (!Track(m_Holder == null)) // a held box stays put: following it would let the holding palm chase it
                return;
            SendHands(HandTarget, false);
            LogPreGrasp(ts);
            if (Gliding())
            {
                ResetWatch();
                return;
            } // on its way along the planned path: that's progress
            ReadForces();
            if (m_Step == 1)
                foreach (Arm a in m_Arms)
                    if (a != m_Holder && a.Force > TouchForce)
                    {
                        if (a.Gap <= ApproachGap)
                            Log.Warn(
                                $"[G1] PG {a.Name} landed on the box ({a.Force:F1} N) -> sliding out, gap widening up to {ApproachGap + k_Widen:F3} m"
                            );
                        a.Gap = Math.Min(a.Gap + k_OpenSpeed * ts, ApproachGap + k_Widen);
                    }
            float err = Math.Max(HandError(m_Arms[0]), HandError(m_Arms[1]));
            bool stalled = err > ReachTolerance && Stalled(err, ReachProgress, PhaseTimeout);
            if (
                m_Step == 1
                && (err <= ReachTolerance || (BothPlaced() && m_SinceBest > PlacedWait))
            )
            {
                if (err > ReachTolerance)
                    Log.Info(
                        $"[G1] hands {err:F3} m from their targets but beside the box faces - grasping"
                    );
                Log.Info(
                    $"[G1] PG end of the slide in after {m_StepTime:F2} s: err L {HandError(m_Arms[0]):F3} R {HandError(m_Arms[1]):F3} m, placed L {Placed(m_Arms[0], out _, out _, out _)} R {Placed(m_Arms[1], out _, out _, out _)}"
                );
                foreach (Arm a in m_Arms)
                    a.PalmOn = false;
                Advance(Phase.Squeeze);
                return;
            }
            if (err > ReachTolerance)
            {
                if (stalled)
                    GraspOutOfReach();
                return;
            }
            if (AwaitStill(m_WatchTime, "in front of the box"))
                return; // hold in front of the box until the body stops swaying
            Log.Info(
                $"[G1] PG in front of the box after {m_StepTime:F2} s (err L {HandError(m_Arms[0]):F3} R {HandError(m_Arms[1]):F3} m, body still {m_Still:F1} s, {BodyMotion():F3} m/s at the hands) -> tare, slide in along the side faces"
            );
            Tare();
            m_Slide = 0f;
            foreach (Arm a in m_Arms)
                a.Gap = a == m_Holder ? 0f : ApproachGap;
            GlideTo(k_SlideSpeed);
            NextStep();
        }

        // Grasp: the thumbs swing onto the front face and index + middle curl into the side faces while the arms close in, each hand pushing
        // toward the other, until both palms have reached their faces and each hand holds the required force for GripConfirm s (the hands follow the box until the first touch). A grip that
        // stops building with a hand at its squeeze limit (palm gap and finger curl both maxed) is reopened and re-centred (MaxSqueezeTries).
        private void StepSqueeze(float ts)
        {
            if (!Track(m_Holder == null && Contact() < TouchForce))
                return;
            SendHands(HandTarget, false);
            float req = RequiredForce(),
                need = req * GripForceLimit / 100f;
            Squeeze(ts, req);
            LogGrip(ts);
            bool stuck = false;
            foreach (Arm a in m_Arms)
            {
                bool held = a.PalmOn && a.Force >= need; // palm on its face; palm + fingers hold GripForceLimit of the required force
                if (!held)
                    m_StepTime = 0f;
                stuck |= !held && a.Gap <= 0.001f - MaxSqueezeDepth;
            }
            if (!stuck)
                m_SinceBest = 0f; // still closing or holding: that's progress
            if (m_StepTime >= k_GripConfirm)
            {
                Log.Info(
                    $"[G1] grip: palm L={m_Arms[0].PalmForce:F1} R={m_Arms[1].PalmForce:F1} N (need {need:F1} of {req:F1} N), fingers L={m_Arms[0].Force - m_Arms[0].PalmForce:F1} R={m_Arms[1].Force - m_Arms[1].PalmForce:F1} N, thumb L={m_Arms[0].ThumbForce:F1} R={m_Arms[1].ThumbForce:F1} N, curl L={m_Arms[0].Curl:F2} R={m_Arms[1].Curl:F2} rad, gap L={m_Arms[0].Gap:F3} R={m_Arms[1].Gap:F3} m"
                );
                m_Y0 = PalmMidY() - BoxCenter(m_Box).Y;
                m_Slot = m_Center - Vector3.Up * m_HalfH; // pick spot (bottom centre): where the box goes back when not carried to the table
                bool far = GripAhead() > k_DragFrom;
                if (far && LiftBox)
                    Log.Info(
                        $"[G1] gripped {GripAhead():F2} m ahead -> dragging the box in to {k_DragTo:F2} m (elbows bent for the lift)"
                    );
                Advance(
                    LiftBox ? (far ? Phase.DragIn : Phase.Lift)
                    : PlaceBox ? Phase.Release
                    : Phase.Holding
                );
            }
            else if (m_SinceBest > StallTime || m_WatchTime > PhaseTimeout)
                RetrySqueeze(need);
        }

        // Grip didn't build: reopen (fingers ease straight, thumbs back), re-measure the box and re-centre the hands beside it, then grasp again;
        // fail after MaxSqueezeTries.
        private void RetrySqueeze(float req)
        {
            string why =
                $"palm L={m_Arms[0].PalmForce:F1} R={m_Arms[1].PalmForce:F1} N of {req:F1} N (fingers L={m_Arms[0].Force - m_Arms[0].PalmForce:F1} R={m_Arms[1].Force - m_Arms[1].PalmForce:F1} N), gap L={m_Arms[0].Gap:F3} R={m_Arms[1].Gap:F3} m";
            if (++m_SqueezeTries >= MaxSqueezeTries)
            {
                Fail($"grip too weak after {MaxSqueezeTries} squeezes: {why}");
                return;
            }
            Log.Warn(
                $"[G1] {why} -> reopening and re-centring on the box ({m_SqueezeTries}/{MaxSqueezeTries - 1})"
            );
            SetGripFrame(m_Fwd);
            foreach (Arm a in m_Arms)
            {
                a.Gap = a == m_Holder ? 0f : ApproachGap; // a holding hand stays on its face
                a.PalmOn = false;
            }
            if (m_Holder != null)
                HoldHere(); // re-pinned to the re-read frame where it is now
            Advance(Phase.PreGrasp);
            NextStep(); // straight to "beside the box": the hands just open and re-centre at grip height
            SendHands(HandTarget, true); // after the phase change, so the reopen eases along a path instead of jumping like a grip target
        }

        // Hands stopped short: grip higher up the box if the worse hand is below its target and there's box left; else fail, stating the shortfall.
        private void GraspOutOfReach()
        {
            Arm w = HandError(m_Arms[0]) >= HandError(m_Arms[1]) ? m_Arms[0] : m_Arms[1];
            Vector3 miss = HandTarget(w) - SiteWorld(w.Site);
            float along = Vector3.Dot(Flat(miss), m_Fwd),
                below = -miss.Y,
                room = MaxGripLift() - m_GripLift;
            string why =
                $"{w.Name} hand short by {miss.Length():F3} m ({along:F3} along the grip axis, {below:F3} below)";
            ResetWatch();
            if (below > 0.01f && room > 0.005f)
            {
                float lift = Math.Min(below, room);
                m_GripLift += lift;
                Log.Warn($"[G1] {why} -> gripping {m_GripLift:F3} m higher up the box"); // the hand goals jump up by this much
                return;
            }
            Fail(
                $"{why}: no higher grip left (grip raised {m_GripLift:F3} m) - raise the belt or move the robot"
            );
        }
    }
}
