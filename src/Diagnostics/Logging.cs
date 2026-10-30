using System;
using System.Collections.Generic;
using Hazel;

namespace G1Pipe
{
    // Log lines: status, grip and pre-grasp traces, contacts, arm and wrist angles.
    public partial class G1BoxGrasp
    {
        // PreGrasp diagnostics. Every frame: each hand geom starting or ending contact (body, raw and tared force), and the sim falling behind
        // real time. Every PreGraspLogInterval s of sim: one body line (sim speed, pelvis, box) and one line per hand (path segment, IK command
        // vs goal vs actual grasp site, tracking lag, reach, placement in the box frame, forces, arm joints and how far they moved).
        private void LogPreGrasp(float ts)
        {
            LogContacts("PG");
            m_DbgSim += ts;
            if (PreGraspLogInterval <= 0f || (m_DbgTimer += ts) < PreGraspLogInterval)
                return;
            float wall = (float)m_Wall.Elapsed.TotalSeconds,
                speed = m_DbgSim / Math.Max(wall, 1e-4f);
            m_Wall.Restart();
            m_DbgTimer = m_DbgSim = 0f;
            m_Mujoco!.GetBodyPoseMj(m_PelvisId, out _, out _, out Vector3 pv, out Vector3 pw);
            m_Mujoco.GetBodyPoseMj(
                m_Boxes[m_Box].Body,
                out _,
                out _,
                out Vector3 bv,
                out Vector3 bw
            );
            Vector3 c = BoxCenter(m_Box);
            Log.Info(
                $"[G1] PG t={m_PhaseTime:F2} step {m_Step} ({(m_Step == 0 ? "to the pre-grip points" : "sliding in")}) | sim {speed:F2}x real time ({wall * 1000f:F0} ms wall)"
                    + $" | pelvis {F(Pelvis())} v {F(m_MjToHz * pv)} ({pv.Length():F3} m/s) yaw {YawOf(Heading()) * Mathf.Rad2Deg:F1} deg, ang vel {pw.Length():F3} rad/s | walk cmd {F(m_Cmd)}"
                    + $" | box {F(c)} v {bv.Length():F3} m/s, ang vel {bw.Length():F3} rad/s, {(c - m_PickAt).Length():F3} m from the reach start, {(c - m_BoxAt).Length():F3} m from the last re-read, belt gap {ChuteGap(m_Box):F3} m, contact {m_Mujoco.GetGeomContactForce(m_Boxes[m_Box].Geom):F1} N"
                    + $" | grip frame centre {F(m_Center)} fwd {F(m_Fwd)}, grip lift {m_GripLift:F3} m{(speed < 0.5f ? " | SIM SLOW" : "")}"
            );
            foreach (Arm a in m_Arms)
            {
                Vector3 site = SiteWorld(a.Site),
                    goal = HandTarget(a),
                    sv = m_MjToHz * m_Mujoco.GetSiteLinearVelocityMj(a.Site),
                    sh = BodyPos(a.Shoulder);
                bool placed = Placed(a, out float side, out float front, out float up);
                float[] q = Array.ConvertAll(a.Joints, j => (float)j.Value);
                float jump = 0f;
                for (int i = 0; i < Math.Min(q.Length, a.LoggedQ.Length); i++)
                    jump = Math.Max(jump, Math.Abs(q[i] - a.LoggedQ[i]));
                a.LoggedQ = q;
                var touching = new List<string>();
                for (int i = 0; i < a.Geoms.Length; i++)
                    if (a.Touch[i])
                        touching.Add(a.GeomName[i]);
                Log.Info(
                    $"[G1] PG   {a.Name}: {(a.Direct ? "tracking the goal directly" : $"segment {a.Tau:F2}/{a.T:F2} s {F(a.From)} -> {F(a.To)}{(a.Rising ? " (rising, wrist held)" : "")}")}"
                        + $" | IK cmd {F(a.Cmd)}, goal {F(goal)}, site {F(site)} v {sv.Length():F3} m/s | lag site->cmd {(a.Cmd - site).Length():F3} m, site->goal {(goal - site).Length():F3} m, cmd->goal {(goal - a.Cmd).Length():F3} m"
                        + $" | reach: site {(site - sh).Length() / k_ArmLength * 100f:F0}% / cmd {(a.Cmd - sh).Length() / k_ArmLength * 100f:F0}% of the arm"
                        + $" | box frame: palm {side:F3} out, thumb {front:F3} fwd, site {up:F3} up{(placed ? " (placed)" : "")}, gap {a.Gap:F3} m, slide {m_Slide:F3} m"
                        + $" | force palm {a.Force:F1} thumb {a.ThumbForce:F1} N, touching [{string.Join(", ", touching)}] | palm {a.PalmErr:F0} deg off"
                        + $" | joints deg [{string.Join(", ", Array.ConvertAll(q, v => (v * Mathf.Rad2Deg).ToString("F0")))}], max change {jump * Mathf.Rad2Deg:F1} deg since the last line"
                        + $" | {WristLog(a)}"
                );
            }
        }

        private static string F(Vector3 v) => $"({v.X:F3}, {v.Y:F3}, {v.Z:F3})";

        // Each hand geom starting or ending contact (body, raw and tared force), with the step, grasp site and box.
        private void LogContacts(string tag)
        {
            foreach (Arm a in m_Arms)
                for (int i = 0; i < a.Geoms.Length; i++)
                {
                    float raw = m_Mujoco!.GetGeomContactForce(a.Geoms[i]);
                    bool touch = raw - a.Base[i] > 0.5f;
                    if (touch != a.Touch[i])
                        Log.Warn(
                            $"[G1] {tag} contact {(touch ? "START" : "end")}: {a.GeomName[i]} raw {raw:F1} N (tare {a.Base[i]:F1}) | {m_Phase} step {m_Step}, site {F(SiteWorld(a.Site))}, box {F(BoxCenter(m_Box))}"
                        );
                    a.Touch[i] = touch;
                }
        }

        // Grip diagnostics (Squeeze, Lift, carry, Place): contact events every frame; every PreGraspLogInterval s of sim one body line (sim
        // speed, required force, pelvis, walk command, box, slip) and one line per hand: palm / finger / thumb force, gap target, the palm's
        // measured distance to its face, how much of the palm's height lies on the face, curl, palm tilt, touching geoms.
        private void LogGrip(float ts)
        {
            LogContacts("GR");
            m_DbgSim += ts;
            if (PreGraspLogInterval <= 0f || (m_DbgTimer += ts) < PreGraspLogInterval)
                return;
            float wall = (float)m_Wall.Elapsed.TotalSeconds,
                speed = m_DbgSim / Math.Max(wall, 1e-4f);
            m_Wall.Restart();
            float dt = Math.Max(m_DbgSim, 1e-3f); // sim s since the last line (joint rates, site velocity)
            m_DbgTimer = m_DbgSim = 0f;
            m_Mujoco!.GetBodyPoseMj(m_PelvisId, out _, out _, out Vector3 pv, out Vector3 pw);
            m_Mujoco.GetBodyPoseMj(m_TorsoId, out _, out _, out _, out Vector3 tw);
            m_Mujoco.GetBodyPoseMj(
                m_Boxes[m_Box].Body,
                out _,
                out _,
                out Vector3 bv,
                out Vector3 bw
            );
            Vector3 c = BoxCenter(m_Box),
                lat = m_Carrying ? BodyDir(m_Lat) : m_Lat;
            string slip =
                m_Phase == Phase.Squeeze
                    ? "not lifted"
                    : $"{PalmMidY() - c.Y - m_Y0:F3} m (+ = box sliding down)";
            Log.Info(
                $"[G1] GR {m_Phase} t={m_PhaseTime:F2} | sim {speed:F2}x real time | need {RequiredForce():F1} N per palm (boost {m_Boost:F2}), lift {m_Lift:F3} m"
                    + $" | pelvis {F(Pelvis())} {pv.Length():F3} m/s, ang vel {pw.Length():F3} rad/s, yaw {YawOf(Heading()) * Mathf.Rad2Deg:F1} deg | walk cmd {F(m_Cmd)}"
                    + $" | turn rate (roll, yaw, pitch) pelvis {F(ToBody(m_MjToHz * pw))} torso {F(ToBody(m_MjToHz * tw))} rad/s, waist (yaw, roll, pitch) [{Degs(m_WaistAdr)}] deg"
                    + $" | box {F(c)} v {bv.Length():F3} m/s, ang vel {F(ToBody(m_MjToHz * bw))} rad/s, slip {slip}, hand gap {(SiteWorld(m_Arms[0].Site) - SiteWorld(m_Arms[1].Site)).Length():F3} m (palms touch at {2f * (m_HalfW + PalmOffset):F3}){(speed < 0.5f ? " | SIM SLOW" : "")}"
            );
            foreach (Arm a in m_Arms)
            {
                Vector3 site = SiteWorld(a.Site);
                float face = a.Side * Vector3.Dot(site - c, lat) - m_HalfW - PalmOffset,
                    lo = site.Y - PalmBelow(a),
                    on =
                        Math.Max(
                            0f,
                            Math.Min(lo + 2f * k_PalmHalf, c.Y + m_HalfH)
                                - Math.Max(lo, c.Y - m_HalfH)
                        ) / (2f * k_PalmHalf);
                var touching = new List<string>();
                for (int i = 0; i < a.Geoms.Length; i++)
                    if (a.Touch[i])
                        touching.Add(a.GeomName[i]);
                Log.Info(
                    $"[G1] GR   {a.Name}: palm {a.PalmForce:F1} N{(a.PalmOn ? "" : " (off its face)")}, fingers {a.Force - a.PalmForce:F1} N, thumb {a.ThumbForce:F1} N"
                        + $" | gap target {a.Gap:F3} m, palm measured {face:F3} m off its face, palm height on the face {on * 100f:F0}% (lower edge {lo - (c.Y - m_HalfH):F3} m over the box bottom)"
                        + $" | curl {a.Curl:F2} rad, thumb {a.Thumb:F2} rad, palm {a.PalmErr:F0} deg off | err {HandError(a):F3} m | touching [{string.Join(", ", touching)}]"
                        + $" | {WristLog(a)}"
                        + $" | {ArmLog(a, dt)}"
                );
            }
        }

        // Wrist roll / pitch / yaw: commanded vs measured (deg) - the actuators' torque limits let contact loads push the joints off their command.
        private string WristLog(Arm a) =>
            $"wrist cmd [{string.Join(", ", Array.ConvertAll(a.WristQ, v => (v * Mathf.Rad2Deg).ToString("F0")))}]"
            + $" measured [{string.Join(", ", Array.ConvertAll(a.WristAdr, i => (m_Mujoco!.GetJointPosition(i) * Mathf.Rad2Deg).ToString("F0")))}] deg";

        // Shoulder pitch / roll / yaw + elbow (measured, deg and deg/s since the last line) and the grasp site vs its IK target in pelvis coords
        // (forward, up, left; velocity in that frame) - shows how the arms swing on the body.
        private string ArmLog(Arm a, float dt)
        {
            float[] q = Array.ConvertAll(a.ArmAdr, i => (float)m_Mujoco!.GetJointPosition(i));
            Vector3 site = ToBody(SiteWorld(a.Site) - Pelvis());
            string log =
                $"arm (sh pitch, roll, yaw, elbow) [{string.Join(", ", Array.ConvertAll(q, v => (v * Mathf.Rad2Deg).ToString("F0")))}] deg"
                + $" rate [{string.Join(", ", Array.ConvertAll(new[] { 0, 1, 2, 3 }, i => ((q[i] - a.LastArmQ[i]) / dt * Mathf.Rad2Deg).ToString("F0")))}] deg/s"
                + $" | site on body (fwd, up, left) {F(site)} v {F((site - a.LastSite) / dt)} m/s, IK target {F(ToBody(HandTarget(a) - Pelvis()))}";
            a.LastArmQ = q;
            a.LastSite = site;
            return log;
        }

        // Measured joint angles (deg) at these qpos addresses, for the log.
        private string Degs(uint[] adr) =>
            string.Join(
                ", ",
                Array.ConvertAll(
                    adr,
                    i => (m_Mujoco!.GetJointPosition(i) * Mathf.Rad2Deg).ToString("F0")
                )
            );

        // Status line: phase, pelvis, last walk command; the walk goal while moving; every box (and why it isn't picked) while
        // looking for one; the handled box; and the hands while they work.
        private void LogStatus()
        {
            Vector3 p = Pelvis();
            string s =
                $"[G1] {m_Phase}: pelvis ({p.X:F2}, {p.Y:F2}, {p.Z:F2}) yaw {YawOf(Heading()) * Mathf.Rad2Deg:F0} deg | cmd vx={m_Cmd.X:F2} vy={m_Cmd.Y:F2} yaw={m_Cmd.Z:F2}";
            if (
                m_Phase
                is Phase.FaceBox
                    or Phase.WalkToBox
                    or Phase.TurnToTable
                    or Phase.WalkToTable
            )
                s += $" | goal ({m_Stand.X:F2}, {m_Stand.Z:F2}), to go {m_ToGo:F2} m";
            if (m_Phase == Phase.FindBox)
                for (int i = 0; i < m_Boxes.Count; i++)
                    s +=
                        $" | box {i}: {Flat(BoxCenter(i) - p).Length():F2} m away, gap {ChuteGap(i):F3} m, speed {BoxSpeed(i):F2}"
                        + (
                            WalkToBox
                                ? $", out of reach by {Math.Max(0f, ReachShort(ChuteEntity!, BoxCenter(i), StandDistance)):F2} m"
                                : ""
                        )
                        + (m_Fails[i] >= MaxBoxAttempts ? ", skipped until it moves" : "");
            else if (m_Box >= 0)
                s += $" | box {m_Box} at {BoxCenter(m_Box)}";
            if (m_Phase >= Phase.Ready && m_Phase <= Phase.Release)
            {
                s += $" | {GripInfo()}";
                foreach (Arm a in m_Arms)
                {
                    bool placed = Placed(a, out float side, out float front, out float up);
                    s +=
                        $" | {a.Name} err {HandError(a):F3} m, force {a.Force:F1} N, thumb {a.ThumbForce:F1} N, gap {a.Gap:F3} m, curl {a.Curl:F2} rad, palm {a.PalmErr:F0} deg off"
                        + $", box frame: palm {side:F3} out, thumb {front:F3} fwd, site {up:F3} up{(placed ? " (placed)" : "")}";
                }
            }
            Log.Info(s);
        }

        // Grip geometry: box CoG (a uniform box geom, so its centre), its size (width = across the squeeze axis), target and actual grasp-site separation
        // (palms touch the box at width + 2 x PalmOffset).
        private string GripInfo()
        {
            Vector3 c = BoxCenter(m_Box);
            float target = (HandTarget(m_Arms[0]) - HandTarget(m_Arms[1])).Length(),
                actual = (SiteWorld(m_Arms[0].Site) - SiteWorld(m_Arms[1].Site)).Length();
            float Under(Arm a) => SiteWorld(a.Site).Y - PalmBelow(a) - (c.Y - m_HalfH); // palm's lower edge over the box bottom
            return $"box CoG ({c.X:F3}, {c.Y:F3}, {c.Z:F3}), {2f * m_HalfW:F3} wide x {2f * m_HalfD:F3} deep x {2f * m_HalfH:F3} high | hand gap target {target:F3} m, actual {actual:F3} m"
                + $" | palm edge over box bottom L {Under(m_Arms[0]):F3} R {Under(m_Arms[1]):F3} m";
        }
    }
}
