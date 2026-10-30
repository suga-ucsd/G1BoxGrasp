using System;
using System.Collections.Generic;
using Hazel;

namespace G1Pipe
{
    // Grip forces: squeeze control, contact reading and taring, required force.
    public partial class G1BoxGrasp
    {
        // While holding: keep squeezing at HeldPalmForce (or what the weight needs), track the hand targets, boost the force if the box slips, fail if it slides out.
        private bool Hold(float ts)
        {
            Squeeze(
                ts,
                Math.Max(RequiredForce(), Math.Min(HeldPalmForce, MaxPalmForce) / k_SqueezeMargin),
                true
            );
            SendHands(HandTarget, false);
            LogGrip(ts);
            float slip = PalmMidY() - BoxCenter(m_Box).Y - m_Y0; // + = box sliding down between the palms
            if (slip > DropTolerance)
            {
                m_Carrying = false; // it's out of the hands: nothing left to hold on to
                Fail($"box slipped out of the hands ({slip:F3} m)");
                return false;
            }
            if (slip > SlipTolerance)
                m_Boost = Math.Min(m_Boost + ts, 2f);
            return true;
        }

        // Squeeze per hand, on the palm's own force: the gap closes at full rate until the palm touches, then presses toward k_SqueezeMargin x
        // target (admittance). The palm is on its face from TouchForce until it stays under half of it for k_PalmHold. Index + middle curl in
        // while the palm is on (easing back while it is off) up to k_MaxCurl, wrapping the box's front corners, but never press past
        // MaxFingerForce (they ease back over it, curl on again under 0.8 x it); once held they stay wrapped, a palm that comes off closes
        // straight back in past them, and the hand never spreads unless the palm presses past MaxPalmForce.
        private void Squeeze(float ts, float target, bool held = false)
        {
            ReadForces();
            foreach (Arm a in m_Arms)
            {
                bool free = !a.PalmOn && (held || a.Force + a.ThumbForce < TouchForce); // palm not on its face yet (or off it while held): close at full rate
                float close = free
                    ? k_MaxSqueezeRate
                    : Mathf.Clamp(
                        k_DepthGain * (k_SqueezeMargin * target - a.PalmForce),
                        -k_MaxSqueezeRate,
                        k_MaxSqueezeRate
                    );
                if (held && a.PalmForce < MaxPalmForce)
                    close = Math.Max(close, 0f);
                a.Gap = Mathf.Clamp(
                    a.Gap - close * ts,
                    -MaxSqueezeDepth,
                    held ? 0f : ApproachGap + k_Widen
                );
                bool touch = a.PalmForce >= (a.PalmOn ? 0.5f * TouchForce : TouchForce);
                a.PalmLow = touch ? 0f : a.PalmLow + ts;
                bool on = touch || (a.PalmOn && a.PalmLow < k_PalmHold);
                if (on != a.PalmOn)
                    Log.Info(
                        $"[G1] GR {a.Name} palm {(on ? "on" : "off")} its face in {m_Phase} ({a.PalmForce:F1} N, gap {a.Gap:F3} m, fingers {a.Force - a.PalmForce:F1} N, curl {a.Curl:F2} rad) -> {(on ? "fingers curl in" : held ? "palm closes in" : "fingers ease back")}"
                    );
                a.PalmOn = on;
                float fingers = a.Force - a.PalmForce;
                a.Curl =
                    fingers > MaxFingerForce ? Math.Max(0f, a.Curl - k_CurlRate * ts)
                    : on
                        ? fingers < 0.8f * MaxFingerForce
                                ? Math.Min(a.Curl + k_CurlRate * ts, k_MaxCurl)
                            : a.Curl
                    : held ? a.Curl
                    : Math.Max(0f, a.Curl - 2f * k_CurlRate * ts);
            }
        }

        // Tared contact force of each hand: palm + fingers (the squeeze on the side face, with the palm's share apart) and thumb (front face).
        private void ReadForces()
        {
            foreach (Arm a in m_Arms)
            {
                a.Force = a.ThumbForce = a.PalmForce = 0f;
                for (int i = 0; i < a.Geoms.Length; i++)
                {
                    float f = Math.Max(0f, m_Mujoco!.GetGeomContactForce(a.Geoms[i]) - a.Base[i]);
                    if (a.IsThumb[i])
                        a.ThumbForce += f;
                    else
                        a.Force += f;
                    if (a.IsPalm[i])
                        a.PalmForce += f;
                }
            }
        }

        // Total tared contact of both hands, thumbs included.
        private float Contact() =>
            m_Arms[0].Force + m_Arms[1].Force + m_Arms[0].ThumbForce + m_Arms[1].ThumbForce;

        // Stores each hand geom's resting contact force so constant self-contact isn't read as grip.
        private void Tare()
        {
            foreach (Arm a in m_Arms)
                for (int i = 0; i < a.Geoms.Length; i++)
                    a.Base[i] = m_Mujoco!.GetGeomContactForce(a.Geoms[i]);
        }

        // Collision geoms of one hand (wrist_yaw_link + hand links) for contact force sensing, and which of them are thumb and palm links.
        private void HandGeoms(Arm a)
        {
            var geoms = new List<uint>();
            var thumb = new List<bool>();
            var names = new List<string>();
            var palm = new List<bool>();
            for (uint g = 0; g < m_Mujoco!.ModelNgeom; g++)
            {
                if (
                    !m_Mujoco.GetGeomDescriptor(
                        g,
                        out int body,
                        out _,
                        out _,
                        out int contype,
                        out int conaffinity,
                        out _
                    )
                    || (contype | conaffinity) == 0
                )
                    continue;
                string name = m_Mujoco.GetBodyNameByIndex((uint)body) ?? "";
                if (name != a.Name + "_wrist_yaw_link" && !name.StartsWith(a.Name + "_hand_"))
                    continue;
                geoms.Add(g);
                thumb.Add(name.StartsWith(a.Name + "_hand_thumb"));
                palm.Add(name == a.Name + "_wrist_yaw_link");
                names.Add($"{name}#{g}");
            }
            a.Geoms = geoms.ToArray();
            a.IsThumb = thumb.ToArray();
            a.IsPalm = palm.ToArray();
            a.GeomName = names.ToArray();
        }

        // The hands hold (or are closing on) the box: targets tracked directly, thumbs and fingers in their grip pose.
        private bool Gripping() =>
            m_Phase
                is Phase.Squeeze
                    or Phase.DragIn
                    or Phase.Holding
                    or Phase.Lift
                    or Phase.PullIn
                    or Phase.BackOff
                    or Phase.TurnToTable
                    or Phase.WalkToTable
                    or Phase.FaceTable
                    or Phase.Place;

        // Squeeze force per hand that holds the box: factor * m*g / (2*mu), boosted while slipping, capped so a palm never presses past MaxPalmForce.
        private float RequiredForce() =>
            Math.Min(
                GripSafetyFactor
                    * m_Mujoco!.GetBodyMass(m_Boxes[m_Box].Body)
                    * 9.81f
                    / (2f * BoxFriction)
                    * m_Boost,
                MaxPalmForce / k_SqueezeMargin
            );

        // Distance from a hand's grasp site to its target.
        private float HandError(Arm a) => (SiteWorld(a.Site) - HandTarget(a)).Length();

        // Average height of the two grasp sites.
        private float PalmMidY() =>
            0.5f * (SiteWorld(m_Arms[0].Site).Y + SiteWorld(m_Arms[1].Site).Y);
    }
}
