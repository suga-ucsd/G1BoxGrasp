using Hazel;

namespace G1Sort
{
    /// <summary>
    /// Console status lines.
    ///
    /// Hand summary legend:
    ///   side=  tared side-face force (N)      gap=  lateral gap (m, - = squeezing)
    ///   in=    inset behind front face (m)    tErr= thumb placement error (m)
    ///   palm=  palm tared contact             c= f= finger curl and tared force
    ///   *      tared contact                  #  on the intended face
    ///   !      fully curled, no contact       front= thumb distance to front face plane (m)
    ///   R:plane/force/stall  how the thumb reached the face
    /// </summary>
    public partial class G1BoxGrasp
    {
        /// <summary>One-line state of a hand (used in periodic logs and failure messages).</summary>
        private string HandSummary(ArmChain a)
        {
            string s = a.Name + " side=" + a.HandForce.ToString("F1") + " gap=" + a.Gap.ToString("F4")
                + " in=" + a.Inset.ToString("F3") + " tErr=" + a.ThumbAlongErr.ToString("F3")
                + " palm=" + InContactEff(a.PalmGeomIds) + " [";
            for (int f = 0; f < a.Fingers.Length; f++)
            {
                Finger fg = a.Fingers[f];
                s += (f > 0 ? " " : "") + fg.Label + " c=" + fg.Curl.ToString("F2") + " f=" + fg.Force.ToString("F1")
                    + (fg.InContact ? "*" : "") + (fg.OnFace ? "#" : "") + (fg.Saturated ? "!" : "");
                if (f == 0 && CornerGrasp)
                    s += " front=" + (ThumbAlong(a) + m_FaceHalfD).ToString("F3") + (a.ThumbReached ? " R:" + a.ThumbReason : "");
            }
            return s + "]";
        }

        /// <summary>Every 1 s in WalkTo/Wait when LogWalkStatus: along, lateral, stand distance, speed, in-reach.</summary>
        private void LogWalkDiagnostics()
        {
            if (!HasBox) { Log.Info("[G1BoxGrasp] " + m_Phase + ": no box (target null or estimate stale)"); return; }
            Vector3 rel = Geo.Flat(m_Tracker.Position - GetPelvis());
            Vector3 g = GripForward();
            Log.Info("[G1BoxGrasp] " + m_Phase + ": along=" + Geo.Dot(rel, g).ToString("F3") + " lat=" + Geo.Dot(rel, Geo.LeftOf(g)).ToString("F3")
                + " standDist=" + ResolveStandDistance().ToString("F3") + " speed=" + Geo.Flat(m_Tracker.Velocity).Length().ToString("F3")
                + " inReach=" + BoxInReach());
        }

        /// <summary>Every 0.5 s in arm phases when LogGraspStatus: phase/stage, force target, both hands, lift/slip/boost.</summary>
        private void LogGraspDiagnostics()
        {
            string s = "[G1BoxGrasp] " + m_Phase + (m_Phase == Phase.Grasp ? "/" + m_GraspStage : "")
                + " tgt=" + RequiredForce().ToString("F1") + (m_Tared ? "" : " (untared)")
                + " | " + HandSummary(m_Left) + " | " + HandSummary(m_Right);
            if (m_Phase == Phase.Lift || m_Phase == Phase.Holding || m_Phase == Phase.SetDown)
                s += " | lift=" + m_LiftH.ToString("F3") + " slip=" + m_LastSlip.ToString("F3") + " boost=" + m_ForceBoost.ToString("F2");
            Log.Info(s);
        }
    }

}
