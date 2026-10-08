using Hazel;

namespace G1Sort
{
    /// <summary>
    /// Contact force sensing.
    ///
    /// GetGeomContactForce returns a scalar force against ANY geom, not just the box,
    /// so constant self-contact would look like grip force. To remove it:
    ///  - Tare() stores each hand geom's force as a baseline right before the hands reach in,
    ///  - TaredForce() = max(0, raw - baseline) is what every grip decision uses.
    /// RunIdleContactCheck() reports any hand contact while the arms are folded, so you
    /// can tell whether self-collision is really disabled.
    /// </summary>
    public partial class G1BoxGrasp
    {
        /// <summary>Raw contact force on one geom (N), magnitude only.</summary>
        private float RawForce(uint id) => m_Mujoco != null ? m_Mujoco.GetGeomContactForce(id) : 0f;

        /// <summary>Logs every hand geom that reports contact or force with the arms folded.</summary>
        private void RunIdleContactCheck()
        {
            if (m_Mujoco == null) return;
            string report = "";
            for (int i = 0; i < 2; i++)
            {
                ArmChain a = Arm(i);
                report += IdleEntries(a.Name + ".palm", a.PalmGeomIds);
                for (int f = 0; f < a.Fingers.Length; f++)
                    report += IdleEntries(a.Name + "." + a.Fingers[f].Label, a.Fingers[f].GeomIds);
            }
            if (report.Length > 0)
                Log.Error("[G1BoxGrasp] IDLE hand contact with arms folded:" + report
                    + " -> self-collision (or belt/body) contact is still enabled; check the hand geoms' contype/conaffinity or add <exclude> pairs. Tare will mask constant contact, but it contaminates finger sensing.");
            else
                Log.Info("[G1BoxGrasp] idle contact check clean - hand geoms touch nothing at rest.");
        }

        /// <summary>Formats the geoms of one group that are touching something.</summary>
        private string IdleEntries(string label, uint[] ids)
        {
            if (m_Mujoco == null) return "";
            string s = "";
            for (int i = 0; i < ids.Length; i++)
            {
                float f = RawForce(ids[i]);
                bool c = m_Mujoco.IsGeomInContact(ids[i]);
                if (f > ContactForceEps || c)
                    s += " " + label + "[geom " + ids[i] + " f=" + f.ToString("F1") + (c ? " contact" : "") + "]";
            }
            return s;
        }

        /// <summary>Captures the per-geom baseline force (hands open, not touching the box).</summary>
        private void Tare()
        {
            m_Base.Clear();
            string nonzero = "";
            for (int i = 0; i < 2; i++)
            {
                ArmChain a = Arm(i);
                for (int k = 0; k < a.AllGeomIds.Length; k++)
                {
                    uint id = a.AllGeomIds[k];
                    float f = RawForce(id);
                    m_Base[id] = f;
                    if (f > ContactForceEps) nonzero += " " + id + "=" + f.ToString("F1");
                }
            }
            m_Tared = true;
            Log.Info("[G1BoxGrasp] force tare captured" + (nonzero.Length > 0 ? " (non-zero baselines:" + nonzero + ")" : " (all zero)"));
        }

        /// <summary>max(0, raw - baseline) for one geom; raw if not tared yet.</summary>
        private float TaredForce(uint id)
        {
            float raw = RawForce(id);
            float b;
            if (m_Tared && m_Base.TryGetValue(id, out b)) raw -= b;
            return raw > 0f ? raw : 0f;
        }

        /// <summary>Sum of tared force over a geom group.</summary>
        private float SumEff(uint[] ids)
        {
            float sum = 0f;
            for (int i = 0; i < ids.Length; i++)
                sum += TaredForce(ids[i]);
            return sum;
        }

        /// <summary>True if any geom in the group has tared force above ContactForceEps.</summary>
        private bool InContactEff(uint[] ids)
        {
            for (int i = 0; i < ids.Length; i++)
                if (TaredForce(ids[i]) > ContactForceEps) return true;
            return false;
        }
    }

}
