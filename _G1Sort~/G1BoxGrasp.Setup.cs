using System;
using System.Collections.Generic;
using Hazel;

namespace G1Sort
{
    /// <summary>
    /// One-time setup helpers called from OnCreate:
    ///  - finger open/closed poses from the Inspector,
    ///  - hand geom resolution (which geom ids belong to palm / thumb / index / middle).
    /// </summary>
    public partial class G1BoxGrasp
    {
        /// <summary>
        /// Copies Inspector finger angles into every Finger.
        /// Thumb open pose = preshape; index/middle open pose = 0.
        /// With MirrorRightFingers, right-hand angles are the negated left ones.
        /// </summary>
        private void ConfigureFingerPoses()
        {
            Vector3 ltO = LeftThumbPreshapeDeg, lt = LeftThumbClosedDeg, li = LeftIndexClosedDeg, lm = LeftMiddleClosedDeg;
            Vector3 rtO = MirrorRightFingers ? ltO * -1f : RightThumbPreshapeDeg;
            Vector3 rt = MirrorRightFingers ? lt * -1f : RightThumbClosedDeg;
            Vector3 ri = MirrorRightFingers ? li * -1f : RightIndexClosedDeg;
            Vector3 rm = MirrorRightFingers ? lm * -1f : RightMiddleClosedDeg;
            Vector3 zero = new Vector3(0f, 0f, 0f);

            SetPose(m_Left.Fingers[0], ltO, lt, 0f);
            SetPose(m_Left.Fingers[1], zero, li, FingerTargetForce);
            SetPose(m_Left.Fingers[2], zero, lm, FingerTargetForce);
            SetPose(m_Right.Fingers[0], rtO, rt, 0f);
            SetPose(m_Right.Fingers[1], zero, ri, FingerTargetForce);
            SetPose(m_Right.Fingers[2], zero, rm, FingerTargetForce);
        }

        /// <summary>Writes open/closed angles (Vector3 = joints 0,1,2) and force target into one finger.</summary>
        private static void SetPose(Finger f, Vector3 open, Vector3 closed, float targetForce)
        {
            float[] o = { open.X, open.Y, open.Z };
            float[] c = { closed.X, closed.Y, closed.Z };
            for (int j = 0; j < f.ClosedDeg.Length && j < 3; j++)
            {
                f.OpenDeg[j] = o[j];
                f.ClosedDeg[j] = c[j];
            }
            f.TargetForce = targetForce;
        }

        /// <summary>
        /// Scans geom ids 0..GeomScanLimit-1 with GetGeomDescriptor (first out param = parent body id)
        /// and hands the (geom, body) list to AssignGeoms for both arms.
        /// GeomScanLimit must not exceed the model's geom count.
        /// </summary>
        private void ResolveGeoms()
        {
            if (m_Mujoco == null) return;
            var scanned = new List<KeyValuePair<uint, int>>();
            for (int g = 0; g < GeomScanLimit; g++)
            {
                int bodyId, type, dataId, matId, group;
                Vector3 size;
                try
                {
                    m_Mujoco.GetGeomDescriptor((uint)g, out bodyId, out type, out dataId, out matId, out group, out size);
                }
                catch (Exception)
                {
                    break;
                }
                scanned.Add(new KeyValuePair<uint, int>((uint)g, bodyId));
            }
            AssignGeoms(m_Left, scanned, LeftFallbackGeomIds);
            AssignGeoms(m_Right, scanned, RightFallbackGeomIds);
        }

        /// <summary>Returns every scanned geom whose parent body is one of <paramref name="bodyIds"/>.</summary>
        private static uint[] CollectForBodies(List<KeyValuePair<uint, int>> scanned, uint[] bodyIds)
        {
            var result = new List<uint>();
            for (int i = 0; i < scanned.Count; i++)
                for (int b = 0; b < bodyIds.Length; b++)
                    if (bodyIds[b] != uint.MaxValue && scanned[i].Value == (int)bodyIds[b])
                    {
                        result.Add(scanned[i].Key);
                        break;
                    }
            return result.ToArray();
        }

        /// <summary>
        /// Fills an arm's geom groups: palm (wrist_yaw_link), each finger, SideGeomIds
        /// (palm + index + middle) and AllGeomIds (side + thumb). Falls back to the
        /// Inspector ids [palm, thumb0, thumb1, thumb2, middle0, middle1, index0, index1]
        /// if the scan found nothing. Logs the result.
        /// </summary>
        private void AssignGeoms(ArmChain a, List<KeyValuePair<uint, int>> scanned, uint[] fb)
        {
            a.PalmGeomIds = CollectForBodies(scanned, new uint[] { a.WristYawLink });
            for (int f = 0; f < a.Fingers.Length; f++)
                a.Fingers[f].GeomIds = CollectForBodies(scanned, a.Fingers[f].BodyIds);

            bool none = a.PalmGeomIds.Length == 0;
            for (int f = 0; f < a.Fingers.Length; f++)
                none &= a.Fingers[f].GeomIds.Length == 0;

            if (none && fb.Length >= 8)
            {
                a.PalmGeomIds = new uint[] { fb[0] };
                a.Fingers[0].GeomIds = new uint[] { fb[1], fb[2], fb[3] };
                a.Fingers[1].GeomIds = new uint[] { fb[6], fb[7] };
                a.Fingers[2].GeomIds = new uint[] { fb[4], fb[5] };
                Log.Error("[G1BoxGrasp] " + a.Name + ": geom scan found nothing - using fallback ids.");
            }

            var side = new List<uint>(a.PalmGeomIds);
            side.AddRange(a.Fingers[1].GeomIds);
            side.AddRange(a.Fingers[2].GeomIds);
            a.SideGeomIds = side.ToArray();

            var all = new List<uint>(a.SideGeomIds);
            all.AddRange(a.Fingers[0].GeomIds);
            a.AllGeomIds = all.ToArray();

            Log.Info("[G1BoxGrasp] " + a.Name + " geoms palm=[" + string.Join(",", a.PalmGeomIds)
                + "] thumb=[" + string.Join(",", a.Fingers[0].GeomIds)
                + "] index=[" + string.Join(",", a.Fingers[1].GeomIds)
                + "] middle=[" + string.Join(",", a.Fingers[2].GeomIds) + "]");
        }

    }
}
