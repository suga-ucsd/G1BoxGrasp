using Hazel;

namespace G1Sort
{

    /// <summary>
    /// One Dex3 finger (thumb: 3 joints, index/middle: 2 joints).
    ///
    /// All joints of a finger move together from a single normalized Curl value:
    ///   joint angle = lerp(OpenDeg[j], ClosedDeg[j], Curl)
    ///
    /// Contact fields (Force, InContact, OnFace) are written by the grip loop in
    /// G1BoxGrasp.Grasp.cs using TARED forces (see ContactSensing.cs).
    /// </summary>
    internal sealed class Finger
    {
        /// <summary>"thumb", "index" or "middle" (used in logs).</summary>
        public readonly string Label;

        /// <summary>Finger joint actuators, proximal to distal.</summary>
        public readonly MujocoActuator?[] Joints;

        /// <summary>Finger link body ids, proximal to distal (last = distal link).</summary>
        public readonly uint[] BodyIds;

        /// <summary>Per-joint angle at Curl = 0 (degrees).</summary>
        public readonly float[] OpenDeg;

        /// <summary>Per-joint angle at Curl = 1 (degrees).</summary>
        public readonly float[] ClosedDeg;

        /// <summary>Collision/visual geoms on this finger's links (filled by ResolveGeoms).</summary>
        public uint[] GeomIds = new uint[0];

        /// <summary>Force target for the finger's admittance loop (thumb: unused, it closes geometrically).</summary>
        public float TargetForce;

        /// <summary>0 = open, 1 = closed.</summary>
        public float Curl;

        /// <summary>Tared contact force summed over this finger's geoms (N).</summary>
        public float Force;

        /// <summary>Tared force above ContactForceEps.</summary>
        public bool InContact;

        /// <summary>In contact on the INTENDED face (thumb: front face, index/middle: side face).</summary>
        public bool OnFace;

        public Finger(string label, MujocoActuator?[] joints, uint[] bodyIds)
        {
            Label = label;
            Joints = joints;
            BodyIds = bodyIds;
            OpenDeg = new float[joints.Length];
            ClosedDeg = new float[joints.Length];
        }

        /// <summary>Distal (tip) link body id, or uint.MaxValue if unknown.</summary>
        public uint DistalBody => BodyIds.Length > 0 ? BodyIds[BodyIds.Length - 1] : uint.MaxValue;

        /// <summary>True if every joint actuator was found.</summary>
        public bool Bound
        {
            get
            {
                for (int i = 0; i < Joints.Length; i++)
                    if (Joints[i] == null) return false;
                return true;
            }
        }

        /// <summary>Fully curled but touching nothing (log symbol '!').</summary>
        public bool Saturated => Curl >= 0.999f && !InContact;

        /// <summary>Writes every joint toward lerp(open, closed, Curl), rate limited.</summary>
        public void Drive(float maxRateDegPerSec, float ts)
        {
            for (int j = 0; j < Joints.Length; j++)
                Geo.WriteJointDeg(Joints[j], Mathf.Lerp(OpenDeg[j], ClosedDeg[j], Curl), maxRateDegPerSec, ts);
        }
    }
}
