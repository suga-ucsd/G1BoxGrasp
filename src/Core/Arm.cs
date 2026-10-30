using Hazel;

namespace G1Pipe
{
    // One arm's handles and per-frame state.
    public partial class G1BoxGrasp
    {
        /// <summary>One arm. Side = +1 left, -1 right: mirroring the two arms is just this sign.</summary>
        private sealed class Arm
        {
            public readonly float Side;
            public readonly string Name,
                Input; // joint/body prefix ("left") and IK input prefix ("LeftArm_")
            public readonly Identifier Target,
                Pole; // cached IK input ids
            public uint Site = uint.MaxValue; // grasp site = the LimbIK end effector
            public uint Shoulder = uint.MaxValue; // shoulder pitch link = the LimbIK chain root
            public uint[] Geoms = new uint[0]; // hand collision geoms
            public float[] Base = new float[0]; // their contact force at tare time
            public float Gap,
                Force; // air between palm and side face (m, - = squeezing), height above contact (m), tared force (N)
            public MujocoActuator[] Joints = new MujocoActuator[0]; // shoulder pitch, roll, yaw, elbow (IK), wrist roll, pitch, yaw (script)
            public uint[] WristLink = new uint[0]; // the links those joints turn
            public float[] WristQ = new float[0]; // wrist ctrl targets (rad)
            public bool WristHeld; // wrists held where they were when the grip was confirmed
            public uint[] WristAdr = new uint[0]; // wrist joints' qpos addresses (measured angles for the log)
            public uint[] ArmAdr = new uint[0]; // shoulder pitch, roll, yaw, elbow qpos addresses (measured angles for the log)
            public float[] LastArmQ = new float[4]; // arm angles (rad) at the last GR line, for their rates
            public Vector3 LastSite; // grasp site in pelvis coords at the last GR line, for its velocity
            public float PalmErr; // deg between the palm normal and the squeeze axis
            public MujocoActuator[] Hand = new MujocoActuator[0]; // thumb 1, thumb 2, index 0, middle 0 (script-owned)
            public bool[] IsThumb = new bool[0]; // which hand geoms are thumb links
            public bool[] IsPalm = new bool[0]; // which hand geoms are the palm (wrist_yaw_link)
            public float ThumbForce,
                PalmForce; // tared thumb contact force on the front face; palm-only share of Force (N)
            public bool PalmOn; // the palm is on its face now (fingers may curl only while it is)
            public float PalmLow; // s the palm has been under its touch force (it counts as off after k_PalmHold)
            public bool Capped; // its IK target is being pulled in to MaxReach of the shoulder
            public float Thumb,
                Tip,
                Curl; // thumb swing back from straight (- = in), thumb tip bend, index + middle curl (rad; right hand mirrored)
            public Vector3 Cmd,
                From,
                To; // IK target actually sent; ends of the path segment being run (SendHands)
            public float T,
                Tau; // segment duration and time into it (s); Tau >= T: no segment running
            public bool Rising; // the segment climbs straight up out of a low spot (wrist held so the fingers don't swing low)
            public bool Direct; // target tracked directly (within k_Snap / gripping) instead of along a path segment
            public string[] GeomName = new string[0]; // body of each hand geom, for the contact log
            public bool[] Touch = new bool[0]; // hand geom in contact at the last PreGrasp frame
            public float[] LoggedQ = new float[0]; // arm joint ctrl at the last PreGrasp line (rad)

            public Arm(float side)
            {
                Side = side;
                Name = side > 0f ? "left" : "right";
                Input = side > 0f ? "LeftArm_" : "RightArm_";
                Target = new Identifier(Input + "Target_Position");
                Pole = new Identifier(Input + "Pole_Target");
            }
        }
    }
}
