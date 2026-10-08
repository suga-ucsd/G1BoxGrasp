using System;
using System.Collections.Generic;
using Hazel;

namespace G1Pipe
{
    // Catches each box that slides down the chute with both hands, places it on the table to the left, and comes back for the next.
    // FaceStation -> WalkToStation -> FaceChute -> Standby -> Squeeze -> Lift -> PullIn -> TurnToTable -> WalkToTable -> Place -> Release -> Retract -> FaceStation ...
    // A failed attempt folds the arms (Recover) and goes back for the next box; only setup errors stop the robot (Failed).
    public class G1BoxGrasp : Entity
    {
        private enum Phase { Start, FaceStation, WalkToStation, FaceChute, Standby, Squeeze, Lift, PullIn, TurnToTable, WalkToTable, Place, Release, Retract, Recover, Failed }

        /// <summary>One arm. Side = +1 left, -1 right: mirroring the two arms is just this sign.</summary>
        private sealed class Arm
        {
            public readonly float Side;
            public readonly string Name, Input;        // joint/body prefix ("left") and IK input prefix ("LeftArm_")
            public readonly Identifier Target, Pole;   // cached IK input ids
            public uint Site = uint.MaxValue;          // grasp site = the LimbIK end effector
            public uint[] Geoms = new uint[0];         // hand collision geoms
            public float[] Base = new float[0];        // their contact force at tare time
            public float Gap, Raise, Force;            // palm offset outside its box face (m, - = squeezing), height above contact (m), tared force (N)

            public Arm(float side)
            {
                Side = side;
                Name = side > 0f ? "left" : "right";
                Input = side > 0f ? "LeftArm_" : "RightArm_";
                Target = new Identifier(Input + "Target_Position");
                Pole = new Identifier(Input + "Pole_Target");
            }
        }

        private static readonly string[] s_LegsAndWaistOnly = {
            "left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint",
            "left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
            "right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint",
            "right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
            "waist_yaw_joint","waist_roll_joint","waist_pitch_joint",
        };

        // Motion graph: Start stores the current pose and blends to the IK solution over Duration; after that targets are tracked every frame.
        private static readonly Identifier s_Duration = new Identifier("Duration"), s_Start = new Identifier("Start");

        // Walker/rotator command IDs: SetFloat(slot, id, value) writes one command channel.
        private const uint k_SetVx = 1u;       // forward speed (m/s)
        private const uint k_SetVy = 2u;       // sideways speed (m/s)
        private const uint k_SetYawRate = 3u;  // turn rate (rad/s)

        private const float k_YawGain = 2f, k_MaxYawRate = 1f;                                    // heading P-gain (rad/s per rad) and cap
        private const float k_ArriveTol = 0.05f;                                                   // m
        private const float k_Settle = 0.6f;                                                       // s at zero velocity before a policy swap or a reach (G9)
        private const float k_TurnRate = 0.7f, k_TurnRampIn = 0.5f, k_TurnTol = 0.05f;             // rotator: max rad/s, ramp-in s, done within rad
        private const float k_Timeout = 10f, k_WalkTimeout = 60f;                                  // s
        private const float k_PositionTol = 0.03f;                                                 // m, grasp site this close to its target = in position
        private const float k_StopSpeed = 0.05f, k_StopZone = 0.06f;                               // box at rest (m/s); its face this close to the stopper (m)
        private const float k_DepthGain = 0.002f, k_MaxSqueezeRate = 0.02f, k_MaxSqueezeDepth = 0.03f;   // squeeze admittance: m/(N*s), m/s, m
        private const float k_ForceTolFrac = 0.35f, k_GripConfirm = 0.5f;                         // grip OK: both hands >= 65% of target for 0.5 s
        private const float k_LiftSpeed = 0.05f, k_SlipTol = 0.01f, k_DropTol = 0.05f;            // m/s, m, m
        private const float k_CarrySpeed = 0.1f;                                                   // m/s, box moves in the hands
        private const float k_PlaceHover = 0.05f, k_PlaceLowerSpeed = 0.03f;                       // m, m/s
        private const float k_OpenSpeed = 0.05f, k_ReleaseGap = 0.05f, k_ReleaseTime = 1.5f, k_RetractUp = 0.10f;   // m/s, m, s, m

        [Group("Walk")]
        public float WalkSpeed = 0.112f;        // m/s
        [Group("Walk")]
        public Entity? StationEntity;           // where the pelvis stands in front of the chute (default: where the robot starts)

        [Group("Box")]
        public Entity? ChuteEntity;                // slanted surface the boxes slide on (unit-cube mesh, local +Y up)
        [Group("Box")]
        public float HandStandoff = 0.03f;         // m, hands this far outside each side face
        [Group("Box")]
        public float FrontInset = 0.05f;           // m, hands this far behind the front face (near the front corners)
        [Group("Box")]
        public float BoxFriction = 1f;             // hand-box mu (MuJoCo uses the larger of the two geoms' friction)

        [Group("Arms")]
        public Vector3 FoldOffset = new Vector3(0.15f, 0.15f, 0.20f);    // hand target while walking, from the pelvis: (forward, up, outward)
        [Group("Arms")]
        public Vector3 PoleOffset = new Vector3(-0.30f, 0.10f, 0.15f);   // elbow hint from the pelvis: (forward, up, outward)
        [Group("Arms")]
        public Vector3 CarryOffset = new Vector3(0.30f, 0.10f, 0f);      // box centre while carrying, from the pelvis: (forward, up, left)
        [Group("Arms")]
        public float MoveDuration = 0.8f;          // s, IK blend time of each big move
        [Group("Arms")]
        public float UpstreamRaise = 0.10f;        // m, uphill hand this far above the box top while boxes slide in
        [Group("Arms")]
        public float LiftHeight = 0.08f;           // m
        [Group("Arms")]
        public float GripSafetyFactor = 2f;        // squeeze per hand = factor * m*g / (2*mu)

        [Group("Place")]
        public Entity? PlaceTarget;                // marker on the table top: first slot for a box's bottom centre
        [Group("Place")]
        public float PlaceSpacing = 0.25f;         // m, next slots continue along PlaceTarget's +X
        [Group("Place")]
        public int PlaceSlots = 4;
        [Group("Place")]
        public float PlaceStandDistance = 0.45f;   // m, pelvis -> slot when placing

        [Group("Policy")]
        public uint WalkerSlotId = 1u;
        [Group("Policy")]
        public uint RotatorSlotId = 2u;

        [HideFromEditor] private RobotControllerComponent? m_Robot;
        [HideFromEditor] private MujocoSceneComponent? m_Mujoco;
        [HideFromEditor] private readonly Arm[] m_Arms = { new Arm(1f), new Arm(-1f) };
        [HideFromEditor] private readonly List<(uint Body, uint Geom, Vector3 Half)> m_Boxes = new List<(uint Body, uint Geom, Vector3 Half)>();   // every free box
        [HideFromEditor] private int m_Box = -1;                              // index of the box being handled
        [HideFromEditor] private uint m_PelvisId = uint.MaxValue;
        [HideFromEditor] private Quaternion m_MjToHz = Quaternion.Identity;   // MuJoCo -> Hazel axes
        [HideFromEditor] private Vector3 m_MjOffset;                          // MuJoCo -> Hazel translation
        [HideFromEditor] private Phase m_Phase = Phase.Start;
        [HideFromEditor] private int m_Step;                                  // sub-step inside a phase
        [HideFromEditor] private float m_PhaseTime, m_StepTime;               // s since the phase / sub-step started
        [HideFromEditor] private bool m_RotatorOn;                            // rotator (not walker) drives the legs
        [HideFromEditor] private Vector3 m_ChuteN, m_ChuteTop, m_Downhill;    // chute surface normal, centre of its top face, unit downhill direction
        [HideFromEditor] private Vector3 m_Station, m_Slot;                   // where the pelvis waits for boxes; table slot for the current box
        [HideFromEditor] private float m_StopSide;                            // +1 = left hand is the stopper

        // Grip frame: box centre, squeeze axis (robot left), forward axis. World coords, or pelvis coords (forward, up, left) while carrying.
        [HideFromEditor] private Vector3 m_Center, m_Lat, m_Fwd;
        [HideFromEditor] private bool m_Carrying;
        [HideFromEditor] private float m_HalfW, m_HalfD, m_HalfH;             // box half extents along m_Lat, m_Fwd and up
        [HideFromEditor] private float m_Lift, m_Boost = 1f, m_Y0;            // lift (m), force boost while slipping, palm-over-box height at grip

        // Components, walker on legs + waist, and the IK setup of both arms (MuJoCo state is read on the first frame).
        protected override void OnCreate()
        {
            m_Robot = GetComponent<RobotControllerComponent>();
            m_Mujoco = GetComponent<MujocoSceneComponent>();
            if (m_Robot == null || m_Mujoco == null) return;
            SetLocomotion(false);
            foreach (Arm a in m_Arms)
            {
                // Set once and never changed: the IK owns the shoulders and the elbow, the wrists stay put.
                foreach (string j in new[] { "_shoulder_pitch_joint", "_shoulder_roll_joint", "_shoulder_yaw_joint", "_elbow_joint" })
                    SetIK(a.Name + j, true);
                foreach (string j in new[] { "_wrist_roll_joint", "_wrist_pitch_joint", "_wrist_yaw_joint" })
                    SetIK(a.Name + j, false);
                m_Robot.SetInputBool(new Identifier(a.Input + "Solve_Orientation"), false);   // position only
                m_Robot.SetInputBool(new Identifier(a.Input + "High_Accuracy"), true);
                m_Robot.SetInputFloat(new Identifier(a.Input + "Pole_Weight"), 1f);
            }
        }

        // Runs the current phase; the robot stands still unless the phase walks or turns.
        protected override void OnUpdate(float ts)
        {
            if (m_Robot == null || m_Mujoco == null) return;
            m_PhaseTime += ts;
            m_StepTime += ts;
            Drive(0f, 0f);

            switch (m_Phase)
            {
                case Phase.Start: Begin(); break;
                case Phase.FaceStation:
                    SendHands(Fold, 0f);
                    if (Flat(m_Station - Pelvis()).Length() < 0.1f || Turn(YawOf(m_Station - Pelvis()))) Advance(Phase.WalkToStation);
                    break;
                case Phase.WalkToStation:
                    SendHands(Fold, 0f);
                    if (WalkTo(m_Station, 0f)) Advance(Phase.FaceChute);
                    else if (m_PhaseTime > k_WalkTimeout) Fail("could not reach the station");
                    break;
                case Phase.FaceChute:
                    SendHands(Fold, 0f);
                    if (Turn(ChuteFacingYaw())) EnterStandby();
                    break;
                case Phase.Standby: StepStandby(); break;
                case Phase.Squeeze: StepSqueeze(ts); break;
                case Phase.Lift: StepLift(ts); break;
                case Phase.PullIn:
                    if (Hold(ts) && MoveTowards(ref m_Center, CarryOffset, k_CarrySpeed * ts)) EnterTurnToTable();
                    break;
                case Phase.TurnToTable:
                    if (Hold(ts) && Turn(YawOf(m_Slot - Pelvis()))) Advance(Phase.WalkToTable);
                    break;
                case Phase.WalkToTable:
                    if (Hold(ts) && WalkTo(m_Slot, PlaceStandDistance)) Advance(Phase.Place);
                    else if (m_PhaseTime > k_WalkTimeout) Fail("could not reach the table");
                    break;
                case Phase.Place: StepPlace(ts); break;
                case Phase.Release: StepRelease(ts); break;
                case Phase.Retract: StepRetract(); break;
                case Phase.Recover:
                    SendHands(Fold, 0f);
                    if (m_PhaseTime >= MoveDuration) Advance(Phase.FaceStation);
                    break;
                default: SendHands(Fold, 0f); break;   // Failed: stand with the arms folded
            }
        }

        // First frames (MuJoCo isn't ready in OnCreate): find the scene's boxes, read the chute and the hands, fold the arms.
        private void Begin()
        {
            m_MjToHz = m_Mujoco!.GetAxisHzToMj().Conjugate;
            m_MjOffset = m_Mujoco.GetSceneTranslation();
            m_PelvisId = m_Mujoco.GetBodyID("pelvis");
            if (ChuteEntity == null || PlaceTarget == null) { Fail("assign ChuteEntity and PlaceTarget", false); return; }
            FindBoxes();
            if (m_Boxes.Count == 0)
            {
                if (m_PhaseTime > 2f) Fail("no free-moving MuJoCo boxes in the scene", false);
                return;   // scene bodies can join the model a few frames after start: retry
            }
            foreach (var b in m_Boxes) m_Mujoco.UnweldBody(b.Body);   // clear welds left by earlier versions of this script (welds default to persistent)
            TransformComponent chute = ChuteEntity.Transform;   // unit-cube mesh, local +Y up
            m_ChuteN = chute.WorldRotationQuat * Vector3.Up;
            m_ChuteTop = chute.WorldTranslation + m_ChuteN * (0.5f * chute.WorldScale.Y);
            m_Downhill = (Vector3.Down - m_ChuteN * Vector3.Dot(Vector3.Down, m_ChuteN)).Normalized();
            foreach (Arm a in m_Arms)
            {
                a.Site = m_Mujoco.GetSiteID(a.Name + "_grasp_site");
                a.Geoms = HandGeoms(a.Name);
                a.Base = new float[a.Geoms.Length];
            }
            m_Station = StationEntity != null ? StationEntity.Transform.WorldTranslation : Pelvis();
            Log.Info($"[G1] {m_Boxes.Count} boxes in the scene");
            SendHands(Fold, MoveDuration);
            Advance(Phase.FaceStation);
        }

        // Catch pose for the next box: a nominal box on the chute centreline in front of the pelvis; downhill hand = stopper, uphill hand hovers above the path.
        private void EnterStandby()
        {
            m_Fwd = Heading();
            m_Lat = LeftOf(m_Fwd);
            m_StopSide = Vector3.Dot(m_Downhill, m_Lat) > 0f ? 1f : -1f;
            m_HalfW = HalfExtent(0, m_Lat);
            m_HalfD = HalfExtent(0, m_Fwd);
            m_HalfH = HalfExtent(0, m_ChuteN);
            Vector3 line = m_ChuteTop + m_ChuteN * m_HalfH;   // centre of a box resting on the chute centreline
            m_Center = line + m_Downhill * (Vector3.Dot(Pelvis() - line, m_Lat) / Vector3.Dot(m_Downhill, m_Lat));
            m_Carrying = false;
            m_Lift = 0f;
            m_Boost = 1f;
            foreach (Arm a in m_Arms)
            {
                a.Gap = a.Side == m_StopSide ? 0f : HandStandoff;
                a.Raise = a.Side == m_StopSide ? 0f : m_HalfH + UpstreamRaise;
            }
            SendHands(HandTarget, MoveDuration);
            Advance(Phase.Standby);
        }

        // Hands wait at the catch pose (checked and tared once); a box that comes to rest against the stopper palm is the one to grasp.
        private void StepStandby()
        {
            if (m_Step == 0 && m_PhaseTime > MoveDuration + 1f)
            {
                float errL = HandError(m_Arms[0]), errR = HandError(m_Arms[1]);
                if (errL > k_PositionTol || errR > k_PositionTol) { Fail($"hands can't reach the catch pose: err L={errL:F3} R={errR:F3} m - move the station closer to the chute", false); return; }
                Tare();
                NextStep();
            }
            for (int i = 0; i < m_Boxes.Count; i++)
            {
                Vector3 rel = BoxCenter(i) - m_Center;
                float face = Vector3.Dot(rel, m_Lat) * m_StopSide + HalfExtent(i, m_Lat) - m_HalfW;                   // downhill face vs the stopper (0 = touching)
                float gap = Vector3.Dot(BoxCenter(i) - m_ChuteTop, m_ChuteN) - HalfExtent(i, m_ChuteN);             // clearance under it (0 = on the chute)
                if (face > -k_StopZone && face < 0.02f && Math.Abs(gap) < 0.02f && Math.Abs(Vector3.Dot(rel, m_Fwd)) < m_HalfD + 0.05f && BoxSpeed(i) < k_StopSpeed)
                {
                    EnterSqueeze(i);
                    return;
                }
            }
        }

        // Grip frame from the caught box, then lower the uphill hand beside it.
        private void EnterSqueeze(int box)
        {
            m_Box = box;
            m_Center = BoxCenter(box);
            m_HalfW = HalfExtent(box, m_Lat);
            m_HalfD = HalfExtent(box, m_Fwd);
            m_HalfH = HalfExtent(box, Vector3.Up);
            foreach (Arm a in m_Arms)
            {
                a.Gap = a.Side == m_StopSide ? 0f : HandStandoff;
                a.Raise = 0f;
            }
            SendHands(HandTarget, MoveDuration);
            Advance(Phase.Squeeze);
        }

        // After the uphill hand is down, squeeze until both palms hold the required force for GripConfirm seconds.
        private void StepSqueeze(float ts)
        {
            SendHands(HandTarget, 0f);
            if (m_PhaseTime < MoveDuration) { m_StepTime = 0f; return; }
            float req = RequiredForce(), min = (1f - k_ForceTolFrac) * req;
            Squeeze(ts, req);
            if (m_Arms[0].Force < min || m_Arms[1].Force < min) m_StepTime = 0f;
            if (m_StepTime >= k_GripConfirm)
            {
                Log.Info($"[G1] grip: L={m_Arms[0].Force:F1} R={m_Arms[1].Force:F1} N (need {req:F1} N)");
                m_Y0 = PalmMidY() - BoxCenter(m_Box).Y;
                Advance(Phase.Lift);
            }
            else if (m_PhaseTime > k_Timeout) Fail($"grip too weak: L={m_Arms[0].Force:F1} R={m_Arms[1].Force:F1} N of {req:F1} N, gap L={m_Arms[0].Gap:F3} R={m_Arms[1].Gap:F3} m");
        }

        // Lift straight up, then hold the box in pelvis coords so it follows the body while turning and walking.
        private void StepLift(float ts)
        {
            if (!Hold(ts)) return;
            m_Lift = Math.Min(m_Lift + k_LiftSpeed * ts, LiftHeight);
            if (m_Lift < LiftHeight) return;
            m_Center = ToBody(m_Center + Vector3.Up * m_Lift - Pelvis());
            m_Lat = ToBody(m_Lat);
            m_Fwd = ToBody(m_Fwd);
            m_Lift = 0f;
            m_Carrying = true;
            Advance(Phase.PullIn);
        }

        // Picks the first free table slot, then turns toward it.
        private void EnterTurnToTable()
        {
            if (!FreeSlot(out m_Slot)) { Fail("the table is full", false); return; }
            Advance(Phase.TurnToTable);
        }

        // Settle, switch the grip back to world coords, move the box over its slot, then lower it until it rests on the table.
        private void StepPlace(float ts)
        {
            if (!Hold(ts)) return;
            Vector3 rest = m_Slot + Vector3.Up * m_HalfH;   // box centre resting on the table
            if (m_Step == 0 && m_StepTime >= k_Settle)
            {
                m_Center = BodyPoint(m_Center);
                m_Lat = BodyDir(m_Lat);
                m_Fwd = BodyDir(m_Fwd);
                m_Carrying = false;
                NextStep();
            }
            else if (m_Step == 1 && MoveTowards(ref m_Center, rest + Vector3.Up * k_PlaceHover, k_CarrySpeed * ts)) NextStep();
            else if (m_Step == 2)
            {
                m_Center = m_Center - Vector3.Up * (k_PlaceLowerSpeed * ts);
                if (BoxCenter(m_Box).Y <= rest.Y + 0.01f || m_Center.Y < rest.Y - 0.02f) Advance(Phase.Release);
            }
        }

        // Open both palms away from the box, then start lifting the hands clear.
        private void StepRelease(float ts)
        {
            foreach (Arm a in m_Arms) a.Gap = Math.Min(a.Gap + k_OpenSpeed * ts, k_ReleaseGap);
            SendHands(HandTarget, 0f);
            if (m_PhaseTime < k_ReleaseTime) return;
            m_Lift = k_RetractUp;
            SendHands(HandTarget, MoveDuration);
            Advance(Phase.Retract);
        }

        // Hands up first, then back to the fold pose - never forward over the placed box (G21) - then go back for the next box.
        private void StepRetract()
        {
            if (m_Step == 0 && m_PhaseTime >= MoveDuration)
            {
                SendHands(Fold, MoveDuration);
                NextStep();
            }
            else if (m_Step == 1 && m_PhaseTime >= 2f * MoveDuration)
            {
                Log.Info($"[G1] placed {Flat(BoxCenter(m_Box) - m_Slot).Length():F3} m from its slot");
                Advance(Phase.FaceStation);
            }
        }

        // While holding: keep squeezing, track the hand targets, boost the force if the box slips, fail if it slides out.
        private bool Hold(float ts)
        {
            Squeeze(ts, RequiredForce());
            SendHands(HandTarget, 0f);
            float slip = PalmMidY() - BoxCenter(m_Box).Y - m_Y0;   // + = box sliding down between the palms
            if (slip > k_DropTol) { Fail($"box slipped out of the hands ({slip:F3} m)"); return false; }
            if (slip > k_SlipTol) m_Boost = Math.Min(m_Boost + ts, 2f);
            return true;
        }

        // Squeeze admittance per hand: the gap closes until the tared force reaches the target.
        private void Squeeze(float ts, float target)
        {
            foreach (Arm a in m_Arms)
            {
                a.Force = 0f;
                for (int i = 0; i < a.Geoms.Length; i++)
                    a.Force += Math.Max(0f, m_Mujoco!.GetGeomContactForce(a.Geoms[i]) - a.Base[i]);
                a.Gap = Mathf.Clamp(a.Gap - Mathf.Clamp(k_DepthGain * (target - a.Force), -k_MaxSqueezeRate, k_MaxSqueezeRate) * ts, -k_MaxSqueezeDepth, HandStandoff);
            }
        }

        // Stores each hand geom's resting contact force so constant self-contact isn't read as grip.
        private void Tare()
        {
            foreach (Arm a in m_Arms)
                for (int i = 0; i < a.Geoms.Length; i++)
                    a.Base[i] = m_Mujoco!.GetGeomContactForce(a.Geoms[i]);
        }

        // Free-moving boxes: scene bodies (named lucky_body_<entity id>) with a box geom and a free joint.
        private void FindBoxes()
        {
            m_Boxes.Clear();
            for (uint g = 0; g < m_Mujoco!.ModelNgeom; g++)
            {
                if (!m_Mujoco.GetGeomDescriptor(g, out int body, out int type, out _, out _, out _, out Vector3 half) || type != 6) continue;   // 6 = mjGEOM_BOX
                uint b = (uint)body;
                if ((m_Mujoco.GetBodyNameByIndex(b) ?? "").StartsWith("lucky_body_") && m_Mujoco.GetBodyFreeJointIndex(b) < m_Mujoco.ModelNjnt)
                    m_Boxes.Add((b, g, half));
            }
        }

        // First table slot (PlaceTarget + k*PlaceSpacing along its +X) with no other box resting near it.
        private bool FreeSlot(out Vector3 slot)
        {
            Vector3 first = PlaceTarget!.Transform.WorldTranslation, row = PlaceTarget.Transform.WorldRotationQuat * Vector3.Right;
            for (int k = 0; k < PlaceSlots; k++)
            {
                slot = first + row * (k * PlaceSpacing);
                bool taken = false;
                for (int i = 0; i < m_Boxes.Count; i++)
                    taken |= i != m_Box && Flat(BoxCenter(i) - slot).Length() < 0.5f * PlaceSpacing && Math.Abs(BoxCenter(i).Y - slot.Y) < 0.3f;
                if (!taken) return true;
            }
            slot = first;
            return false;
        }

        // Collision geoms of one hand (wrist_yaw_link + hand links), for contact force sensing.
        private uint[] HandGeoms(string side)
        {
            var geoms = new List<uint>();
            for (uint g = 0; g < m_Mujoco!.ModelNgeom; g++)
            {
                if (!m_Mujoco.GetGeomDescriptor(g, out int body, out _, out _, out int contype, out int conaffinity, out _) || (contype | conaffinity) == 0) continue;
                string name = m_Mujoco.GetBodyNameByIndex((uint)body) ?? "";
                if (name == side + "_wrist_yaw_link" || name.StartsWith(side + "_hand_")) geoms.Add(g);
            }
            return geoms.ToArray();
        }

        // Hand target: palm Gap outside its side face, FrontInset behind the front face, lifted by m_Lift (+ the arm's Raise).
        private Vector3 HandTarget(Arm a)
        {
            Vector3 c = m_Carrying ? BodyPoint(m_Center) : m_Center;
            Vector3 lat = m_Carrying ? BodyDir(m_Lat) : m_Lat, fwd = m_Carrying ? BodyDir(m_Fwd) : m_Fwd;
            return c + lat * (a.Side * (m_HalfW + a.Gap)) + fwd * (FrontInset - m_HalfD) + Vector3.Up * (m_Lift + a.Raise);
        }

        // Folded hand target in front of the body.
        private Vector3 Fold(Arm a) => BodyPoint(Mirror(FoldOffset, a.Side));

        // Writes both hand targets and elbow poles; duration > 0 also restarts the graph's min-jerk move (Start).
        private void SendHands(Func<Arm, Vector3> target, float duration)
        {
            foreach (Arm a in m_Arms)
            {
                m_Robot!.SetInputVector3(a.Target, target(a));
                m_Robot.SetInputVector3(a.Pole, BodyPoint(Mirror(PoleOffset, a.Side)));
            }
            if (duration <= 0f) return;
            m_Robot!.SetInputFloat(s_Duration, duration);
            m_Robot.SetInputTrigger(s_Start);
        }

        // Closed-loop walk: steer at the goal, stop standDist short of it (turns in place if badly misaligned). True on arrival.
        private bool WalkTo(Vector3 goal, float standDist)
        {
            Vector3 to = Flat(goal - Pelvis());
            float left = to.Length() - standDist, err = Mathf.WrapToPi(YawOf(to) - YawOf(Heading()));
            if (left <= k_ArriveTol) return true;
            Drive(Math.Abs(err) < 0.6f ? (left < 0.3f ? 0.5f : 1f) * WalkSpeed : 0f, Mathf.Clamp(err * k_YawGain, -k_MaxYawRate, k_MaxYawRate));
            return false;
        }

        // Turn in place with the rotator: settle, swap in, ramped P-turn, stop, swap back, settle (G9). True when done.
        private bool Turn(float yaw)
        {
            float err = Mathf.WrapToPi(yaw - YawOf(Heading()));
            switch (m_Step)
            {
                case 0:   // settle on the walker; no turn needed if already facing
                    if (m_StepTime < k_Settle) return false;
                    if (Math.Abs(err) < k_TurnTol) return true;
                    SetLocomotion(true);
                    break;
                case 1:   // turn, yaw rate ramped in from standstill
                    Drive(0f, Mathf.Clamp(err * k_YawGain, -k_TurnRate, k_TurnRate) * Math.Min(1f, m_StepTime / k_TurnRampIn));
                    if (Math.Abs(err) > k_TurnTol && m_StepTime < k_Timeout) return false;
                    break;
                case 2:   // zero yaw while the rotator is still active, then hand back to the walker
                    if (m_StepTime < 0.2f) return false;
                    SetLocomotion(false);
                    break;
                default:  // settle on the walker
                    return m_StepTime >= k_Settle;
            }
            NextStep();
            return false;
        }

        // Heading that faces the chute square: perpendicular to downhill, toward the chute.
        private float ChuteFacingYaw()
        {
            Vector3 face = LeftOf(FlatDir(m_Downhill));
            return YawOf(Vector3.Dot(face, ChuteEntity!.Transform.WorldTranslation - Pelvis()) >= 0f ? face : face * -1f);
        }

        // Swaps walker <-> rotator; the incoming policy drives legs + waist only, the arms stay with the IK (G8).
        private void SetLocomotion(bool rotator)
        {
            uint on = rotator ? RotatorSlotId : WalkerSlotId, off = rotator ? WalkerSlotId : RotatorSlotId;
            m_Robot!.SetDrivenJoints(on, s_LegsAndWaistOnly);
            m_Robot.SetPolicyActive(off, false);
            m_Robot.SetPolicyActive(on, true);
            m_RotatorOn = rotator;
        }

        // Sends (vx, 0, yaw rate) to whichever locomotion policy is active.
        private void Drive(float vx, float yawRate)
        {
            uint slot = m_RotatorOn ? RotatorSlotId : WalkerSlotId;
            m_Robot!.SetFloat(slot, k_SetVx, vx);
            m_Robot.SetFloat(slot, k_SetVy, 0f);
            m_Robot.SetFloat(slot, k_SetYawRate, yawRate);
        }

        // The only place the phase changes: resets the phase timers and logs the transition.
        private void Advance(Phase next)
        {
            Log.Info($"[G1] {m_Phase} -> {next}");
            m_Phase = next;
            m_Step = 0;
            m_PhaseTime = m_StepTime = 0f;
        }

        // Logs why, stops turning and folds the arms (letting go of any box); retry = go back for the next box, else stop for good.
        private void Fail(string reason, bool retry = true)
        {
            Log.Error($"[G1] failed in {m_Phase}: {reason}");
            if (m_RotatorOn) SetLocomotion(false);
            SendHands(Fold, MoveDuration);
            Advance(retry ? Phase.Recover : Phase.Failed);
        }

        // Next sub-step of the current phase.
        private void NextStep()
        {
            m_Step++;
            m_StepTime = 0f;
        }

        // Hands one joint to the IK solver (true) or keeps it out of the IK (false).
        private void SetIK(string joint, bool on)
        {
            MujocoActuator? act = m_Mujoco?.GetActuator(joint);
            if (act != null) act.IK = on;
        }

        // Box i from MuJoCo: centre (world), speed, and half extent along a world direction (support function of its box geom).
        private Vector3 BoxCenter(int i)
        {
            m_Mujoco!.GetGeomPoseMj(m_Boxes[i].Geom, out Vector3 p, out _);
            return m_MjToHz * p + m_MjOffset;
        }
        private float BoxSpeed(int i)
        {
            m_Mujoco!.GetBodyPoseMj(m_Boxes[i].Body, out _, out _, out Vector3 v, out _);
            return v.Length();
        }
        private float HalfExtent(int i, Vector3 dir)
        {
            m_Mujoco!.GetGeomPoseMj(m_Boxes[i].Geom, out _, out Quaternion q);
            Quaternion r = m_MjToHz * q;
            Vector3 h = m_Boxes[i].Half;
            return Math.Abs(Vector3.Dot(dir, r * Vector3.Right)) * h.X + Math.Abs(Vector3.Dot(dir, r * Vector3.Up)) * h.Y + Math.Abs(Vector3.Dot(dir, r * Vector3.Forward)) * h.Z;
        }

        // Squeeze force per hand that holds the box: factor * m*g / (2*mu), boosted while slipping.
        private float RequiredForce() => GripSafetyFactor * m_Mujoco!.GetBodyMass(m_Boxes[m_Box].Body) * 9.81f / (2f * BoxFriction) * m_Boost;

        // Distance from a hand's grasp site to its target.
        private float HandError(Arm a) => (SiteWorld(a.Site) - HandTarget(a)).Length();

        // Average height of the two grasp sites.
        private float PalmMidY() => 0.5f * (SiteWorld(m_Arms[0].Site).Y + SiteWorld(m_Arms[1].Site).Y);

        // MuJoCo poses in Hazel world coordinates: axis swap + scene translation, as the reference project did (PLAN.md G22).
        private Vector3 SiteWorld(uint site) => m_MjToHz * m_Mujoco!.GetSitePositionMj(site) + m_MjOffset;
        private Vector3 BodyPos(uint body) { m_Mujoco!.GetBodyPoseMj(body, out Vector3 p, out _, out _, out _); return m_MjToHz * p + m_MjOffset; }
        private Quaternion BodyRot(uint body) { m_Mujoco!.GetBodyPoseMj(body, out _, out Quaternion q, out _, out _); return m_MjToHz * q; }

        // Pelvis position (world) and horizontal heading, from MuJoCo.
        private Vector3 Pelvis() => BodyPos(m_PelvisId);
        private Vector3 Heading() => FlatDir(BodyRot(m_PelvisId) * Vector3.Right);

        // Pelvis coords (forward, up, left) <-> world, using the pelvis heading only so the grip ignores body sway.
        private Vector3 BodyDir(Vector3 b) => Heading() * b.X + Vector3.Up * b.Y + LeftOf(Heading()) * b.Z;
        private Vector3 BodyPoint(Vector3 b) => Pelvis() + BodyDir(b);
        private Vector3 ToBody(Vector3 v) => new Vector3(Vector3.Dot(v, Heading()), v.Y, Vector3.Dot(v, LeftOf(Heading())));

        // Engine is Y-up: yaw = atan2(-z, x), left of a heading = (z, 0, -x); "flat" drops the height.
        private static float YawOf(Vector3 d) => Mathf.Atan2(-d.Z, d.X);
        private static Vector3 LeftOf(Vector3 f) => new Vector3(f.Z, 0f, -f.X);
        private static Vector3 Flat(Vector3 v) => new Vector3(v.X, 0f, v.Z);
        private static Vector3 FlatDir(Vector3 v) => Flat(v).Normalized();

        // (forward, up, outward) offset of one arm: the right arm mirrors the outward part.
        private static Vector3 Mirror(Vector3 o, float side) => new Vector3(o.X, o.Y, side * o.Z);

        // Moves p toward goal by at most step; true once there.
        private static bool MoveTowards(ref Vector3 p, Vector3 goal, float step)
        {
            Vector3 d = goal - p;
            float len = d.Length();
            p = len <= step ? goal : p + d * (step / len);
            return len <= step;
        }
    }
}
