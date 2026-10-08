using Hazel;

namespace G1Sort
{
    /// <summary>
    /// WalkTo and Wait phases.
    ///
    /// StepWalkTo picks one of three walkers:
    ///   StationEntity set               -> StepWalkToStation (moving-box station)
    ///   AlignToBoxFaces + face latched  -> StepWalkAligned   (square up to a face)
    ///   otherwise                       -> StepWalkLegacy    (straight at the stand point)
    /// All three end in EnterArmsReady, which decides Wait / CatchReady / PreGrasp.
    /// </summary>
    public partial class G1BoxGrasp
    {
        /// <summary>Phase.WalkTo step: dispatches to the right walker.</summary>
        private void StepWalkTo()
        {
            if (StationEntity != null) { StepWalkToStation(); return; }
            if (!HasBox) { Walk(0f, 0f, 0f); return; }
            if (AlignToBoxFaces && m_ApproachLatched) StepWalkAligned();
            else StepWalkLegacy();
        }

        /// <summary>Direction to face at the station: belt normal pointing toward the conveyor (or the station's +X).</summary>
        private Vector3 StationFacing()
        {
            Entity? s = StationEntity;
            if (s == null) return GetHorizontalForward();
            Entity? c = ConveyorEntity;
            if (c != null)
            {
                Vector3 n = Geo.LeftOf(BeltDir());
                Vector3 toBelt = Geo.Flat(c.Transform.WorldTranslation - s.Transform.WorldTranslation);
                return Geo.Dot(n, toBelt) >= 0f ? n : n * -1f;
            }
            return Geo.FlatDir(s.Transform.WorldRotationQuat * new Vector3(1f, 0f, 0f), GetHorizontalForward());
        }

        /// <summary>Walk to StationEntity, then yaw in place to face the belt, then EnterArmsReady.</summary>
        private void StepWalkToStation()
        {
            Entity? s = StationEntity;
            if (s == null) return;
            Vector3 pelvis = GetPelvis();
            Vector3 goal = s.Transform.WorldTranslation;
            float dist = Geo.Flat(goal - pelvis).Length();

            if (m_PhaseElapsed >= WalkTimeout && Fail("station walk timeout")) return;

            if (dist > StationTolerance)
            {
                float err = HeadingErrorTo(goal);
                float speed = dist < 0.3f ? WalkSpeed * 0.5f : WalkSpeed;
                Walk(Geo.Abs(err) < 0.6f ? speed : 0f, 0f, Mathf.Clamp(err * YawGain, -MaxYawRate, MaxYawRate));
                return;
            }

            Vector3 face = StationFacing();
            float yawErr = Mathf.WrapToPi(Geo.YawFromDir(face) - GetPelvisYaw());
            Walk(0f, 0f, Mathf.Clamp(yawErr * YawGain, -MaxYawRate, MaxYawRate));
            if (Geo.Abs(yawErr) <= AlignYawTolDeg * Mathf.Deg2Rad)
            {
                m_ApproachDir = face;
                m_ApproachLatched = true;
                EnterArmsReady();
            }
        }

        /// <summary>
        /// Walk straight at the (predicted) stand point. With PredictiveIntercept the lead
        /// time is solved in two fixed-point iterations. Arrives at distance &lt;= standDist + tol.
        /// </summary>
        private void StepWalkLegacy()
        {
            Vector3 pelvis = GetPelvis();
            float standDist = ResolveStandDistance();
            Vector3 offset = StandOffset();

            float lead = 0f;
            if (PredictiveIntercept && WalkSpeed > 0.001f)
            {
                for (int i = 0; i < 2; i++)
                {
                    float d = Geo.Flat(m_Tracker.Predict(lead) + offset - pelvis).Length();
                    lead = Mathf.Clamp((d - standDist) / WalkSpeed, 0f, MaxLeadTime);
                }
            }

            Vector3 stand = m_Tracker.Predict(lead) + offset;
            float distance = Geo.Flat(stand - pelvis).Length();
            Walk(distance > standDist + ArriveTolerance ? WalkSpeed : 0f, 0f,
                Mathf.Clamp(HeadingErrorTo(stand) * YawGain, -MaxYawRate, MaxYawRate));

            if (WalkGiveUp(pelvis, standDist)) return;
            bool arrived = distance <= standDist + ArriveTolerance;
            if (m_PhaseElapsed >= WalkTimeout)
            {
                if (Fail("walk timeout")) return;
                arrived = true;
            }
            if (arrived) EnterArmsReady();
        }

        /// <summary>
        /// Face-aligned approach.
        ///  Stage A: walk to a waypoint AlignRadius behind the stand point on the approach axis.
        ///  Stage B: square yaw to the face, strafe to centre, walk straight in.
        /// Arrives when along, yaw and lateral are all within tolerance.
        /// </summary>
        private void StepWalkAligned()
        {
            Vector3 pelvis = GetPelvis();
            Vector3 g = m_ApproachDir;
            Vector3 gl = Geo.LeftOf(g);
            float standDist = ResolveStandDistance();
            Vector3 offset = StandOffset();

            float lead = 0f;
            if (PredictiveIntercept && WalkSpeed > 0.001f)
            {
                for (int i = 0; i < 2; i++)
                {
                    float a = Geo.Dot(m_Tracker.Predict(lead) + offset - pelvis, g);
                    lead = Mathf.Clamp((a - standDist) / WalkSpeed, 0f, MaxLeadTime);
                }
            }

            Vector3 stand = m_Tracker.Predict(lead) + offset;
            Vector3 rel = stand - pelvis;
            float along = Geo.Dot(rel, g);
            float lat = Geo.Dot(rel, gl);

            if (WalkGiveUp(pelvis, standDist)) return;
            bool timedOut = m_PhaseElapsed >= WalkTimeout;
            if (timedOut && Fail("walk timeout")) return;

            if (!m_InFinalApproach)
            {
                bool onLine = along <= standDist + AlignRadius + WaypointTol && Geo.Abs(lat) <= WaypointTol;
                if (onLine || along <= standDist + WaypointTol)
                    m_InFinalApproach = true;
            }

            // Stage A: go to the waypoint.
            if (!m_InFinalApproach)
            {
                Vector3 wp = stand - g * (standDist + AlignRadius);
                float err = HeadingErrorTo(wp);
                Walk(Geo.Abs(err) < 1.0f ? WalkSpeed : 0f, 0f, Mathf.Clamp(err * YawGain, -MaxYawRate, MaxYawRate));
                if (timedOut) EnterArmsReady();
                return;
            }

            // Stage B: square up, strafe, walk in.
            float yawErr = Mathf.WrapToPi(Geo.YawFromDir(g) - GetPelvisYaw());
            float alignFactor = Mathf.Clamp(1f - Geo.Abs(yawErr) / 0.6f, 0f, 1f);
            float vx = along > standDist + ArriveTolerance ? WalkSpeed * alignFactor : 0f;
            float vy = LateralGain != 0f ? Mathf.Clamp(lat * LateralGain, -MaxStrafe, MaxStrafe) : 0f;
            Walk(vx, vy, Mathf.Clamp(yawErr * YawGain, -MaxYawRate, MaxYawRate));

            bool yawOk = Geo.Abs(yawErr) <= AlignYawTolDeg * Mathf.Deg2Rad;
            bool latOk = LateralGain == 0f || Geo.Abs(lat) <= LateralTol;
            bool arrived = along <= standDist + ArriveTolerance && yawOk && latOk;
            if (timedOut) arrived = true;
            if (arrived) EnterArmsReady();
        }

        /// <summary>Offset of StandTarget from the box (zero if unused).</summary>
        private Vector3 StandOffset()
        {
            Entity? st = StandTarget;
            Entity? t = TargetObject;
            if (st == null || t == null || UseExternalEstimate) return new Vector3(0f, 0f, 0f);
            return st.Transform.WorldTranslation - t.Transform.WorldTranslation;
        }

        /// <summary>With failure handling on: fails if the box is far and receding faster than 1.2x WalkSpeed.</summary>
        private bool WalkGiveUp(Vector3 pelvis, float standDist)
        {
            if (!EnableFailureHandling) return false;
            Vector3 toBox = Geo.Flat(m_Tracker.Position - pelvis);
            float len = toBox.Length();
            if (len <= standDist + 0.5f) return false;
            float recede = Geo.Dot(m_Tracker.Velocity, toBox) / len;
            return recede > WalkSpeed * 1.2f && Fail("box receding faster than walk speed");
        }

        /// <summary>Stand distance adapted to box depth: StandDistance + (depth - ReferenceDepth) / 2, min 0.2 m.</summary>
        private float ResolveStandDistance()
        {
            float w, d, h;
            Dims(out w, out d, out h);
            float s = StandDistance + (d - ReferenceDepth) * 0.5f;
            return s < 0.2f ? 0.2f : s;
        }

        /// <summary>Box horizontal speed above MovingSpeedThreshold.</summary>
        private bool BoxMoving() => HasBox && Geo.Flat(m_Tracker.Velocity).Length() > MovingSpeedThreshold;

        /// <summary>Box inside the reach window (ReachAlongTol / ReachLateralTol) in front of the robot.</summary>
        private bool BoxInReach()
        {
            if (!HasBox) return false;
            Vector3 rel = Geo.Flat(m_Tracker.Position - GetPelvis());
            float along = Geo.Dot(rel, GripForward());
            float lat = Geo.Dot(rel, Geo.LeftOf(GripForward()));
            return Geo.Abs(along - ResolveStandDistance()) <= ReachAlongTol && Geo.Abs(lat) <= ReachLateralTol;
        }

        /// <summary>
        /// Decision point after walking:
        ///   no box -> Wait; box moving -> CatchReady; station and box not in reach -> Wait;
        ///   box does not fit -> Fail; otherwise -> PreGrasp.
        /// </summary>
        private void EnterArmsReady()
        {
            if (!HasBox || !m_FaceValid) { Advance(Phase.Wait); return; }
            if (EnableMovingCatch && BoxMoving()) { EnterCatchReady(); return; }
            if (StationEntity != null && !BoxInReach()) { Advance(Phase.Wait); return; }
            if (!CheckBoxFits()) return;
            EnterPreGrasp();
        }

        /// <summary>Phase.Wait step: arms folded; start a catch or a static pre-grasp when possible.</summary>
        private void StepWait()
        {
            if (!HasBox || !m_FaceValid) return;
            if (EnableMovingCatch && BoxMoving()) { EnterCatchReady(); return; }
            if (BoxInReach() && CheckBoxFits()) EnterPreGrasp();
        }
    }

}
