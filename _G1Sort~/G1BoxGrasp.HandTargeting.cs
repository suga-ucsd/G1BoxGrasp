using System;
using Hazel;

namespace G1Sort
{
    /// <summary>
    /// Where the hands go and how targets are sent to the IK component.
    ///
    ///  - Fold pose (walking / recovery).
    ///  - HandIkTarget: the one formula for every hand target near the box.
    ///  - Thumb placement: ServoInset slides each hand so the open thumb clears the front face.
    ///  - SendHandTargets / RetargetIfDrifted: the only path to the IK inputs.
    ///  - Constraints: belt clearance and reach clamp.
    /// </summary>
    public partial class G1BoxGrasp
    {
        // ==================================================================
        // Fold pose
        // ==================================================================

        /// <summary>Captures the fold frame from the current pelvis pose.</summary>
        private void BuildFoldPlan()
        {
            m_PlanLeft = GetHorizontalLeft();
            m_PlanMid = GetPelvis() + GetHorizontalForward() * FoldForward + new Vector3(0f, -FoldDrop, 0f);
        }

        /// <summary>Sends both hands to the tucked fold pose.</summary>
        private void CommandFold(float duration)
        {
            if (m_Robot == null) return;
            m_Robot.SetInputFloat(m_DurationId, duration);
            for (int i = 0; i < 2; i++)
            {
                ArmChain a = Arm(i);
                m_Robot.SetInputVector3(a.PosId, m_PlanMid + m_PlanLeft * (a.Side * FoldHalfGap));
                m_Robot.SetInputVector3(a.RotId, new Vector3(0f, GetPelvisYaw() * Mathf.Rad2Deg, 0f));
                m_Robot.SetInputVector3(a.PoleId, BodyRelative(PoleOffset.X, PoleOffset.Y, PoleOffset.Z, a.Side));
                m_Robot.SetInputFloat(a.PoleWeightId, PoleWeight);
                if (!SingleStartTrigger) m_Robot.SetInputTrigger(m_StartId);
            }
            if (SingleStartTrigger) m_Robot.SetInputTrigger(m_StartId);
        }

        /// <summary>Elbow angle in folded phases: FoldedElbowDeg, lerped from its last angle during Recover.</summary>
        private float ElbowCommandDeg(ArmChain a)
        {
            if (m_Phase == Phase.Recover)
                return Geo.LerpAngleDeg(a.ElbowRecoverFrom, FoldedElbowDeg, Mathf.Clamp(m_PhaseElapsed / Math.Max(RecoverDuration, 0.01f), 0f, 1f));
            return FoldedElbowDeg;
        }

        // ==================================================================
        // Hand targets
        // ==================================================================

        /// <summary>
        /// IK target for one hand:
        ///   halfLat = clamp(halfW + gap, minHalf, MaxHalfGap)
        ///   up      = halfH * ContactHeightFraction + HandHeightOffset + lift
        ///   along   = ContactForwardOffset + (-halfD + Inset)       [corner grasp]
        ///   target  = centre + lat*side*halfLat + Y*up + fwd*along + lat*side*PalmSurfaceOffset
        /// then ConstrainHand (belt clearance) and ClampToReach.
        /// gap: + outside the side face, - interference. side: +1 left, -1 right.
        /// </summary>
        private Vector3 HandIkTarget(Vector3 center, float side, float gap, float lift, float minHalf)
        {
            Vector3 lat = CurLat;
            Vector3 fwd = CurFwd;
            float halfLat = Mathf.Clamp(m_FaceHalfW + gap, minHalf, MaxHalfGap);
            float contactY = m_FaceHalfH * ContactHeightFraction + HandHeightOffset + lift;
            float along = ContactForwardOffset + (CornerGrasp ? (-m_FaceHalfD + ArmBySide(side).Inset) : 0f);
            Vector3 surface = center + lat * (side * halfLat) + new Vector3(0f, contactY, 0f) + fwd * along;
            Vector3 ik = surface + lat * (side * PalmSurfaceOffset);
            ik = ConstrainHand(ik);
            ik = ClampToReach(ik, side);
            return ik;
        }

        // ==================================================================
        // Thumb placement
        // ==================================================================

        /// <summary>Thumb tip (thumb_2_link origin) along the approach axis, relative to the box centre.</summary>
        private float ThumbAlong(ArmChain a)
        {
            if (m_Mujoco == null || a.Thumb.DistalBody == uint.MaxValue) return 0f;
            return Geo.Dot(Geo.Flat(m_Mujoco.GetPosition(a.Thumb.DistalBody) - m_Tracker.Position), CurFwd);
        }

        /// <summary>Thumb tip lateral distance from the box centre line, positive toward this hand's side.</summary>
        private float ThumbLatOut(ArmChain a)
        {
            if (m_Mujoco == null || a.Thumb.DistalBody == uint.MaxValue) return 0f;
            return Geo.Dot(Geo.Flat(m_Mujoco.GetPosition(a.Thumb.DistalBody) - m_Tracker.Position), CurLat) * a.Side;
        }

        /// <summary>
        /// Slides the hand along the approach axis so the OPEN thumb sits ThumbClearance in
        /// front of the front face:
        ///   err    = thumbAlong - (-halfD - ThumbClearance)
        ///   Inset += clamp(-InsetGain * err, +/-InsetMaxRate) * dt
        /// Works without knowing hand geometry: moving the hand moves the thumb 1:1.
        /// </summary>
        private void ServoInset(ArmChain a, float ts)
        {
            if (!CornerGrasp || m_Mujoco == null || a.Thumb.DistalBody == uint.MaxValue)
            {
                a.ThumbAlongErr = 0f;
                return;
            }
            float desired = -m_FaceHalfD - ThumbClearance;
            a.ThumbAlongErr = ThumbAlong(a) - desired;
            if (!AutoAdjustInset) return;
            float rate = Mathf.Clamp(-InsetGain * a.ThumbAlongErr, -InsetMaxRate, InsetMaxRate);
            a.Inset = Mathf.Clamp(a.Inset + rate * ts, MinFrontInset, MaxFrontInset);
        }

        /// <summary>True when thumb placement is not needed or the thumb error is within ThumbPlaceTol.</summary>
        private bool ThumbPlaced(ArmChain a) =>
            !CornerGrasp || !AutoAdjustInset || a.Thumb.DistalBody == uint.MaxValue || Geo.Abs(a.ThumbAlongErr) <= ThumbPlaceTol;

        // ==================================================================
        // Sending targets to the IK component
        // ==================================================================

        /// <summary>Sends both hand targets with one Duration and one Start trigger; remembers what was sent.</summary>
        private void SendHandTargets(Vector3 targetL, Vector3 targetR, float duration)
        {
            if (m_Robot == null) return;
            m_Robot.SetInputFloat(m_DurationId, duration);
            SendOneArmTarget(m_Left, targetL);
            SendOneArmTarget(m_Right, targetR);
            if (SingleStartTrigger) m_Robot.SetInputTrigger(m_StartId);
            m_LastCommandedL = targetL;
            m_LastCommandedR = targetR;
        }

        /// <summary>Writes one arm's position, orientation (unused: orientation solving is off), pole and pole weight.</summary>
        private void SendOneArmTarget(ArmChain a, Vector3 hand)
        {
            if (m_Robot == null) return;
            m_Robot.SetInputVector3(a.PosId, hand);
            m_Robot.SetInputVector3(a.RotId, new Vector3(0f, GetPelvisYaw() * Mathf.Rad2Deg, 0f));
            m_Robot.SetInputVector3(a.PoleId, BodyRelative(PoleOffset.X, PoleOffset.Y, PoleOffset.Z, a.Side));
            m_Robot.SetInputFloat(a.PoleWeightId, PoleWeight);
            if (!SingleStartTrigger) m_Robot.SetInputTrigger(m_StartId);
        }

        /// <summary>Resends only when <paramref name="interval"/> has passed AND a target moved more than <paramref name="threshold"/>.</summary>
        private void RetargetIfDrifted(Vector3 tL, Vector3 tR, float interval, float threshold, float duration, float ts)
        {
            m_RetargetTimer += ts;
            if (m_RetargetTimer < interval) return;
            if ((tL - m_LastCommandedL).Length() > threshold || (tR - m_LastCommandedR).Length() > threshold)
            {
                SendHandTargets(tL, tR, duration);
                m_RetargetTimer = 0f;
            }
        }

        /// <summary>Arms ready for a new grasp: elbow IK on, fingers open, gaps/insets reset, tare cleared.</summary>
        private void ActivateArms()
        {
            m_Tared = false;
            for (int i = 0; i < 2; i++)
            {
                ArmChain a = Arm(i);
                a.SetElbowIK(true);
                a.SetCurlAll(0f);
                a.ResetThumbState();
                a.Contacted = false;
                a.Gap = HandStandoff;
                a.Inset = FrontInset;
                a.ThumbAlongErr = 1f;
            }
        }

        // ==================================================================
        // Constraints
        // ==================================================================

        /// <summary>Belt direction = ConveyorEntity local +X (world X if no conveyor).</summary>
        private Vector3 BeltDir()
        {
            Entity? c = ConveyorEntity;
            Vector3 fallback = new Vector3(1f, 0f, 0f);
            if (c == null) return fallback;
            return Geo.FlatDir(c.Transform.WorldRotationQuat * fallback, fallback);
        }

        /// <summary>True if a point is over the belt footprint (half width + margin).</summary>
        private bool IsOverBelt(Vector3 p)
        {
            Entity? c = ConveyorEntity;
            if (c == null) return false;
            Vector3 origin = c.Transform.WorldTranslation;
            Vector3 lat = Geo.LeftOf(BeltDir());
            float d = Geo.Dot(new Vector3(p.X - origin.X, 0f, p.Z - origin.Z), lat);
            return Geo.Abs(d) <= ConveyorHalfWidth + ClearanceMargin;
        }

        /// <summary>Over the belt, floors target height at ConveyorSurfaceY + ClearanceMargin.</summary>
        private Vector3 ConstrainHand(Vector3 p)
        {
            if (ConveyorEntity == null || !IsOverBelt(p))
                return p;
            float minY = ConveyorSurfaceY + ClearanceMargin;
            return p.Y < minY ? new Vector3(p.X, minY, p.Z) : p;
        }

        /// <summary>Pulls targets inside MaxArmReach of an approximate shoulder point (off when MaxArmReach = 0).</summary>
        private Vector3 ClampToReach(Vector3 hand, float side)
        {
            if (MaxArmReach <= 0f) return hand;
            Vector3 shoulder = BodyRelative(ShoulderForwardOffset, ShoulderHeightOffset, ShoulderLateralOffset, side);
            Vector3 d = hand - shoulder;
            float len = d.Length();
            if (len <= MaxArmReach || len < 0.0001f) return hand;
            return shoulder + d * (MaxArmReach / len);
        }
    }

}
