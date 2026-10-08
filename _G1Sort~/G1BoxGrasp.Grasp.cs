using System;
using Hazel;

namespace G1Sort
{
    /// <summary>
    /// Grasp phase and the grip control loops.
    ///
    /// Stages:
    ///   Approach : hands move to FingerApproachGap outside the side faces.
    ///   Contact  : each palm creeps inward (Gap -= ContactAdvanceRate*dt) until tared side contact.
    ///   Close    : UpdateHandGrip every frame until GripConfirmed holds for GraspHoldTimeToConfirm.
    ///
    /// Loops in UpdateHandGrip (per hand):
    ///   arm squeeze   : Gap integrates (F_req - F_side) -> palm interference = normal force
    ///   index/middle  : curl toward FingerTargetForce
    ///   thumb         : geometric closing onto the front face (UpdateThumb)
    /// </summary>
    public partial class G1BoxGrasp
    {
        /// <summary>
        /// Starts the grasp: latches frame and grasp centre, captures tare (hands open,
        /// off the box), resets gaps/curls/thumb. From a catch, starts at Contact with the
        /// stopper already contacted; otherwise starts at Approach.
        /// </summary>
        private void EnterGrasp(bool fromCatch)
        {
            m_PreGraspAchieved = true;
            if (!m_FrameLatched)
            {
                m_FrameLatched = true;
                m_LatchedFwd = m_FaceNF;
                m_LatchedLat = m_FaceNL;
            }
            // Hands are open and at standoff here: capture baseline forces now.
            if (!fromCatch || !m_Tared) Tare();

            m_Moving = fromCatch;
            // Latched centre: thumbs push the box away, so targets must not chase it.
            m_GraspCenter = fromCatch ? m_CatchCenter : m_FaceCenter;
            m_ForceBoost = 1f;
            m_GraspStageElapsed = 0f;
            m_GraspHoldTime = 0f;

            for (int i = 0; i < 2; i++)
            {
                ArmChain a = Arm(i);
                a.SetCurlAll(0f);
                a.ResetThumbState();
                a.Contacted = false;
                a.Gap = fromCatch ? HandStandoff : FingerApproachGap;
            }

            if (fromCatch)
            {
                ArmChain stopper = ArmBySide(m_StopperSide);
                stopper.Contacted = true;
                stopper.Gap = CatchStopperGap;
                m_GraspStage = GraspStage.Contact;
            }
            else
            {
                m_GraspStage = GraspStage.Approach;
                SendHandTargets(HandIkTarget(m_GraspCenter, 1f, m_Left.Gap, 0f, MinSqueezeHalfGap),
                    HandIkTarget(m_GraspCenter, -1f, m_Right.Gap, 0f, MinSqueezeHalfGap), GraspApproachDuration);
            }

            Log.Info("[G1BoxGrasp] grasp start: " + (2f * m_FaceHalfW).ToString("F3") + " x " + (2f * m_FaceHalfD).ToString("F3")
                + " x " + (2f * m_FaceHalfH).ToString("F3") + " m, mass=" + BoxMassKg().ToString("F2") + " kg, side target "
                + RequiredForce().ToString("F1") + " N/hand");
            Advance(Phase.Grasp);
        }

        /// <summary>Builds both hand targets from each arm's Gap and the given lift height, resends if drifted.</summary>
        private void RetargetGrasp(Vector3 center, float lift, float ts)
        {
            Vector3 tL = HandIkTarget(center, 1f, m_Left.Gap, lift, MinSqueezeHalfGap);
            Vector3 tR = HandIkTarget(center, -1f, m_Right.Gap, lift, MinSqueezeHalfGap);
            RetargetIfDrifted(tL, tR, GraspRetargetInterval, GraspRetargetThreshold, GraspRetargetMoveDuration, ts);
        }

        /// <summary>Phase.Grasp step: runs the current stage.</summary>
        private void StepGrasp(float ts)
        {
            m_GraspStageElapsed += ts;

            // ---- Approach ----
            if (m_GraspStage == GraspStage.Approach)
            {
                bool contact = InContactEff(m_Left.SideGeomIds) || InContactEff(m_Right.SideGeomIds);
                if (contact || m_GraspStageElapsed >= GraspApproachDuration)
                {
                    m_GraspStage = GraspStage.Contact;
                    m_GraspStageElapsed = 0f;
                }
                return;
            }

            // ---- Contact ----
            if (m_GraspStage == GraspStage.Contact)
            {
                for (int i = 0; i < 2; i++)
                {
                    ArmChain a = Arm(i);
                    a.HandForce = SumEff(a.SideGeomIds);
                    if (a.Contacted) continue;
                    if (InContactEff(a.SideGeomIds)) a.Contacted = true;
                    else a.Gap = Math.Max(a.Gap - ContactAdvanceRate * ts, -MaxSqueezeDepth);
                }
                RetargetGrasp(m_GraspCenter, 0f, ts);

                if (m_Left.Contacted && m_Right.Contacted)
                {
                    m_GraspStage = GraspStage.Close;
                    m_GraspStageElapsed = 0f;
                    for (int i = 0; i < 2; i++)
                    {
                        ArmChain a = Arm(i);
                        a.ThumbStartAlong = ThumbAlong(a);
                        a.ThumbPrevAlong = a.ThumbStartAlong;
                    }
                    Log.Info("[G1BoxGrasp] both hands on side faces (gap L=" + m_Left.Gap.ToString("F4") + " R=" + m_Right.Gap.ToString("F4") + ") - closing");
                }
                else if (m_GraspStageElapsed >= ContactTimeout)
                {
                    Fail("no side-face contact (L=" + m_Left.Contacted + " R=" + m_Right.Contacted
                        + ", gap L=" + m_Left.Gap.ToString("F3") + " R=" + m_Right.Gap.ToString("F3") + ") - check box size/position estimate");
                }
                return;
            }

            // ---- Close ----
            UpdateHandGrip(m_Left, ts);
            UpdateHandGrip(m_Right, ts);
            RetargetGrasp(m_GraspCenter, 0f, ts);

            if (GripConfirmed()) m_GraspHoldTime += ts; else m_GraspHoldTime = 0f;
            if (m_GraspHoldTime >= GraspHoldTimeToConfirm)
            {
                EnterLift();
                return;
            }
            if (m_GraspStageElapsed >= SqueezeTimeout)
                Fail("grip timeout - " + HandSummary(m_Left) + " | " + HandSummary(m_Right) + " tgt=" + RequiredForce().ToString("F1"));
        }

        /// <summary>
        /// One hand's grip loops (called in Close, Lift, Holding, SetDown):
        ///  1. arm squeeze:  rate = clamp(DepthGain*(F_req - F_side), +/-MaxSqueezeRate)
        ///                   Gap  = clamp(Gap - rate*dt, -MaxSqueezeDepth, FingerApproachGap)
        ///  2. index/middle: curl toward TargetForce within FingerForceBand; classify side-face contact
        ///  3. thumb:        UpdateThumb
        /// </summary>
        private void UpdateHandGrip(ArmChain a, float ts)
        {
            a.HandForce = SumEff(a.SideGeomIds);
            float rate = Mathf.Clamp(DepthGain * (RequiredForce() - a.HandForce), -MaxSqueezeRate, MaxSqueezeRate);
            a.Gap = Mathf.Clamp(a.Gap - rate * ts, -MaxSqueezeDepth, FingerApproachGap);

            Vector3 boxC = m_Tracker.Position;
            for (int f = 1; f < a.Fingers.Length; f++)
            {
                Finger fg = a.Fingers[f];
                fg.Force = SumEff(fg.GeomIds);
                fg.InContact = fg.Force > ContactForceEps;

                if (fg.Force > fg.TargetForce + FingerForceBand)
                    fg.Curl -= FingerBackoffRate * ts;
                else if (fg.Force < fg.TargetForce - FingerForceBand)
                    fg.Curl += (fg.InContact ? FingerContactCloseRate : FingerFreeCloseRate) * ts;
                fg.Curl = Geo.Clamp01(fg.Curl);

                // On the side face = in contact AND distal link at (or beyond) the side face plane.
                fg.OnFace = fg.InContact;
                if (CornerGrasp && fg.InContact && fg.DistalBody != uint.MaxValue && m_Mujoco != null)
                {
                    Vector3 rel = Geo.Flat(m_Mujoco.GetPosition(fg.DistalBody) - boxC);
                    fg.OnFace = Geo.Dot(rel, CurLat) * a.Side >= m_FaceHalfW - FaceClassifyTol;
                }
            }

            UpdateThumb(a, ts);
        }

        /// <summary>
        /// Thumb geometric closing. Curls at ThumbCloseRate until the thumb reaches the
        /// front face by any of:
        ///   plane: thumb origin within ThumbTouchTol of the front face plane,
        ///   force: tared thumb force >= ThumbTouchForce,
        ///   stall: near the face and not moving for ThumbStallTime while curling.
        /// Then holds Curl = reachedCurl + ThumbPressCurl.
        /// Also logs a wrong curl direction, a full curl without reaching, or passing beside the box.
        /// </summary>
        private void UpdateThumb(ArmChain a, float ts)
        {
            Finger th = a.Thumb;
            th.Force = SumEff(th.GeomIds);
            th.InContact = th.Force > ContactForceEps;

            if (!CornerGrasp)
            {
                th.Curl = Geo.Clamp01(th.Curl + (th.InContact ? 0f : ThumbCloseRate * ts));
                th.OnFace = th.InContact;
                return;
            }

            float along = ThumbAlong(a);
            float latOut = ThumbLatOut(a);
            float front = -m_FaceHalfD;
            float speed = ts > 0.00001f ? (along - a.ThumbPrevAlong) / ts : 0f;
            a.ThumbPrevAlong = along;

            if (!a.ThumbReached)
            {
                th.Curl = Geo.Clamp01(th.Curl + ThumbCloseRate * ts);

                bool planeReached = along >= front - ThumbTouchTol;
                bool forceReached = th.Force >= ThumbTouchForce;
                bool nearFace = along >= front - ThumbStallZone;
                if (nearFace && speed < ThumbStallSpeed && th.Curl > 0.05f && th.Curl < 0.999f)
                    a.ThumbStallTimer += ts;
                else
                    a.ThumbStallTimer = 0f;
                bool stalled = a.ThumbStallTimer >= ThumbStallTime;

                if (planeReached || forceReached || stalled)
                {
                    a.ThumbReached = true;
                    a.ThumbReachedCurl = th.Curl;
                    a.ThumbReason = planeReached ? "plane" : (forceReached ? "force" : "stall");
                    Log.Info("[G1BoxGrasp] " + a.Name + " thumb reached front face (" + a.ThumbReason + ") at curl=" + th.Curl.ToString("F2")
                        + " along=" + (along - front).ToString("F3") + " latOut=" + latOut.ToString("F3") + "/" + m_FaceHalfW.ToString("F3"));
                }

                // Direction check: after ThumbDirCheckCurl the tip must have moved toward the face.
                if (!a.ThumbDirChecked && th.Curl >= ThumbDirCheckCurl)
                {
                    a.ThumbDirChecked = true;
                    float travel = along - a.ThumbStartAlong;
                    if (travel < ThumbDirMinTravel)
                        Log.Error("[G1BoxGrasp] " + a.Name + " thumb curl moved the tip " + travel.ToString("F3")
                            + " m toward the front face after curl " + ThumbDirCheckCurl.ToString("F2")
                            + " - thumb curls the wrong way: flip the sign of Left/RightThumbClosedDeg, or adjust the preshape (thumb_0).");
                }

                if (th.Curl >= 0.999f && !a.ThumbReached && !a.ThumbMissLogged)
                {
                    a.ThumbMissLogged = true;
                    Log.Error("[G1BoxGrasp] " + a.Name + " thumb fully curled without reaching the front face (along gap="
                        + (front - along).ToString("F3") + " m) - reduce ThumbClearance or raise ThumbClosedDeg.");
                }
            }
            else
            {
                th.Curl = Math.Min(a.ThumbReachedCurl + ThumbPressCurl, 1f);
            }

            bool withinFaceWidth = latOut <= m_FaceHalfW + FaceClassifyTol;
            th.OnFace = a.ThumbReached && withinFaceWidth;
            if (a.ThumbReached && !withinFaceWidth && !a.ThumbMissLogged)
            {
                a.ThumbMissLogged = true;
                Log.Error("[G1BoxGrasp] " + a.Name + " thumb passed BESIDE the box (latOut=" + latOut.ToString("F3")
                    + " > halfWidth " + m_FaceHalfW.ToString("F3") + ") - thumb preshape points outward.");
            }
        }

        /// <summary>
        /// One hand is gripping when: side force in band (upper bound skipped for a caught box),
        /// palm in contact (if required), enough side fingers on face, thumb on front face (if required).
        /// </summary>
        private bool HandGripOk(ArmChain a, float tgt)
        {
            float lo = tgt * (1f - ForceTolFrac);
            float hi = tgt * (1f + ForceTolFrac);
            bool forceOk = a.HandForce >= lo && (m_Moving || a.HandForce <= hi);
            bool palmOk = !RequirePalmContact || InContactEff(a.PalmGeomIds);
            bool sideOk = a.SideFingersOnFace() >= MinSideFingersOnFace;
            bool thumbOk = !CornerGrasp || !RequireThumbOnFront || a.Thumb.OnFace;
            return forceOk && palmOk && sideOk && thumbOk;
        }

        /// <summary>Both hands gripping and |F_L - F_R| within the balance tolerance.</summary>
        private bool GripConfirmed()
        {
            float tgt = RequiredForce();
            float balTol = (m_Moving ? MovingBalanceTolFrac : BalanceTolFrac) * tgt;
            return HandGripOk(m_Left, tgt) && HandGripOk(m_Right, tgt)
                && Geo.Abs(m_Left.HandForce - m_Right.HandForce) <= balTol;
        }
    }

}
