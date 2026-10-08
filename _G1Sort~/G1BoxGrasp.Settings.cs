using Hazel;

namespace G1Sort
{
    /// <summary>
    /// Every Inspector field of G1BoxGrasp, grouped exactly as they appear in the editor,
    /// plus the read-only status properties other scripts can poll.
    /// Units: metres, seconds, degrees, newtons unless stated.
    /// </summary>
    public partial class G1BoxGrasp
    {
        // ---- Targets -------------------------------------------------------
        [Group("Targets")]
        public Entity? TargetObject;                // the box (MuJoCo body); swap at runtime with SetTarget()
        [Group("Targets")]
        public Entity? StandTarget;                 // optional: stand relative to this instead of the box
        [Group("Targets")]
        public Entity? StationEntity;               // optional: fixed spot beside the belt for moving boxes
        [Group("Targets")]
        [Units("m")]
        public float StationTolerance = 0.05f;      // how close to the station counts as arrived

        // ---- Box -----------------------------------------------------------
        [Group("Box")]
        [Units("m")]
        public float ReferenceWidth = 0.15f;        // fallback dims when size is unknown
        [Group("Box")]
        [Units("m")]
        public float ReferenceDepth = 0.15f;        // also the depth StandDistance was tuned for
        [Group("Box")]
        [Units("m")]
        public float ReferenceHeight = 0.15f;
        [Group("Box")]
        [Units("m")]
        [Tooltip("Box size in BOX-LOCAL axes (X, Y=height, Z). Set per box via SetTarget(box, size).")]
        public Vector3 BoxSize = new Vector3(0.15f, 0.146f, 0.146f);
        [Group("Box")]
        public bool UseDensityForMass = true;       // true: mass = density * volume; false: BoxMass
        [Group("Box")]
        [Units("kg/m^3")]
        public float BoxDensity = 500f;
        [Group("Box")]
        [Units("kg")]
        public float BoxMass = 1.6f;
        [Group("Box")]
        public float BoxFriction = 1.0f;            // effective hand-box sliding friction (mu)

        // ---- Perception ----------------------------------------------------
        [Group("Perception")]
        public bool UseExternalEstimate = false;    // true: box comes from PushBoxEstimate() instead of TargetObject
        [Group("Perception")]
        [Units("s")]
        public float ExternalEstimateTimeout = 0.3f;

        // ---- Locomotion ----------------------------------------------------
        [Group("Locomotion")]
        [Units("m/s")]
        [Slider(0.0f, 1.5f)]
        public float WalkSpeed = 0.112f;
        [Group("Locomotion")]
        [Units("m")]
        [Slider(0.2f, 1.2f)]
        public float StandDistance = 0.585f;        // pelvis -> box centre at arrival (reference box)
        [Group("Locomotion")]
        [Units("m")]
        [Slider(0.0f, 0.5f)]
        public float ArriveTolerance = 0.005f;
        [Group("Locomotion")]
        [Slider(0.0f, 10.0f)]
        public float YawGain = 2.0f;                // heading P-gain (rad/s per rad)
        [Group("Locomotion")]
        [Units("rad/s")]
        [Slider(0.0f, 3.0f)]
        public float MaxYawRate = 1.0f;
        [Group("Locomotion")]
        [Units("s")]
        [Slider(0.0f, 60.0f)]
        public float WalkTimeout = 20.0f;
        [Group("Locomotion")]
        public bool PredictiveIntercept = true;     // walk to where the box WILL be
        [Group("Locomotion")]
        [Units("s")]
        public float VelocityFilterTime = 0.25f;
        [Group("Locomotion")]
        [Units("m/s")]
        public float MaxBoxSpeed = 3.3f;            // faster frame-to-frame motion = teleport
        [Group("Locomotion")]
        [Units("s")]
        public float MaxLeadTime = 5f;

        // ---- Approach ------------------------------------------------------
        [Group("Approach")]
        public bool AlignToBoxFaces = true;         // approach square to a box face (StepWalkAligned)
        [Group("Approach")]
        [Units("m")]
        public float AlignRadius = 0.8f;            // waypoint distance behind the stand point
        [Group("Approach")]
        [Units("m")]
        public float WaypointTol = 0.12f;
        [Group("Approach")]
        [Units("deg")]
        public float AlignYawTolDeg = 10f;
        [Group("Approach")]
        [Units("1/s")]
        public float LateralGain = 0.5f;            // strafe gain in final approach (0 = off)
        [Group("Approach")]
        [Units("m/s")]
        public float MaxStrafe = 0.05f;
        [Group("Approach")]
        [Units("m")]
        public float LateralTol = 0.03f;
        [Group("Approach")]
        [Units("m")]
        public float ReachAlongTol = 0.10f;         // station mode: box "in reach" window along approach
        [Group("Approach")]
        [Units("m")]
        public float ReachLateralTol = 0.08f;       // station mode: box "in reach" window sideways

        // ---- Motion --------------------------------------------------------
        [Group("Motion")]
        [Units("s")]
        [Slider(0.2f, 1.5f)]
        public float PreBendDuration = 0.345f;      // IK duration of the fold pose
        [Group("Motion")]
        public bool SingleStartTrigger = true;      // one IK Start trigger for both arms

        // ---- Fold ----------------------------------------------------------
        [Group("Fold")]
        [Units("m")]
        public float FoldForward = 0.05f;
        [Group("Fold")]
        [Units("m")]
        public float FoldDrop = 0.45f;
        [Group("Fold")]
        [Units("m")]
        public float FoldHalfGap = 0.20f;

        // ---- Pre-Grasp -----------------------------------------------------
        [Group("Pre-Grasp")]
        [Units("m")]
        public float HandStandoff = 0.03f;          // hands this far outside the side faces
        [Group("Pre-Grasp")]
        [Units("m")]
        public float MinHalfGap = 0.12f;            // clamp on hand distance from box centre line (pre-grasp)
        [Group("Pre-Grasp")]
        [Units("m")]
        public float MaxHalfGap = 0.26f;            // arm span limit; wider boxes are rejected
        [Group("Pre-Grasp")]
        [Units("m")]
        public float HandHeightOffset = 0f;
        [Group("Pre-Grasp")]
        [Units("s")]
        public float ApproachDuration = 0.8f;
        [Group("Pre-Grasp")]
        [Units("m")]
        public float RetargetThreshold = 0.01f;
        [Group("Pre-Grasp")]
        [Units("s")]
        public float RetargetInterval = 0.15f;
        [Group("Pre-Grasp")]
        [Units("s")]
        public float RetargetMoveDuration = 0.2f;
        [Group("Pre-Grasp")]
        [Units("m")]
        public float PositionTolerance = 0.02f;     // HandPoint error to count as in position
        [Group("Pre-Grasp")]
        [Units("s")]
        public float HoldTimeToConfirm = 0.5f;
        [Group("Pre-Grasp")]
        [Units("s")]
        public float PreGraspTimeout = 8f;
        [Group("Pre-Grasp")]
        [Units("m")]
        public float HandReach = 0.30f;             // elbow -> hand distance for the HandPoint proxy

        // ---- Corner Grasp --------------------------------------------------
        [Group("Corner Grasp")]
        public bool CornerGrasp = true;             // false = flat side squeeze, thumb not used on front face
        [Group("Corner Grasp")]
        [Units("m")]
        public float FrontInset = 0.05f;            // initial hand depth behind the front face
        [Group("Corner Grasp")]
        [Units("m")]
        public float MinFrontInset = -0.05f;
        [Group("Corner Grasp")]
        [Units("m")]
        public float MaxFrontInset = 0.20f;
        [Group("Corner Grasp")]
        public bool AutoAdjustInset = true;         // slide hands until the open thumb clears the front face
        [Group("Corner Grasp")]
        [Units("m")]
        public float ThumbClearance = 0.03f;        // desired open-thumb gap in front of the front face
        [Group("Corner Grasp")]
        [Units("1/s")]
        public float InsetGain = 3f;
        [Group("Corner Grasp")]
        [Units("m/s")]
        public float InsetMaxRate = 0.05f;
        [Group("Corner Grasp")]
        [Units("m")]
        public float ThumbPlaceTol = 0.015f;
        [Group("Corner Grasp")]
        [Units("m")]
        public float FaceClassifyTol = 0.03f;       // tolerance for "finger is on the intended face"
        [Group("Corner Grasp")]
        public bool RequireThumbOnFront = true;
        [Group("Corner Grasp")]
        public int MinSideFingersOnFace = 1;

        // ---- Thumb ---------------------------------------------------------
        [Group("Thumb")]
        [Units("1/s")]
        [Tooltip("Thumb curl speed while closing onto the front face.")]
        public float ThumbCloseRate = 0.4f;
        [Group("Thumb")]
        [Tooltip("Extra curl added once the thumb reaches the face - this is the thumb's pressure.")]
        public float ThumbPressCurl = 0.08f;
        [Group("Thumb")]
        [Units("m")]
        [Tooltip("Thumb counts as reached when thumb_2_link origin is within this of the front face plane (origin sits behind the pad).")]
        public float ThumbTouchTol = 0.02f;
        [Group("Thumb")]
        [Units("N")]
        [Tooltip("Tared thumb force that also counts as reached.")]
        public float ThumbTouchForce = 0.5f;
        [Group("Thumb")]
        [Units("m/s")]
        [Tooltip("Stall detection: tip moving slower than this near the face while curling = blocked by the box.")]
        public float ThumbStallSpeed = 0.005f;
        [Group("Thumb")]
        [Units("s")]
        public float ThumbStallTime = 0.25f;
        [Group("Thumb")]
        [Units("m")]
        public float ThumbStallZone = 0.05f;
        [Group("Thumb")]
        [Tooltip("After this much curl, the tip must have moved ThumbDirMinTravel toward the front face, else a direction error is logged.")]
        public float ThumbDirCheckCurl = 0.3f;
        [Group("Thumb")]
        [Units("m")]
        public float ThumbDirMinTravel = 0.005f;

        // ---- Contact Sensing -----------------------------------------------
        [Group("Contact Sensing")]
        [Units("N")]
        [Tooltip("Tared force above this = contact. Tare is captured just before the hands reach in, removing constant self-contact.")]
        public float ContactForceEps = 0.5f;
        [Group("Contact Sensing")]
        [Tooltip("With arms folded after startup, log any hand geom reporting contact (self-collision / belt).")]
        public bool IdleContactCheck = true;
        [Group("Contact Sensing")]
        [Units("s")]
        public float IdleCheckTime = 1.5f;

        // ---- Palm Contact --------------------------------------------------
        [Group("Palm Contact")]
        public float ContactHeightFraction = 0f;    // contact height as fraction of box half-height (negative = below COM)
        [Group("Palm Contact")]
        [Units("m")]
        public float ContactForwardOffset = 0f;
        [Group("Palm Contact")]
        [Units("m")]
        public float PalmSurfaceOffset = 0f;        // IK point -> palm surface, along the squeeze axis

        // ---- Wrist Lock ----------------------------------------------------
        [Group("Wrist Lock")]
        [Units("deg")]
        [Slider(-110.0f, 110.0f)]
        public float WristRollDeg = 0f;             // right hand mirrors roll and yaw
        [Group("Wrist Lock")]
        [Units("deg")]
        [Slider(-90.0f, 90.0f)]
        public float WristPitchDeg = 0f;
        [Group("Wrist Lock")]
        [Units("deg")]
        [Slider(-90.0f, 90.0f)]
        public float WristYawDeg = 0f;
        [Group("Wrist Lock")]
        [Units("deg/s")]
        public float WristLockRateDegPerSec = 60f;

        // ---- Fingers -------------------------------------------------------
        // Vector3 = joint 0, 1, 2 (Z unused for index/middle). Open pose of index/middle = 0.
        [Group("Fingers")]
        [Units("deg")]
        public Vector3 LeftThumbPreshapeDeg = new Vector3(0f, 0f, 0f);
        [Group("Fingers")]
        [Units("deg")]
        public Vector3 LeftThumbClosedDeg = new Vector3(0f, 30f, 70f);
        [Group("Fingers")]
        [Units("deg")]
        public Vector3 LeftIndexClosedDeg = new Vector3(-70f, -70f, 0f);
        [Group("Fingers")]
        [Units("deg")]
        public Vector3 LeftMiddleClosedDeg = new Vector3(-70f, -70f, 0f);
        [Group("Fingers")]
        public bool MirrorRightFingers = true;      // true: right = negated left (Right* fields ignored)
        [Group("Fingers")]
        [Units("deg")]
        public Vector3 RightThumbPreshapeDeg = new Vector3(0f, 0f, 0f);
        [Group("Fingers")]
        [Units("deg")]
        public Vector3 RightThumbClosedDeg = new Vector3(0f, -30f, -70f);
        [Group("Fingers")]
        [Units("deg")]
        public Vector3 RightIndexClosedDeg = new Vector3(70f, 70f, 0f);
        [Group("Fingers")]
        [Units("deg")]
        public Vector3 RightMiddleClosedDeg = new Vector3(70f, 70f, 0f);
        [Group("Fingers")]
        [Units("1/s")]
        public float FingerFreeCloseRate = 0.8f;    // curl speed before contact
        [Group("Fingers")]
        [Units("1/s")]
        public float FingerContactCloseRate = 0.2f; // curl speed once touching
        [Group("Fingers")]
        [Units("1/s")]
        public float FingerBackoffRate = 0.5f;
        [Group("Fingers")]
        [Units("N")]
        public float FingerTargetForce = 3f;        // index/middle force target
        [Group("Fingers")]
        [Units("N")]
        public float FingerForceBand = 1.0f;
        [Group("Fingers")]
        [Units("deg/s")]
        public float MaxFingerJointRateDegPerSec = 180f;

        // ---- Grasp ---------------------------------------------------------
        [Group("Grasp")]
        [Units("m")]
        public float FingerApproachGap = 0.01f;     // gap before the Contact stage; also the outward Gap clamp
        [Group("Grasp")]
        [Units("s")]
        public float GraspApproachDuration = 0.5f;
        [Group("Grasp")]
        [Units("m/s")]
        public float ContactAdvanceRate = 0.01f;    // palms creep inward at this speed until contact
        [Group("Grasp")]
        [Units("s")]
        public float ContactTimeout = 3f;
        [Group("Grasp")]
        public float GripSafetyFactor = 2f;
        [Group("Grasp")]
        [Units("m")]
        public float MaxSqueezeDepth = 0.02f;       // max palm interference into the box
        [Group("Grasp")]
        [Units("m")]
        public float MinSqueezeHalfGap = 0.02f;
        [Group("Grasp")]
        [Units("m/(N*s)")]
        public float DepthGain = 0.002f;            // squeeze admittance gain
        [Group("Grasp")]
        [Units("m/s")]
        public float MaxSqueezeRate = 0.02f;
        [Group("Grasp")]
        public float ForceTolFrac = 0.35f;          // side force band as fraction of target
        [Group("Grasp")]
        public float BalanceTolFrac = 0.3f;         // allowed |F_L - F_R| as fraction of target
        [Group("Grasp")]
        public bool RequirePalmContact = true;
        [Group("Grasp")]
        [Units("s")]
        public float GraspHoldTimeToConfirm = 0.5f;
        [Group("Grasp")]
        [Units("s")]
        public float SqueezeTimeout = 6f;
        [Group("Grasp")]
        [Units("m")]
        public float GraspRetargetThreshold = 0.002f;
        [Group("Grasp")]
        [Units("s")]
        public float GraspRetargetInterval = 0.08f;
        [Group("Grasp")]
        [Units("s")]
        public float GraspRetargetMoveDuration = 0.12f;

        // ---- Lift ----------------------------------------------------------
        [Group("Lift")]
        [Units("m")]
        public float LiftHeight = 0.10f;
        [Group("Lift")]
        [Units("m/s")]
        public float LiftSpeed = 0.03f;
        [Group("Lift")]
        [Units("s")]
        public float LiftSettleTime = 0.5f;
        [Group("Lift")]
        [Units("m")]
        public float SlipTolerance = 0.01f;         // palm rise - box rise above this = slipping
        [Group("Lift")]
        [Units("m")]
        public float DropTolerance = 0.05f;         // slip or drift above this = dropped
        [Group("Lift")]
        [Units("1/s")]
        public float SlipBoostRate = 0.5f;
        [Group("Lift")]
        public float MaxForceBoost = 2f;
        [Group("Lift")]
        [Units("s")]
        public float SlipGiveUpTime = 1.0f;
        [Group("Lift")]
        [Units("s")]
        public float ReleaseDuration = 0.6f;

        // ---- Moving Box ----------------------------------------------------
        [Group("Moving Box")]
        public bool EnableMovingCatch = true;
        [Group("Moving Box")]
        [Units("m/s")]
        public float MovingSpeedThreshold = 0.03f;
        [Group("Moving Box")]
        [Units("m")]
        public float CatchStopperGap = 0f;
        [Group("Moving Box")]
        [Units("m")]
        public float UpstreamRaiseClearance = 0.06f;
        [Group("Moving Box")]
        [Units("s")]
        public float CatchLowerDuration = 0.5f;
        [Group("Moving Box")]
        [Units("s")]
        public float CatchTimeout = 15f;
        [Group("Moving Box")]
        public float MovingBalanceTolFrac = 0.8f;

        // ---- Geoms ---------------------------------------------------------
        // Note: array fields do not serialize from the Inspector in the current engine
        // build, so these fallbacks may arrive empty at runtime. They are only used if
        // the geom scan in Setup.cs finds nothing.
        [Group("Geoms")]
        public int GeomScanLimit = 100;
        [Group("Geoms")]
        public uint[] LeftFallbackGeomIds = new uint[] { 54, 56, 58, 60, 62, 64, 66, 68 };
        [Group("Geoms")]
        public uint[] RightFallbackGeomIds = new uint[] { 84, 86, 88, 90, 92, 94, 96, 98 };

        // ---- Elbow ---------------------------------------------------------
        [Group("Elbow")]
        [Units("deg")]
        [Slider(-90.0f, 90.0f)]
        public float FoldedElbowDeg = -90f;

        // ---- Conveyor ------------------------------------------------------
        [Group("Conveyor")]
        public Entity? ConveyorEntity;              // local +X = belt direction
        [Group("Conveyor")]
        [Units("m")]
        public float ConveyorSurfaceY = 0.8f;
        [Group("Conveyor")]
        [Units("m")]
        public float ConveyorHalfWidth = 0.4f;
        [Group("Conveyor")]
        [Units("m")]
        public float ClearanceMargin = 0.05f;

        // ---- Safety --------------------------------------------------------
        [Group("Safety")]
        public bool EnableFailureHandling = true;
        [Group("Safety")]
        public int MaxRetries = 2;
        [Group("Safety")]
        [Units("s")]
        public float RecoverDuration = 1.0f;
        [Group("Safety")]
        [Units("deg/s")]
        public float MaxJointRateDegPerSec = 240f;
        [Group("Safety")]
        [Units("m")]
        public float MaxArmReach = 0f;              // 0 = off
        [Group("Safety")]
        [Units("m")]
        public float ShoulderForwardOffset = 0.05f;
        [Group("Safety")]
        [Units("m")]
        public float ShoulderLateralOffset = 0.18f;
        [Group("Safety")]
        [Units("m")]
        public float ShoulderHeightOffset = 0.35f;

        // ---- Policy --------------------------------------------------------
        [Group("Policy")]
        public uint WalkerSlotId = 1u;
        [Group("Policy")]
        public string PelvisBodyName = "pelvis";

        // ---- IK Input Names ------------------------------------------------
        [Group("IK Input Names")] public string LeftInputPrefix = "LeftArm_";
        [Group("IK Input Names")] public string RightInputPrefix = "RightArm_";
        [Group("IK Input Names")] public string DurationInput = "Duration";
        [Group("IK Input Names")] public string StartInput = "Start";

        // ---- Pole Hint -----------------------------------------------------
        [Group("Pole Hint")]
        [Slider(0.0f, 1.0f)]
        public float PoleWeight = 1.0f;
        [Group("Pole Hint")]
        [Units("m")]
        public Vector3 PoleOffset = new Vector3(-0.30f, 0.10f, 0.15f); // (forward, up, lateral) from pelvis

        // ---- Debug ---------------------------------------------------------
        [Group("Debug")]
        public bool LogWalkStatus = false;
        [Group("Debug")]
        public bool LogGraspStatus = true;

        // ==================================================================
        // Read-only status for other scripts
        // ==================================================================
        public bool IsPreGrasped => m_PreGraspAchieved;
        public bool IsHolding => m_Phase == Phase.Holding;
        public bool NeedsScoop => m_Phase == Phase.ScoopRequired;
        public bool IsFailed => m_Phase == Phase.Failed;
        public int Retries => m_Retries;
        public string LastFailure => m_LastFailure;
        public string PhaseName => m_Phase.ToString();
    }

}
