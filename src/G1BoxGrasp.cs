using System;
using Hazel;

namespace G1Pipe
{
    // Picks up each box resting on the conveyor with both hands, places it on the table to the left, and comes back for the next.
    // FindBox -> FaceBox -> WalkToBox -> FaceChute -> Approach -> Ready -> PreGrasp -> Squeeze -> (DragIn) -> Lift -> PullIn -> TurnToTable -> WalkToTable -> FaceTable -> Place -> Release -> Retract -> FindBox ...
    // CatchBox: a box sliding down the tilted conveyor is tracked from the start; FindBox -> Catch puts the downhill hand's palm in its path
    // ahead of where the box will be by the time the hand gets there, and the box stops against it. The robot catches from where it stands.
    // Caught, it goes straight into the grasp: the stopper holds still on the box (m_Holder) while the other hand reaches in front of it and
    // slides in along its side face (PreGrasp), then both squeeze and the usual Lift -> ... follows.
    // The "Phases" checkboxes switch groups of phases off for testing. Every phase has a progress watchdog: a stall triggers a corrective
    // retry (step closer, re-squeeze, re-turn); when those run out the attempt fails (Recover) and the next box is tried; a box that fails
    // MaxBoxAttempts times is skipped until it moves. Where everything lives: src/README.md.
    public partial class G1BoxGrasp : Entity
    {
        private enum Phase
        {
            Start,
            FindBox,
            FaceBox,
            WalkToBox,
            FaceChute,
            Approach,
            Ready,
            Catch,
            PreGrasp,
            Squeeze,
            DragIn,
            Holding,
            Lift,
            PullIn,
            BackOff,
            TurnToTable,
            WalkToTable,
            FaceTable,
            Place,
            Release,
            Retract,
            Recover,
            WalkerTest,
            ArmTest,
            Done,
            Failed,
        }

        // Components, and the walker on legs + waist from the first step so the robot stands (the rest waits for MuJoCo in Begin).
        protected override void OnCreate()
        {
            m_Robot = GetComponent<RobotControllerComponent>();
            m_Mujoco = GetComponent<MujocoSceneComponent>();
            if (m_Robot == null || m_Mujoco == null)
                Log.Error(
                    $"[G1] '{Tag}' needs a RobotControllerComponent and a MujocoSceneComponent - script does nothing"
                );
            else
                SetLocomotion(WalkerSlotId);
        }

        // Runs the current phase; the robot stands still unless the phase walks or turns.
        protected override void OnUpdate(float ts)
        {
            if (m_Robot == null || m_Mujoco == null)
                return;
            m_Dt = ts;
            if (m_PelvisId != uint.MaxValue)
            {
                m_Still = BodyMotion() < StillSpeed ? m_Still + ts : 0f;
                TrackCarryFrame(ts);
                float pole =
                    m_Phase >= Phase.PreGrasp && m_Phase <= Phase.Release ? GripPoleOut : 0f;
                m_PoleOut += Mathf.Clamp(pole - m_PoleOut, -k_PoleRate * ts, k_PoleRate * ts); // eased, so the arms re-fold slowly and never jolt the box
            }
            m_PhaseTime += ts;
            m_StepTime += ts;
            m_SinceBest += ts;
            m_WatchTime += ts;
            if (
                LogInterval > 0f
                && m_Phase != Phase.Start
                && m_PelvisId != uint.MaxValue
                && (m_LogTimer += ts) >= LogInterval
            )
            {
                m_LogTimer = 0f;
                LogStatus();
            }
            Drive(0f, 0f, 0f);

            switch (m_Phase)
            {
                case Phase.Start:
                    StartEpisodeState();
                    Begin();
                    break;
                case Phase.FindBox:
                    SendHands(Fold, false);
                    if (ArmTest)
                    {
                        int seed =
                            ArmTestSeed != 0 ? ArmTestSeed : Environment.TickCount & 0x7fffffff;
                        m_Rng = new System.Random(seed);
                        m_ArmIdx = 0;
                        m_ArmResults.Clear();
                        Log.Info(
                            $"[G1] arm test: {(ArmTestSweep ? $"{s_ArmSweep.Length} sweep targets (height, distance, separation, offset)" : $"{ArmTestTrials} random reachable target pairs, seed {seed}")}, no grasp"
                        );
                        Advance(Phase.ArmTest);
                        break;
                    }
                    if (WalkerTest)
                    {
                        m_TestIdx = 0;
                        Log.Info($"[G1] walker test: {s_WalkerTests.Length} commands, no grasp");
                        Advance(Phase.WalkerTest);
                        break;
                    }
                    m_Box = CatchBox ? IncomingBox() : -1;
                    if (m_Box >= 0)
                    {
                        EnterCatch();
                        break;
                    }
                    m_Box = RestingBox();
                    if (m_Box < 0)
                        break; // nothing to pick yet: the status line lists every box and why
                    m_Reach = StandDistance;
                    m_SqueezeTries = m_WalkRetries = 0;
                    m_Stand = StandFor(
                        ChuteEntity!,
                        BoxCenter(m_Box),
                        m_Reach,
                        FaceOf(ChuteEntity!)
                    );
                    Log.Info(
                        $"[G1] picking box {m_Box} at {BoxCenter(m_Box)}, {Flat(BoxCenter(m_Box) - Pelvis()).Length():F2} m from the pelvis{(WalkToBox ? $", standing at {m_Stand}" : "")}"
                    );
                    if (WalkToBox)
                        Advance(Phase.FaceBox);
                    else
                        EnterApproach();
                    break;
                case Phase.FaceBox:
                    SendHands(Fold, false);
                    if (
                        (m_Step == 0 && Flat(m_Stand - Pelvis()).Length() < k_NearRadius)
                        || Turn(YawOf(m_Stand - Pelvis()))
                    )
                        Advance(Phase.WalkToBox);
                    break;
                case Phase.WalkToBox:
                    SendHands(Fold, false);
                    if (WalkTo(m_Stand, ArriveTolerance, true))
                        Advance(Phase.FaceChute);
                    else if (Stalled(m_ToGo, k_WalkProgress, WalkTimeout))
                        WalkStalled(Phase.FaceBox);
                    break;
                case Phase.FaceChute:
                    SendHands(Fold, false);
                    if (Turn(YawOf(FaceOf(ChuteEntity!))))
                        EnterApproach();
                    break;
                case Phase.Approach:
                    StepApproach();
                    break;
                case Phase.Ready:
                    StepReady();
                    break;
                case Phase.Catch:
                    StepCatch(ts);
                    break;
                case Phase.PreGrasp:
                    StepPreGrasp(ts);
                    break;
                case Phase.Squeeze:
                    StepSqueeze(ts);
                    break;
                case Phase.DragIn:
                    StepDragIn(ts);
                    break;
                case Phase.Holding:
                    Hold(ts);
                    break; // test end state: keep the grip
                case Phase.Lift:
                    StepLift(ts);
                    break;
                case Phase.PullIn:
                    if (
                        Hold(ts)
                        && MoveTowards(
                            ref m_Center,
                            new Vector3(CarryX(), CarryOffset.Y, CarryOffset.Z),
                            k_CarrySpeed * ts
                        )
                    )
                        EnterTurnToTable();
                    break;
                case Phase.BackOff:
                    if (!Hold(ts))
                        break;
                    if (m_Step == 2)
                        HoldStation(); // the back-off spot, once backed away
                    if (StepBackOff())
                        Advance(Phase.TurnToTable);
                    break;
                case Phase.TurnToTable:
                    RaiseOverTable(ts);
                    if (!Hold(ts))
                        break;
                    bool turned = WalkerTurn(YawOf(FaceOf(PlaceTarget!)), k_CarryTurnTol);
                    HoldStation(); // on the back-off spot, turns included
                    if (turned)
                        Advance(Phase.WalkToTable);
                    break;
                case Phase.WalkToTable:
                    RaiseOverTable(ts);
                    if (!Hold(ts))
                        break;
                    bool walked = CarryWalk();
                    if (m_Step != 1)
                        HoldStation(); // the back-off spot before the walk, the stand spot after it
                    if (walked)
                        Advance(Phase.FaceTable);
                    break;
                case Phase.FaceTable:
                    RaiseOverTable(ts);
                    if (!Hold(ts))
                        break;
                    bool faced = WalkerTurn(YawOf(FaceOf(PlaceTarget!)), TurnTolerance);
                    HoldStation(); // on the stand spot, turns included
                    if (faced)
                        Advance(PlaceBox ? Phase.Place : Phase.Holding);
                    break;
                case Phase.Place:
                    StepPlace(ts);
                    break;
                case Phase.Release:
                    StepRelease(ts);
                    break;
                case Phase.Retract:
                    StepRetract();
                    break;
                case Phase.Recover:
                    StepRecover();
                    break;
                case Phase.WalkerTest:
                    StepWalkerTest();
                    break;
                case Phase.ArmTest:
                    StepArmTest();
                    break;
                default:
                    SendHands(Fold, false);
                    break; // Done / Failed: stand with the arms folded
            }
            if (m_Phase == Phase.Start)
                return;
            OrientWrists(ts);
            HoldWaist();
            PoseHands(ts);
            RecordTick(ts);
        }
    }
}
