using System;
using System.IO;
using System.Reflection;
using Hazel;
using Hazel.Data;

namespace G1Pipe
{
    // Episode runner for ACT data (RecordEpisodes): every catch -> place run is one Observer episode, and the script writes a sidecar of
    // what the Observer can't record (data_pipeline/README.html, section 3.3). Done or Failed (or EpisodeTimeout) ends the episode;
    // the next one starts exactly as Play did (same scene state on the same frame, fresh walker), until EpisodeCount.
    public partial class G1BoxGrasp
    {
        [Group("Recording")]
        public bool RecordEpisodes = false; // needs a LuckyHub sign-in, and the scene's recorder set to MujocoFull with its cameras

        [Group("Recording")]
        public int EpisodeCount = 5;

        [Group("Recording")]
        public float EpisodeTimeout = 80f; // s, a run still going then ends the episode as failed (the Observer cuts episodes at 90)

        [Group("Recording")]
        public string SidecarDirectory = ""; // runs go in run_<time> folders here ("" = ~/Documents/LuckyEngine/DataSidecars)

        /// <summary>Episode runner state: kept through the per-episode reset of everything else.</summary>
        private sealed class Recorder
        {
            public int Episode; // Observer episode index (sidecar numbering)
            public bool Begun,
                Restore; // the next Start frame puts the scene back as Play started it
            public string Dir = "";
            public StreamWriter? Csv;
            public int Frame;
            public float Time;
            public string FailReason = "";
            public float[]? StartQpos; // the whole scene's qpos on the frame Play set the robot up (every episode starts from it)
            public Vector3 BoxStart,
                RobotStart;
        }

        private Recorder m_Rec = new Recorder();
        private static readonly string[] s_PhaseLabels =
        {
            "catch",
            "grasp",
            "lift",
            "carry",
            "place",
            "retract",
        };

        // Recording ends each run (no next box), so the episode ends with it.
        private bool Repeat => RepeatLoop && !RecordEpisodes;

        // Phase -> dataset label (task_index): catch, grasp, lift, carry, place, retract.
        private static int PhaseLabel(Phase p) =>
            p <= Phase.Catch ? 0
            : p <= Phase.Holding ? 1
            : p <= Phase.PullIn ? 2
            : p <= Phase.FaceTable ? 3
            : p == Phase.Place ? 4
            : 5;

        // Start frames while recording: the first episode saves the scene's qpos on the frame Begin sets the robot up; each later one
        // puts it back on that same frame, with the walker switched off and on (fresh, as on Play), so Begin runs as it did on Play.
        private void StartEpisodeState()
        {
            Recorder r = m_Rec;
            if (!RecordEpisodes)
                return;
            if (!r.Restore)
            {
                if (r.Episode == 0)
                    m_Mujoco!.GetFullQpos(r.StartQpos ??= new float[m_Mujoco.ModelNq]);
                return;
            }
            r.Restore = false;
            m_Robot!.SetPolicyActive(RotatorSlotId, false);
            m_Robot.SetPolicyActive(WalkerSlotId, false);
            m_Mujoco!.ResetToInitialState();
            m_Mujoco.SetFullQpos(r.StartQpos!);
            m_Mujoco.ReseedPolicyPDTargets();
            SetLocomotion(WalkerSlotId);
        }

        // End of every OnUpdate: begins an episode once the run is set up, writes this tick's sidecar row, ends the episode with the run.
        private void RecordTick(float ts)
        {
            Recorder r = m_Rec;
            if (!RecordEpisodes || m_Phase == Phase.Start || r.Episode >= EpisodeCount)
                return;
            if (!r.Begun)
            {
                BeginRecordedEpisode();
                return;
            }
            int label = PhaseLabel(m_Phase);
            Observer.SetCurrentTaskIndex(label);
            r.Csv!.WriteLine(
                FormattableString.Invariant(
                    $"{r.Frame},{r.Time:F4},{label},{(m_WaistLocked ? 1 : 0)},{m_Cmd.X:R},{m_Cmd.Y:R},{m_Cmd.Z:R},{m_Arms[0].Joints[4].Value:R}"
                )
            );
            r.Frame++;
            r.Time += ts;
            if (m_Phase == Phase.Done)
                EndRecordedEpisode(true);
            else if (m_Phase == Phase.Failed || r.Time >= EpisodeTimeout)
            {
                if (r.FailReason == "")
                    r.FailReason = $"timed out in {m_Phase}";
                EndRecordedEpisode(false);
            }
        }

        // Opens the session on the first episode (labels registered first), then the episode and its sidecar rows.
        private void BeginRecordedEpisode()
        {
            Recorder r = m_Rec;
            if (!Observer.IsRecording)
            {
                for (int i = 0; i < s_PhaseLabels.Length; i++)
                    Observer.RegisterTask(i, s_PhaseLabels[i]);
                Observer.StartRecording();
                if (!Observer.IsRecording)
                {
                    Log.Error(
                        "[G1] recording didn't start: sign in to LuckyHub and keep the Recorder runner enabled - RecordEpisodes off"
                    );
                    RecordEpisodes = false;
                    return;
                }
                string root =
                    SidecarDirectory != ""
                        ? SidecarDirectory
                        : Path.Combine(
                            Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                            "Documents",
                            "LuckyEngine",
                            "DataSidecars"
                        );
                r.Dir = Path.Combine(root, $"run_{DateTime.Now:yyyyMMdd_HHmmss}");
                Directory.CreateDirectory(r.Dir);
                Log.Info($"[G1] recording {EpisodeCount} episodes, sidecars in {r.Dir}");
            }
            Observer.BeginEpisode();
            r.Csv = new StreamWriter(Path.Combine(r.Dir, $"episode_{r.Episode:D6}.csv"));
            r.Csv.WriteLine(
                "frame,sim_time,task_index,waist_lock,cmd_vx,cmd_vy,cmd_yaw,probe_ctrl"
            );
            r.Frame = 0;
            r.Time = 0f;
            r.FailReason = "";
            r.BoxStart = BoxCenter(0);
            r.RobotStart = Pelvis();
            r.Begun = true;
            Log.Info($"[G1] episode {r.Episode} begins");
        }

        // Ends the episode (Observer and sidecar header), then stops the recording after the last one or resets for the next.
        private void EndRecordedEpisode(bool success)
        {
            Recorder r = m_Rec;
            Observer.EndCurrentEpisode(success);
            r.Csv!.Dispose();
            Vector3 size = m_Boxes[0].Half * 2f; // MuJoCo frame: x, y across, z up
            string V(Vector3 v) => FormattableString.Invariant($"[{v.X:F4}, {v.Y:F4}, {v.Z:F4}]");
            string planar =
                success && m_Box >= 0
                    ? FormattableString.Invariant($"{Flat(BoxCenter(m_Box) - m_Slot).Length():F4}")
                    : "null";
            File.WriteAllText(
                Path.Combine(r.Dir, $"episode_{r.Episode:D6}.json"),
                FormattableString.Invariant(
                    $$"""
                    {"episode": {{r.Episode}}, "seed": null, "success": {{(
                        success ? "true" : "false"
                    )}}, "fail_reason": "{{r.FailReason.Replace("\\", "/").Replace("\"", "'")}}",
                     "box": {"tag": "box0", "width": {{Math.Max(
                        size.X,
                        size.Y
                    ):F4}}, "depth": {{Math.Min(
                        size.X,
                        size.Y
                    ):F4}}, "height": {{size.Z:F4}}, "mass": {{m_Mujoco!.GetBodyMass(
                        m_Boxes[0].Body
                    ):F4}}},
                     "box_start": {{V(r.BoxStart)}}, "robot_start": {{V(
                        r.RobotStart
                    )}}, "perturbation": "none",
                     "release_error": {"height": null, "planar": {{planar}}}, "max_slip": null, "max_wrist_dev_deg": null, "seconds": {{r.Time:F2}}}

                    """
                )
            );
            Log.Info(
                $"[G1] episode {r.Episode} {(success ? "succeeded" : $"failed ({r.FailReason})")} after {r.Time:F1} s, {r.Frame} ticks"
            );
            r.Episode++;
            r.Begun = false;
            if (r.Episode >= EpisodeCount)
            {
                Observer.StopRecording();
                Log.Info($"[G1] recording stopped after {r.Episode} episodes, sidecars in {r.Dir}");
                return;
            }
            ResetEpisode();
        }

        // Every private field back to its initial value (a fresh instance's; the inspector's public fields stay) and the walker on, so the
        // next frame is a Start frame again: StartEpisodeState puts the scene back there and Begin sets the robot up as on Play.
        private void ResetEpisode()
        {
            var fresh = new G1BoxGrasp();
            foreach (
                FieldInfo f in typeof(G1BoxGrasp).GetFields(
                    BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.DeclaredOnly
                )
            )
                if (f.Name is not (nameof(m_Robot) or nameof(m_Mujoco) or nameof(m_Rec)))
                    f.SetValue(this, f.GetValue(fresh));
            SetLocomotion(WalkerSlotId);
            m_Rec.Restore = true;
        }
    }
}
