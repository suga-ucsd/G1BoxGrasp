using System;
using System.IO;
using System.Reflection;
using Hazel;
using Hazel.Data;

namespace G1Pipe
{
    // Episode runner for ACT data (RecordEpisodes): every catch -> place run is one Observer episode, and the script writes a sidecar of
    // what the Observer can't record (data_pipeline/README.html, section 3.3). Every episode, the first too, starts from the same
    // reset on the frame Begin sets the robot up (Play's qpos, fresh walker); one box of the scene's pool, picked with the episode's
    // seed, starts at BoxStart on the belt, the others wait in a row at ParkSpot (where they sit in the editor doesn't matter, as long
    // as no two overlap). Done or Failed (or
    // EpisodeTimeout) ends the episode, until EpisodeCount.
    public partial class G1BoxGrasp
    {
        [Group("Recording")]
        public bool RecordEpisodes = false; // needs a LuckyHub sign-in, and the scene's recorder set to MujocoFull with its cameras

        [Group("Recording")]
        public int EpisodeCount = 5;

        [Group("Recording")]
        public float EpisodeTimeout = 80f; // s, a run still going then ends the episode as failed (the Observer cuts episodes at 90)

        [Group("Recording")]
        public bool RandomizeBox = true; // the pool (every free box in the scene) is dealt like cards: each round of pool-size episodes uses every box once

        [Group("Recording")]
        public int Seed = 0; // seeds the deal and each episode's start jitter (0 = a seed from the clock, logged)

        [Group("Recording")]
        public Vector3 BoxStart = new Vector3(0.61614f, 0.734044f, 0.978122f); // world centre of the episode's box at the start (top of the belt)

        [Group("Recording")]
        public Vector2 StartJitter = new Vector2(0.1f, 0.02f); // m the start moves: up to this far down the belt (never up: BoxStart sits at its top end), +- this across it

        [Group("Recording")]
        public Vector3 ParkSpot = new Vector3(-2f, 0.1f, -1.5f); // world spot of the first box not in use; the rest line up 0.6 m apart along +Z

        [Group("Recording")]
        public string SidecarDirectory = ""; // runs go in run_<time> folders here ("" = ~/Documents/LuckyEngine/DataSidecars)

        /// <summary>Episode runner state: kept through the per-episode reset of everything else.</summary>
        private sealed class Recorder
        {
            public int Episode; // Observer episode index (sidecar numbering)
            public int BaseSeed,
                Box; // this episode's box (index into m_Boxes)
            public Vector2 Offset; // m this episode's start moved along the belt and across it
            public bool Begun;
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

        // Start frames while recording: the scene's qpos is saved on the first episode's frames, and every episode (the first too) starts
        // from it: walker off, MuJoCo reset (velocities, contacts), the saved qpos, the episode's boxes placed, walker on (fresh, as on
        // Play). Begin then sets the robot up as it does on Play.
        private void StartEpisodeState()
        {
            Recorder r = m_Rec;
            if (!RecordEpisodes)
                return;
            if (r.Episode == 0)
                m_Mujoco!.GetFullQpos(r.StartQpos = new float[m_Mujoco.ModelNq]); // scene bodies can join the model a few frames in
            m_Robot!.SetPolicyActive(RotatorSlotId, false);
            m_Robot.SetPolicyActive(WalkerSlotId, false);
            m_Mujoco!.ResetToInitialState();
            m_Mujoco.SetFullQpos(r.StartQpos!);
            PlaceBoxes();
            m_Mujoco.ReseedPolicyPDTargets();
            SetLocomotion(WalkerSlotId);
        }

        // The episode's box (picked with seed BaseSeed + episode; box 0 without RandomizeBox) goes to BoxStart, every other box to its own
        // spot in the row at ParkSpot, all upright as in the editor, so no two boxes ever start inside each other.
        private void PlaceBoxes()
        {
            Recorder r = m_Rec;
            FindBoxes();
            if (m_Boxes.Count == 0)
                return; // not in the model yet: Begin retries next frame
            if (r.BaseSeed == 0)
                r.BaseSeed = Seed != 0 ? Seed : Environment.TickCount & 0xffff | 1;
            int pool = m_Boxes.Count;
            var deal = new int[pool]; // this round's order: a seeded shuffle of the pool
            var shuffle = new System.Random(r.BaseSeed * 7919 + r.Episode / pool);
            for (int i = 0; i < pool; i++)
            {
                int j = shuffle.Next(i + 1);
                deal[i] = deal[j];
                deal[j] = i;
            }
            r.Box = RandomizeBox ? deal[r.Episode % pool] : 0;
            var rng = new System.Random(r.BaseSeed + r.Episode);
            r.Offset = new Vector2(
                StartJitter.X * (float)rng.NextDouble(),
                StartJitter.Y * (2f * (float)rng.NextDouble() - 1f)
            );
            Quaternion belt = ChuteEntity!.Transform.WorldRotationQuat;
            Vector3 n = belt * Vector3.Up,
                down = (n * n.Y - Vector3.Up).Normalized(); // down the belt, in its plane
            Vector3 start = BoxStart + down * r.Offset.X + Vector3.Cross(n, down) * r.Offset.Y;
            Quaternion toMj = m_Mujoco!.GetAxisHzToMj();
            Vector3 offset = m_Mujoco.GetSceneTranslation();
            for (int i = 0, slot = 0; i < m_Boxes.Count; i++)
            {
                Vector3 p =
                    toMj
                    * (
                        (i == r.Box ? start : ParkSpot + new Vector3(0f, 0f, 0.6f * slot++))
                        - offset
                    );
                uint a = m_Mujoco.GetJointQposAdr(m_Mujoco.GetBodyFreeJointIndex(m_Boxes[i].Body));
                float[] v = { p.X, p.Y, p.Z, toMj.W, toMj.X, toMj.Y, toMj.Z }; // upright: no rotation in the editor
                for (uint k = 0; k < 7; k++)
                    m_Mujoco.SetQposByIndex(a + k, v[k]);
            }
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
            if (r.Frame == 0)
                r.BoxStart = BoxCenter(r.Box);
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
                Log.Info(
                    $"[G1] recording {EpisodeCount} episodes{(RandomizeBox ? $" from a pool of {m_Boxes.Count} boxes, seed {r.BaseSeed}" : "")}, sidecars in {r.Dir}"
                );
            }
            Observer.BeginEpisode();
            r.Csv = new StreamWriter(Path.Combine(r.Dir, $"episode_{r.Episode:D6}.csv"));
            r.Csv.WriteLine(
                "frame,sim_time,task_index,waist_lock,cmd_vx,cmd_vy,cmd_yaw,probe_ctrl"
            );
            r.Frame = 0;
            r.Time = 0f;
            r.FailReason = "";
            r.RobotStart = Pelvis();
            r.Begun = true;
            Log.Info(
                $"[G1] episode {r.Episode} begins with {BoxTag(r.Box)} ({2f * Math.Max(m_Boxes[r.Box].Half.X, m_Boxes[r.Box].Half.Y):F2} m wide, {m_Mujoco!.GetBodyMass(m_Boxes[r.Box].Body):F2} kg), start moved {r.Offset.X:F3} m down the belt, {r.Offset.Y:F3} m across"
            );
        }

        // A box's entity tag (its MuJoCo body is lucky_body_<entity id>).
        private string BoxTag(int i) =>
            Scene
                .FindEntityByID(
                    ulong.Parse(
                        m_Mujoco!.GetBodyNameByIndex(m_Boxes[i].Body)!["lucky_body_".Length..]
                    )
                )
                ?.Tag
            ?? $"box {i}";

        // Ends the episode (Observer and sidecar header), then stops the recording after the last one or resets for the next.
        private void EndRecordedEpisode(bool success)
        {
            Recorder r = m_Rec;
            Observer.EndCurrentEpisode(success);
            r.Csv!.Dispose();
            Vector3 size = m_Boxes[r.Box].Half * 2f; // MuJoCo frame: x, y across, z up
            string V(Vector3 v) => FormattableString.Invariant($"[{v.X:F4}, {v.Y:F4}, {v.Z:F4}]");
            string planar =
                success && m_Box >= 0
                    ? FormattableString.Invariant($"{Flat(BoxCenter(m_Box) - m_Slot).Length():F4}")
                    : "null";
            File.WriteAllText(
                Path.Combine(r.Dir, $"episode_{r.Episode:D6}.json"),
                FormattableString.Invariant(
                    $$"""
                    {"episode": {{r.Episode}}, "seed": {{r.BaseSeed + r.Episode}}, "success": {{(
                        success ? "true" : "false"
                    )}}, "fail_reason": "{{r.FailReason.Replace("\\", "/").Replace("\"", "'")}}",
                     "box": {"tag": "{{BoxTag(r.Box)}}", "width": {{Math.Max(
                        size.X,
                        size.Y
                    ):F4}}, "depth": {{Math.Min(
                        size.X,
                        size.Y
                    ):F4}}, "height": {{size.Z:F4}}, "mass": {{m_Mujoco!.GetBodyMass(
                        m_Boxes[r.Box].Body
                    ):F4}}},
                     "box_start": {{V(r.BoxStart)}}, "robot_start": {{V(
                        r.RobotStart
                    )}}, "perturbation": "none", "start_offset": [{{r.Offset.X:F4}}, {{r.Offset.Y:F4}}],
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
        // next frame is a Start frame again: StartEpisodeState puts the scene back and Begin sets the robot up as on Play.
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
        }
    }
}
