"""Runs a trained ACT checkpoint closed-loop in LuckyEngine over gRPC and scores it. The scene's G1ActAgent publishes
observation.state (37) + done, success, phase, and applies the 35-value action; the External Action Gate steps the
simulation once per call, so the loop needs no timing of its own.

    python eval_act.py --checkpoint outputs/act_g1/checkpoints/last/pretrained_model --episodes 50 \
        --cameras head=head_camera wrist_left=left_wrist_camera wrist_right=right_wrist_camera --size 256x256 --csv eval.csv
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import pandas as pd

from common import ACTION_NAMES, PHASES, ROBOT_TYPE, STATE_NAMES, STATUS_NAMES, TASK


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", required=True, help="pretrained_model folder written by lerobot-train")
    ap.add_argument("--cameras", nargs="+", required=True, help="key=scene camera entity, keys as in the dataset")
    ap.add_argument("--size", default="256x256", help="HxW the engine renders the cameras at (the training size)")
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--max-steps", type=int, default=4000, help="per episode (80 s at 50 Hz)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=50051)
    ap.add_argument("--robot", default="unitree_g1", help="robot name the engine knows the agent by")
    ap.add_argument("--ensemble", type=float, default=0.01, help="temporal ensembling coefficient (0 = run whole chunks)")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--csv", default="eval_act.csv")
    a = ap.parse_args()
    cams = dict(c.split("=", 1) for c in a.cameras)
    h, w = map(int, a.size.split("x"))

    import torch
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.utils import prepare_observation_for_inference
    from luckyrobots import Session

    # Policy: temporal ensembling queries a chunk every step and blends the overlaps (smooth arms on a held box)
    cfg = PreTrainedConfig.from_pretrained(a.checkpoint)
    cfg.device = a.device
    if a.ensemble > 0:
        cfg.temporal_ensemble_coeff, cfg.n_action_steps = a.ensemble, 1
    policy = ACTPolicy.from_pretrained(a.checkpoint, config=cfg)
    pre, post = make_pre_post_processors(policy.config, pretrained_path=a.checkpoint)
    device = torch.device(a.device)

    # Engine: connect to the running editor's server, render the cameras at the training size
    session = Session(host=a.host, port=a.port)
    session.connect(robot=a.robot)
    session.configure_cameras([{"name": n, "width": w, "height": h} for n in cams.values()])
    ns, nst = len(STATE_NAMES), len(STATUS_NAMES)

    results = []
    for ep in range(a.episodes):
        policy.reset()
        resp, t0, phase_max = session.reset(), time.perf_counter(), 0
        for step in range(a.max_steps):
            obs = np.asarray(resp.observation, np.float32)
            if len(obs) != ns + nst:
                print(f"agent publishes {len(obs)} values, expected {ns} state + {nst} status", file=sys.stderr)
                return 1
            done, success, phase = obs[ns:]
            phase_max = max(phase_max, int(phase))
            if done > 0.5:
                break
            frames = {f.name: np.frombuffer(f.data, np.uint8).reshape(f.height, f.width, f.channels)[..., :3] for f in resp.camera_frames}
            if set(cams.values()) - set(frames):
                print(f"camera frames missing: {set(cams.values()) - set(frames)}", file=sys.stderr)
                return 1
            raw = {"observation.state": obs[:ns], **{f"observation.images.{k}": frames[n].copy() for k, n in cams.items()}}
            batch = pre(prepare_observation_for_inference(raw, device, TASK, ROBOT_TYPE))
            with torch.inference_mode():
                action = post(policy.select_action(batch))
            action = action.squeeze(0).cpu().numpy().astype(float)
            if len(action) != len(ACTION_NAMES):
                print(f"policy gives {len(action)} actions, the agent takes {len(ACTION_NAMES)}", file=sys.stderr)
                return 1
            resp = session.step(action.tolist())
        results.append({"episode": ep, "success": bool(success > 0.5), "done": bool(done > 0.5), "steps": step + 1,
                        "phase_reached": PHASES[min(phase_max, len(PHASES) - 1)], "wall_s": time.perf_counter() - t0})
        r = results[-1]
        print(f"episode {ep:3d}: {'success' if r['success'] else 'fail'} after {r['steps']} steps, reached {r['phase_reached']}")
        session.report_progress(task_name="g1_box", policy_name="act", phase="eval", current_episode=ep + 1, total_episodes=a.episodes,
                                status_text=f"success {np.mean([x['success'] for x in results]):.0%}", finished=ep + 1 == a.episodes)
    df = pd.DataFrame(results)
    df.to_csv(a.csv, index=False)
    print(f"\nsuccess {df['success'].mean():.0%} over {len(df)} episodes; phase reached: {df['phase_reached'].value_counts().to_dict()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
