#!/usr/bin/env bash
# End-to-end check without the engine: fake session -> qa -> le2act -> load + alignment/video checks -> 20 ACT steps
# -> eval_act against a stub luckyrobots Session. Usage: tests/selftest.sh path/to/g1_three_fingers.xml [workdir]
set -euo pipefail
here=$(cd "$(dirname "$0")/.." && pwd)
mjcf=${1:?path to g1_three_fingers.xml}
work=${2:-$(mktemp -d)}
cams=(head=HeadCam wrist_left=LeftWristCam wrist_right=RightWristCam)
mkdir -p "$work" && cd "$work"
rm -rf fake outputs
python "$here/make_fake_session.py" --out fake --episodes 3 --frames 120 --lag 1 --mjcf "$mjcf"
python "$here/qa.py" --session fake/session_fake --sidecar fake/sidecar_fake --mjcf "$mjcf" --cameras HeadCam LeftWristCam RightWristCam >/dev/null
python "$here/le2act.py" --session fake/session_fake --sidecar fake/sidecar_fake --cameras "${cams[@]}" --size 48x64 \
  --out fake/ds --repo-id local/fake_act > fake/convert.log 2>&1
PYTHONPATH="$here" python - <<PY
import av, numpy as np, pandas as pd
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from common import ACTION_NAMES, PROBE
ds = LeRobotDataset("local/fake_act", root="fake/ds")
assert (ds.num_episodes, ds.num_frames) == (2, 238), (ds.num_episodes, ds.num_frames)
side = pd.read_csv("fake/sidecar_fake/episode_000000.csv")
acts = np.stack([ds[i]["action"].numpy() for i in range(119)])
assert np.allclose(acts[:, ACTION_NAMES.index(PROBE)], side.probe_ctrl[:119], atol=1e-6), "actions misaligned"
src = [f.to_ndarray(format="rgb24") for f in av.open("fake/session_fake/videos/observation.images.HeadCam/chunk-000/file-000.mp4").decode(video=0)]
for t in (1, 60, 118):
    d = ds[t]["observation.images.head"].permute(1, 2, 0).numpy() * 255
    assert min((t - 1, t, t + 1), key=lambda k: np.abs(d - src[k]).mean()) == t, f"video frame {t} out of sync"
print("dataset: 2 episodes, actions aligned, video in sync")
PY
"$here/train_act.sh" smoke fake/ds cpu > fake/train.log 2>&1
PYTHONPATH="$here/tests/stub" python "$here/eval_act.py" --checkpoint outputs/act_g1_smoke_ds/checkpoints/last/pretrained_model \
  --cameras "${cams[@]}" --size 48x64 --episodes 1 --device cpu --csv fake/eval.csv | tail -1
echo "selftest passed in $work"
