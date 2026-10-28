"""Stand-in for luckyrobots.Session: 37 state + done/success/phase, RGBA frames, checks each action's size."""
from types import SimpleNamespace
import numpy as np


class Session:
    def __init__(self, host="127.0.0.1", port=50051):
        self.t, self.cams = 0, []

    def connect(self, robot=None, timeout_s=120.0):
        print(f"[stub] connect robot={robot}")

    def configure_cameras(self, cams):
        self.cams = cams
        print(f"[stub] cameras {[(c['name'], c['height'], c['width']) for c in cams]}")

    def _obs(self):
        done = self.t >= 30
        state = np.random.default_rng(self.t).normal(0, 0.1, 37)
        frames = [SimpleNamespace(name=c["name"], width=c["width"], height=c["height"], channels=4,
                                  data=np.full((c["height"], c["width"], 4), self.t % 256, np.uint8).tobytes()) for c in self.cams]
        return SimpleNamespace(observation=list(state) + [float(done), float(done), min(self.t // 6, 5)], camera_frames=frames)

    def reset(self, agent_name=""):
        self.t = 0
        return self._obs()

    def step(self, actions=None, agent_name="", action_groups=None):
        assert len(actions) == 35 and all(np.isfinite(actions)), len(actions)
        self.t += 1
        return self._obs()

    def report_progress(self, **kw):
        pass
