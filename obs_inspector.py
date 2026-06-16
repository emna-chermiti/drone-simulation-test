# obs_inspector.py
import gymnasium, PyFlyt.gym_envs, numpy as np

env = gymnasium.make("PyFlyt/QuadX-Hover-v4", render_mode=None, agent_hz=40)
obs, _ = env.reset()
print(f"Obs length: {len(obs)}")
print(f"Obs space: {env.observation_space}")
for i, v in enumerate(obs):
    print(f"  obs[{i:2d}] = {v:10.4f}")
env.close()