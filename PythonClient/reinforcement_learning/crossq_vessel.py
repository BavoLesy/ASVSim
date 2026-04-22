import setup_path
import argparse
import os
import subprocess
import time

import numpy as np
import wandb
from wandb.integration.sb3 import WandbCallback

from sb3_contrib import CrossQ
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback

from airgym.envs.vessel_env import PCGVesselEnv


class EpisodeEndCallback(BaseCallback):
    """Logs episode end reason, mean actions, and final goal distance to wandb."""

    _REASONS = ("collision", "goal_reached", "timeout")

    def __init__(self, verbose=0):
        super().__init__(verbose)
        self._ep_actions = []

    def _on_step(self) -> bool:
        actions = self.locals["actions"]  # shape (n_envs, 2)
        self._ep_actions.append(actions[0].copy())

        for info in self.locals["infos"]:
            reason = info.get("end_reason")
            if reason is not None:
                dx = info.get("distance_to_goal_x", 0.0)
                dy = info.get("distance_to_goal_y", 0.0)
                ep_actions = np.array(self._ep_actions)
                wandb.log(
                    {
                        **{f"episode/{r}": int(r == reason) for r in self._REASONS},
                        "episode/final_distance_to_goal": float(np.sqrt(dx**2 + dy**2)),
                        "episode/waypoints_reached": info.get("waypoints_reached", 0),
                        "episode/mean_thrust": float(ep_actions[:, 0].mean()),
                        "episode/mean_rudder_angle": float(ep_actions[:, 1].mean()),
                        "episode/count": info.get("episode_num", 0),
                    },
                    step=self.num_timesteps,
                )
                self._ep_actions = []
        return True


def make_env(ip_address, terrain_regen_interval, num_obstacles, num_dynamic_obstacles, num_waypoints, seed, sim_path, sim_wait, action_repeat):
    def _init():
        max_timesteps = 800 // action_repeat
        env = PCGVesselEnv(
            ip_address=ip_address,
            terrain_regen_interval=terrain_regen_interval,
            num_obstacles=num_obstacles,
            num_dynamic_obstacles=num_dynamic_obstacles,
            goal_distance=num_waypoints,
            max_timesteps=max_timesteps,
            seed=seed,
            sim_path=sim_path,
            sim_wait=sim_wait,
            action_repeat=action_repeat,
        )
        return Monitor(env)
    return _init


def start_simulator(sim_path="Blocks/Blocks.exe", wait=10, log_file=None):
    print(f"Starting simulator: {sim_path}")
    if log_file:
        log = open(log_file, "w")
        proc = subprocess.Popen(
            [sim_path],
            stdout=log,
            stderr=subprocess.STDOUT,
        )
    else:
        proc = subprocess.Popen(
            [sim_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    print(f"Simulator started (PID: {proc.pid}), waiting {wait}s for it to initialize...")
    time.sleep(wait)
    return proc


def main():
    parser = argparse.ArgumentParser(description="Train vessel RL agent with PCG environments")
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="Simulator IP address")
    parser.add_argument("--timesteps", type=int, default=2500000, help="Total training timesteps")
    parser.add_argument("--terrain-regen", type=int, default=10, help="Regenerate terrain every N episodes")
    parser.add_argument("--num-obstacles", type=int, default=4, help="Number of static obstacles per episode")
    parser.add_argument("--num-dynamic-obstacles", type=int, default=0, help="Number of dynamic (moving) obstacles per episode")
    parser.add_argument("--num-waypoints", type=int, default=1, help="Number of waypoints (1 or 2)")
    parser.add_argument("--sim-path", type=str, default="Blocks/Blocks.exe", help="Path to simulator executable")
    parser.add_argument("--sim-wait", type=int, default=10, help="Seconds to wait for simulator startup")
    parser.add_argument("--sim-log", action="store_true", default=False, help="Log simulator output to logs/sim.log")
    parser.add_argument("--action-repeat", type=int, default=1, help="Number of times to repeat each action")
    parser.add_argument("--seed", type=int, default=43, help="Random seed for reproducibility")
    parser.add_argument("--wandb-key", type=str, default=None, help="Weights & Biases API key (optional, disables wandb if not set)")
    args = parser.parse_args()

    # Create output directories
    os.makedirs("logs/training/models", exist_ok=True)
    os.makedirs("logs/training/tb", exist_ok=True)

    sim_log_path = "logs/sim.log" if args.sim_log else None
    sim_proc = start_simulator(args.sim_path, args.sim_wait, sim_log_path)

    # wandb setup (optional)
    use_wandb = args.wandb_key is not None
    if use_wandb:
        wandb.login(key=args.wandb_key)
        run = wandb.init(
            project="vessel-rl",
            config={
                "algorithm": "CrossQ",
                "total_timesteps": args.timesteps,
                "terrain_regen_interval": args.terrain_regen,
                "num_obstacles": args.num_obstacles,
                "num_waypoints": args.num_waypoints,
                "num_dynamic_obstacles": args.num_dynamic_obstacles,
                "learning_rate": 0.0003,
                "batch_size": 256,
                "buffer_size": 500000,
                "gamma": 0.99,
                "seed": args.seed,
            },
            sync_tensorboard=True,
        )

    env = DummyVecEnv([make_env(args.ip, args.terrain_regen, args.num_obstacles, args.num_dynamic_obstacles, args.num_waypoints, args.seed, args.sim_path, args.sim_wait, args.action_repeat)])
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0)
    model = CrossQ(
        "MlpPolicy",
        env,
        learning_rate=0.0003,
        gamma=0.99,
        verbose=1,
        batch_size=256,
        buffer_size=500000,
        learning_starts=5000,
        train_freq=1,
        stats_window_size=10,
        seed=args.seed,
        device="auto",
        tensorboard_log="logs/training/tb/",
        policy_kwargs=dict(net_arch=[512, 512]),
    )

    callbacks = []
    if use_wandb:
        callbacks.append(WandbCallback(verbose=2))
    callbacks.append(EpisodeEndCallback())
    callbacks.append(CheckpointCallback(
        save_freq=25000,
        save_path="logs/training/models/",
        name_prefix="crossq_pcg_vessel",
    ))

    model.learn(
        total_timesteps=args.timesteps,
        tb_log_name="crossq_pcg_vessel_" + str(int(time.time())),
        callback=callbacks,
    )

    model.save("logs/training/models/crossq_pcg_vessel_policy")
    print("Training complete. Model saved to logs/training/models/")

    if use_wandb:
        run.finish()

    sim_proc.terminate()
    sim_proc.wait()


if __name__ == "__main__":
    main()
