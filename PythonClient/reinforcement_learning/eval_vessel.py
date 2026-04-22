"""Evaluate a trained CrossQ checkpoint in the simulator.

Usage:
    python eval_vessel.py --checkpoint logs/training/models/crossq_pcg_vessel_25000_steps.zip
    python eval_vessel.py --checkpoint logs/training/models/crossq_pcg_vessel_policy.zip --episodes 200
"""

import setup_path
import argparse
import time

import numpy as np
from sb3_contrib import CrossQ
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from airgym.envs.vessel_env import PCGVesselEnv


def make_env(args):
    def _init():
        env = PCGVesselEnv(
            ip_address=args.ip,
            terrain_regen_interval=args.terrain_regen,
            num_obstacles=args.num_obstacles,
            num_dynamic_obstacles=args.num_dynamic_obstacles,
            goal_distance=args.num_waypoints,
            max_timesteps=800 // args.action_repeat,
            seed=args.seed,
            sim_path=args.sim_path,
            sim_wait=args.sim_wait,
            action_repeat=args.action_repeat,
        )
        return Monitor(env)
    return _init


def main():
    parser = argparse.ArgumentParser(description="Evaluate a trained CrossQ checkpoint")
    parser.add_argument("--checkpoint", type=str, required=True, help="Path to model .zip file")
    parser.add_argument("--episodes", type=int, default=10, help="Number of evaluation episodes")
    parser.add_argument("--ip", type=str, default="127.0.0.1")
    parser.add_argument("--terrain-regen", type=int, default=10)
    parser.add_argument("--num-obstacles", type=int, default=4)
    parser.add_argument("--num-dynamic-obstacles", type=int, default=0)
    parser.add_argument("--num-waypoints", type=int, default=1)
    parser.add_argument("--sim-path", type=str, default="Blocks/Blocks.exe")
    parser.add_argument("--sim-wait", type=int, default=10)
    parser.add_argument("--action-repeat", type=int, default=1)
    parser.add_argument("--seed", type=int, default=43)
    parser.add_argument("--deterministic", action="store_true", default=True,
                        help="Use deterministic actions (no exploration noise)")
    args = parser.parse_args()

    env = DummyVecEnv([make_env(args)])
    env = VecNormalize(env, norm_obs=True, norm_reward=False, clip_obs=10.0)

    model = CrossQ.load(args.checkpoint, env=env, device="auto")
    print(f"Loaded checkpoint: {args.checkpoint}")

    # --- Run evaluation episodes ---
    results = []
    for ep in range(args.episodes):
        obs = env.reset()
        done = False
        total_reward = 0.0
        steps = 0

        while not done:
            action, _ = model.predict(obs, deterministic=args.deterministic)
            obs, reward, done, info = env.step(action)
            total_reward += reward[0]
            steps += 1

        info = info[0]  # unwrap from vec env
        end_reason = info.get("end_reason", "unknown")
        waypoints = info.get("waypoints_reached", 0)
        dist_x = info.get("distance_to_goal_x", 0.0)
        dist_y = info.get("distance_to_goal_y", 0.0)
        final_dist = np.sqrt(dist_x**2 + dist_y**2)

        results.append({
            "episode": ep + 1,
            "reward": total_reward,
            "steps": steps,
            "end_reason": end_reason,
            "waypoints_reached": waypoints,
            "final_distance": final_dist,
        })
        print(f"[Ep {ep+1:3d}] {end_reason:14s} | reward={total_reward:8.1f} | "
              f"steps={steps:4d} | wps={waypoints} | dist={final_dist:.1f}m")

    # --- Summary ---
    rewards = [r["reward"] for r in results]
    distances = [r["final_distance"] for r in results]
    successes = sum(1 for r in results if r["end_reason"] == "goal_reached")
    collisions = sum(1 for r in results if r["end_reason"] == "collision")
    timeouts = sum(1 for r in results if r["end_reason"] == "timeout")

    print("\n" + "=" * 60)
    print(f"Checkpoint : {args.checkpoint}")
    print(f"Episodes   : {args.episodes}")
    print(f"Success    : {successes}/{args.episodes} ({100*successes/args.episodes:.0f}%)")
    print(f"Collision  : {collisions}/{args.episodes} ({100*collisions/args.episodes:.0f}%)")
    print(f"Timeout    : {timeouts}/{args.episodes} ({100*timeouts/args.episodes:.0f}%)")
    print(f"Reward     : {np.mean(rewards):.1f} +/- {np.std(rewards):.1f}")
    print(f"Final dist : {np.mean(distances):.1f} +/- {np.std(distances):.1f} m")
    print("=" * 60)


if __name__ == "__main__":
    main()
