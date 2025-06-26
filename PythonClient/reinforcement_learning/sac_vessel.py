import setup_path
import gymnasium as gym
import airgym
import time

from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback

# Create a DummyVecEnv for main airsim gym env
env = gym.make("airgym:airsim-vessel-sample-v0")

# Initialize SAC algorithm
model = SAC(
    "MlpPolicy",
    env,
    learning_rate=0.0003,
    verbose=1,
    batch_size=32,
    buffer_size=4000,
    learning_starts=500,
    train_freq=1,
    tau=0.010,
    target_entropy=-2,
    stats_window_size=10,
    device="auto",
    tensorboard_log="./sac_vessel_tb/",
)

# Create an evaluation callback with the same env, called every 5000 iterations
callbacks = []
eval_callback = EvalCallback(
    env,
    callback_on_new_best=None,
    n_eval_episodes=5,
    best_model_save_path=".",
    log_path=".",
    eval_freq=5000,
)
callbacks.append(eval_callback)

kwargs = {}
kwargs["callback"] = callbacks

# Train for a certain number of timesteps
model.learn(
    total_timesteps=25000, 
    tb_log_name="sac_airsim_vessel_run_" + str(time.time()), 
    **kwargs
)

# Save policy weights
model.save("sac_airsim_vessel_policy") 