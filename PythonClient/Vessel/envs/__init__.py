from gymnasium.envs.registration import register
from Vessel.envs.Shipsim_gym import ShippingSim

register(
    id="ship-sim-v0",
    entry_point="Vessel.envs:ShippingSim",
)
