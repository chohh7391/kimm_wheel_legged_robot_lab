# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Ant locomotion environment (similar to OpenAI Gym Ant-v2).
"""

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

'''Driving Rough'''
gym.register(
    id="Kimm-Wheel-Legged-Robot-Velocity-Driving-Rough-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:KimmWheelLeggedRobotDrivingRoughEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:KimmWheelLeggedRobotDrivingRoughPPORunnerCfg",
    },
)

gym.register(
    id="Kimm-Wheel-Legged-Robot-Velocity-Driving-Rough-v1",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_fixed_hip_cfg:KimmWheelLeggedRobotDrivingFixedHipRoughEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:KimmWheelLeggedRobotDrivingRoughPPORunnerCfg",
    },
)

gym.register(
    id="Kimm-Wheel-Legged-Robot-Velocity-Driving-Rough-v2",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_fixed_hip_ankle_cfg:KimmWheelLeggedRobotDrivingFixedHipAnkleRoughEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:KimmWheelLeggedRobotDrivingRoughPPORunnerCfg",
    },
)




'''Driving Flat'''
gym.register(
    id="Kimm-Wheel-Legged-Robot-Velocity-Driving-Flat-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:KimmWheelLeggedRobotDrivingFlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:KimmWheelLeggedRobotDrivingFlatPPORunnerCfg",
    },
)

gym.register(
    id="Kimm-Wheel-Legged-Robot-Velocity-Driving-Flat-v1",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_fixed_hip_cfg:KimmWheelLeggedRobotDrivingFixedHipFlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:KimmWheelLeggedRobotDrivingFlatPPORunnerCfg",
    },
)

gym.register(
    id="Kimm-Wheel-Legged-Robot-Velocity-Driving-Flat-v2",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_fixed_hip_ankle_cfg:KimmWheelLeggedRobotDrivingFixedHipAnkleFlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:KimmWheelLeggedRobotDrivingFlatPPORunnerCfg",
    },
)