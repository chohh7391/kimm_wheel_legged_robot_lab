# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

# Configuration for PPO training on rough terrain
@configclass
class KimmWheelLeggedRobotDrivingRoughPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 200000
    save_interval = 100
    experiment_name = "kimm_wheel_legged_robot/driving/rough"
    empirical_normalization = False
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )

@configclass
class KimmWheelLeggedRobotDrivingFixedHipRoughPPORunnerCfg(KimmWheelLeggedRobotDrivingRoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()
        self.experiment_name = "kimm_wheel_legged_robot/driving/fixed_hip/rough"

@configclass
class KimmWheelLeggedRobotDrivingFixedHipAnkleRoughPPORunnerCfg(KimmWheelLeggedRobotDrivingRoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()
        self.experiment_name = "kimm_wheel_legged_robot/driving/fixed_hip_ankle/rough"


# Configuration for PPO training on flat terrain
@configclass
class KimmWheelLeggedRobotDrivingFlatPPORunnerCfg(KimmWheelLeggedRobotDrivingRoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 100000
        self.experiment_name = "kimm_wheel_legged_robot/driving/flat"

@configclass
class KimmWheelLeggedRobotDrivingFixedHipFlatPPORunnerCfg(KimmWheelLeggedRobotDrivingRoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 100000
        self.experiment_name = "kimm_wheel_legged_robot/driving/fixed_hip/flat"

@configclass
class KimmWheelLeggedRobotDrivingFixedHipAnkleFlatPPORunnerCfg(KimmWheelLeggedRobotDrivingRoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 100000
        self.experiment_name = "kimm_wheel_legged_robot/driving/fixed_hip_ankle/flat"
