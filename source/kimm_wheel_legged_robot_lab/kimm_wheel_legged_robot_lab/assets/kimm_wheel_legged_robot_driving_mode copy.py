import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from .kimm_wheel_legged_robot import (
    ROBOT_DESCRIPTION_PATH,
    KIMM_WHEEL_LEGGED_ROBOT_CFG,
    BODY_JOINTS, HIP_JOINTS, KNEE_JOINTS, ANKLE_JOINTS, WHEEL_JOINTS,
)

DRIVING_MODE_HEIGHT = 0.47

# Robot Configurations
KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_CFG = KIMM_WHEEL_LEGGED_ROBOT_CFG.copy()
KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_CFG.init_state.pos = (0.0, 0.0, DRIVING_MODE_HEIGHT)
KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_CFG.init_state.joint_pos["left_knee_joint"] = 1.9
KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_CFG.init_state.joint_pos["right_knee_joint"] = -1.9
KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_CFG.actuators["wheels"] = ImplicitActuatorCfg(
    joint_names_expr=WHEEL_JOINTS,
    effort_limit_sim=120.0,
    velocity_limit_sim=5.02,
    stiffness=200.0,
    damping=5.0,
    friction=0.0,
)


# Fixed Hip version
KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_FIXED_HIP_CFG = KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_CFG.copy()
KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_FIXED_HIP_CFG.spawn.usd_path = f"{ROBOT_DESCRIPTION_PATH}/usd/kimm_wheel_legged_robot_fixed_hip/kimm_wheel_legged_robot_fixed_hip.usd"
# Remove hip joints
for hip_joint in HIP_JOINTS:
    KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_FIXED_HIP_CFG.init_state.joint_pos.pop(hip_joint, None)
KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_FIXED_HIP_CFG.actuators.pop("hips", None)


# Fixed Hip & Ankle version
KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_FIXED_HIP_ANKLE_CFG = KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_CFG.copy()
KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_FIXED_HIP_ANKLE_CFG.spawn.usd_path = f"{ROBOT_DESCRIPTION_PATH}/usd/kimm_wheel_legged_robot_fixed_hip_ankle/kimm_wheel_legged_robot_fixed_hip_ankle.usd"
# Remove hip joints
for hip_joint in HIP_JOINTS:
    KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_FIXED_HIP_ANKLE_CFG.init_state.joint_pos.pop(hip_joint, None)
# Remove ankle joints
for ankle_joint in ANKLE_JOINTS:
    KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_FIXED_HIP_ANKLE_CFG.init_state.joint_pos.pop(ankle_joint, None)
KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_FIXED_HIP_ANKLE_CFG.actuators.pop("hips", None)
KIMM_WHEEL_LEGGED_ROBOT_DRIVING_MODE_FIXED_HIP_ANKLE_CFG.actuators.pop("ankles", None)