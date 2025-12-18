import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg, DCMotorCfg, DelayedPDActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

import os 

CONCERT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{os.path.abspath(os.path.dirname(__file__))}/usd/concert_base_only_simplified/concert_base_only_simplified.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=10.0,
            max_angular_velocity=1000,
            max_depenetration_velocity=1.0,
            enable_gyroscopic_forces=True,
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
            fix_root_link=False,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.806), # height from floor when in homing
        # joint_pos={
        # }
    ),
    actuators={
        "wheel_motor": DCMotorCfg(
            joint_names_expr=["J_wheel_.*"],
            saturation_effort=24,
            effort_limit=24,
            velocity_limit=9.5,
            stiffness=0, 
            damping=90,
            armature=0.234,
            friction=2.68,
            dynamic_friction=2.68,
            viscous_friction=1.7,
        ),
        "steering_motor":  DCMotorCfg(
            joint_names_expr=["J1_.*"],
            saturation_effort=100,
            effort_limit=100,
            velocity_limit=7.5,
            stiffness=500, 
            damping=30,
            armature=0.234,
            friction=4.68,
            dynamic_friction=4.68,
            viscous_friction=1.7,
        ),
    },
    #TODO what these do?
    #soft_joint_pos_limit_factor=1.0,
)

CONCERT_URDF_PATH = f"{os.path.abspath(os.path.dirname(__file__))}/urdf/concert_base_only_simplified.urdf"

CONCERT_URDF = open(CONCERT_URDF_PATH, 'r').read()