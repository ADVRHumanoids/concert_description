# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This script demonstrates how to use the interactive scene interface to setup a scene with multiple prims.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p scripts/tutorials/02_scene/create_scene.py --num_envs 32

"""

"""Launch Isaac Sim Simulator first."""


import argparse
import sys

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# parse the arguments
args_cli, hydra_args = parser.parse_known_args()

# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import math
import time

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.assets.articulation import Articulation
from isaaclab.sensors.imu import Imu
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass

import os
import socket
import yaml

from concert_isaac.scene.simple_scene import ConcertBaseOnlyCfg
import xbot2_bridge as xb

##
# Pre-defined configs
##


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene, urdf_str: str):
    """Runs the simulation loop."""
    
    # Extract scene entities
    robot : Articulation = scene["robot"]
    imu_sensors: dict[str, Imu] = {}
    for sn, s in scene.sensors.items():
        print(f"Sensor name: {sn}, type: {type(s)}")
        if isinstance(s, Imu):
            print(f"IMU sensor found: {sn}")
            imu_sensors[sn] = s
    
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    time_sim = 0

    # Real-time factor tracking
    rtf_last_print = time.time()
    rtf_sim_elapsed = 0.0
    rtf_steps = 0

    # Setup XBot2 bridge
    bridge = xb.IsaacXBot2Bridge(robot, imu_sensors, urdf_str)
    
    # Simulation loop
    qinit = robot.data.default_joint_pos.clone()
    robot.write_joint_position_to_sim(qinit)

    while simulation_app.is_running():

        tic = time.time()

        # isaac <-> xbot2 communication
        bridge.send_to_clients(time_sim)
        bridge.recv_from_clients()
        

        # simulation loop
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        # track real-time factor
        time_sim += sim_dt
        rtf_sim_elapsed += sim_dt
        rtf_steps += 1
        count += 1

        # print real-time factor every ~1s
        now = time.time()
        real_elapsed = now - rtf_last_print
        if real_elapsed >= 1.0:
            real_time_factor = rtf_sim_elapsed / real_elapsed if real_elapsed > 0 else float('inf')
            print(
                f"[INFO] Real-time factor: {real_time_factor:.3f}x "
                f"(sim: {rtf_sim_elapsed:.3f}s, real: {real_elapsed:.3f}s, steps: {rtf_steps})"
            )
            rtf_last_print = now
            rtf_sim_elapsed = 0.0
            rtf_steps = 0

        # time delay for real-time evaluation
        toc = time.time()
        sleep_time = sim_dt - (toc - tic)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    
    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])
    
    # Design scene
    scene_cfg = ConcertBaseOnlyCfg()
    scene_cfg.scene.num_envs = args_cli.num_envs
    scene_cfg.scene.env_spacing = 2.0
    scene = InteractiveScene(scene_cfg.scene)
    
    # Play the simulator
    sim.reset()
    
    # Now we are ready!
    print("[INFO]: Setup complete...")
    
    # Run the simulator
    run_simulator(sim, scene, scene_cfg.urdf)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
