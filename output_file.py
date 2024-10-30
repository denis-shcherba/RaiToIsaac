import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different single-arm manipulators.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import numpy as np
import torch

import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

##
# Pre-defined configs
##
# isort: off
from omni.isaac.lab_assets import (
    FRANKA_PANDA_CFG,
)

# isort: on


def design_scene() -> tuple[dict, list[float]]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Single origin with Franka Panda
    origin = [.0, .0, .0]

    prim_utils.create_prim("/World/Origin1", "Xform", translation=origin)


    # Table
    cfg_table = sim_utils.MeshCuboidCfg(
        size=(2.5, 2.5, .1),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.3,.3,.3)),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        physics_material=sim_utils.RigidBodyMaterialCfg(),
        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True)
    )
    
    cfg_table.func("/World/Origin1/Table", cfg_table, translation=(.0, .0, .6))

    # Test cube
    cfg_cuboid = sim_utils.MeshCuboidCfg(
        size=(0.06, 0.06, 0.06),
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(1.0, 0.0, 0.0)
        ),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            kinematic_enabled=False
        ),
        physics_material=sim_utils.RigidBodyMaterialCfg(),
        collision_props=sim_utils.CollisionPropertiesCfg(
            collision_enabled=True
        )
    )

    cfg_cuboid.func(
        "/World/Origin1/Test",
        cfg_cuboid,
        translation=(-0.35, 0.1, 1.0)
    )
    # Robot
    franka_arm_cfg = FRANKA_PANDA_CFG.replace(prim_path="/World/Origin1/Robot")
    franka_arm_cfg.init_state.pos = (0.0, 0.0, .65)
    franka_panda = Articulation(cfg=franka_arm_cfg)

    # Return the scene information
    scene_entities = {"franka_panda": franka_panda}
    return scene_entities, origin


def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):
    """Runs the simulation loop."""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    # Simulate physics
    while simulation_app.is_running():
        # reset
        if count % 200 == 0:
            # reset counters
            sim_time = 0.0
            count = 0
            # reset the scene entities
            for index, robot in enumerate(entities.values()):
                # root state
                root_state = robot.data.default_root_state.clone()
                root_state[:, :3] += origins[index]
                robot.write_root_state_to_sim(root_state)
                # set joint positions
                joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
                robot.write_joint_state_to_sim(joint_pos, joint_vel)
                # clear internal buffers
                robot.reset()
            print("[INFO]: Resetting robots state...")
        # apply random actions to the robots
        for robot in entities.values():
            # generate random joint positions
            joint_pos_target = robot.data.default_joint_pos + torch.randn_like(robot.data.joint_pos) * 0.1
            joint_pos_target = joint_pos_target.clamp_(
                robot.data.soft_joint_pos_limits[..., 0], robot.data.soft_joint_pos_limits[..., 1]
            )
            # apply action to the robot
            robot.set_joint_position_target(joint_pos_target)
            # write data to sim
            robot.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        for robot in entities.values():
            robot.update(sim_dt)


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
    # design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
