# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates different single-arm manipulators.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p source/standalone/demos/arms.py

"""

"""Launch Isaac Sim Simulator first."""

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
import robotic as ry 

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
def config2config(C: ry.Config) -> tuple[dict, list[float]]:
    # TODO Camera, Origin maybe? (for different origins, actually not that important rn)7
    
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
        
    cfg = sim_utils.DomeLightCfg(
        intensity=700.0,
        color=(1.0, 0.98, 0.95)  # Warm, slightly yellow-white
    )
    cfg.func("/World/Light", cfg)

    origin = [.0, .0, .0]

    prim_utils.create_prim("/World/Origin", "Xform", translation=origin)

    tmp_config = ry.Config()
    tmp_config.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))

    frameNames = C.getFrameNames()     
    scene_entities = {}

    panda_init_joint_state = {
            "panda_joint1": 0,
            "panda_joint2": -0.5,
            "panda_joint3": 0.,
            "panda_joint4": -2.,
            "panda_joint5": 0.,
            "panda_joint6": 2.,
            "panda_joint7": -.5,
            "panda_finger_joint.*": 0.04,
    }

    for name in frameNames:

        frame = C.getFrame(name)

        if "table" in name:
            # Table (just works if table is in frameName)
            cfg_table = sim_utils.MeshCuboidCfg(
                size=tuple(frame.getSize()[:3]),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(frame.info()["color"])),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                physics_material=sim_utils.RigidBodyMaterialCfg(),
                collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True)
            )
    
            cfg_table.func(f"/World/Origin/name", cfg_table, translation=tuple(frame.getPosition()))

        elif "panda_base" in name:            
            # Panda
            franka_name = name
            franka_arm_cfg = FRANKA_PANDA_CFG.replace(prim_path=f"/World/Origin/{franka_name}")
            franka_arm_cfg.init_state.pos = tuple(frame.getPosition())
            franka_arm_cfg.init_state.rot = (1/np.sqrt(2), 0.0, 0.0, 1/np.sqrt(2))
            franka_arm_cfg.init_state.joint_pos = panda_init_joint_state
            franka_panda = Articulation(cfg=franka_arm_cfg)

            scene_entities[franka_name] = franka_panda

        elif "table" in name and frame.getParent().info()["name"]=="origin":
            pass
            

        # TODO mass, friction ? maybe
        elif not frame.getParent():
            ST = frame.getShapeType()

            if ST == ry.ST.box:
                cfg_cuboid = sim_utils.MeshCuboidCfg(
                    size=tuple(frame.getSize()),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color = tuple(frame.getMeshColors()[0][0:3]/255)),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
                    physics_material=sim_utils.RigidBodyMaterialCfg(),
                    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True)
                )

                cfg_cuboid.func("/World/Origin_obj/Cuboid", cfg_cuboid, translation=tuple(frame.getPosition()), orientation=tuple(frame.getQuaternion()))
        
        
            if ST == ry.ST.sphere:
                cfg_cuboid = sim_utils.MeshSphereCfg(
                radius=tuple(frame.getSize()),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color = tuple(frame.getMeshColors()[0][0:3]/255)),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
                physics_material=sim_utils.RigidBodyMaterialCfg(),
                collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True)
            )

                cfg_cuboid.func("/World/Origin_obj/Sphere", cfg_cuboid, translation=tuple(frame.getPosition()), orientation=tuple(frame.getQuaternion()))

            if ST == ry.ST.capsule:
                cfg_cuboid = sim_utils.MeshCapsuleCfg(
                radius= frame.getSize()[1],
                height= frame.getSize()[0], 
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color = tuple(frame.getMeshColors()[0][0:3]/255)),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
                physics_material=sim_utils.RigidBodyMaterialCfg(),
                collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True)
                )

                cfg_cuboid.func("/World/Origin_obj/Capsule", cfg_cuboid, translation=tuple(frame.getPosition()), orientation=tuple(frame.getQuaternion()))

            if ST == ry.ST.cylinder:
                cfg_cylinder = sim_utils.MeshCylinderCfg(
                    radius= frame.getSize()[1],
                    height= frame.getSize()[0], 
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color = tuple(frame.getMeshColors()[0][0:3]/255)),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False),
                    physics_material=sim_utils.RigidBodyMaterialCfg(),
                    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True)
                )

                cfg_cylinder.func("/World/Origin_obj/Cylinder", cfg_cylinder, translation=tuple(frame.getPosition()))

    return scene_entities



def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation]):
    """Runs the simulation loop."""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    delta = np.asarray([-0.09897264,  0.31354774 , 0.04517093, -2.67476797, -0.35052074 , 2.98068887,-0.49513032, .04, .04]) - np.asarray([0, -.5,  0,  -2,  0 , 2, -.5, .04, .04])
    start = np.asarray([0, -.5,  0,  -2,  0 , 2, -.5, .04, .04])
    # 3 180
    #sim.pause()

    # Simulate physics
    while simulation_app.is_running():
        # reset
        if count == 0:
            # reset counters
            sim_time = 0.0
            count = 0
            # reset the scene entities
            for index, robot in enumerate(entities.values()):
                # root state
                root_state = robot.data.default_root_state.clone()
                robot.write_root_state_to_sim(root_state)
                # set joint positions
                joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
                robot.write_joint_state_to_sim(joint_pos, joint_vel)
                # clear internal buffers
                robot.reset()
            print("[INFO]: Resetting robots state...")
        # apply random actions to the robots
        
        if count % 100 ==0:
            print("h")

        for robot in entities.values():
            if count < 300:  

                # generate random joint positions
                joint_pos_target = robot.data.default_joint_pos
                joint_pos_target += torch.tensor(delta, device="cuda")/300
                robot.set_joint_position_target(joint_pos_target)
                robot.write_data_to_sim() # perform step

                
                joint_eff_target = torch.tensor([0,0,0,0,0,0,0,0,0], device="cuda")
                robot.set_joint_effort_target(joint_eff_target)
                robot.write_data_to_sim()

            # if count > 300:  
            #     joint_eff_target = torch.tensor([0,0,0,0,0,0,0,.1,.1], device="cuda")
            #     robot.set_joint_effort_target(joint_eff_target)
            #     robot.write_data_to_sim()
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        for robot in entities.values():
            robot.update(sim_dt)


def main():
    """Main function."""

    C = ry.Config()
    C.addFile('scenarios/pandaOneBlock.g')

    C.view(True)

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([0, 3.5, 3.5], [0.0, 0.0, 0.5])
    # design scene
    scene_entities = config2config(C)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene_entities)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
