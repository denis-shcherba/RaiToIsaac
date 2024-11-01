import robotic as ry

import argparse
from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser(description="This script demonstrates different single-arm manipulators.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import numpy as np
import torch 

import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR


from omni.isaac.lab_assets import (
    FRANKA_PANDA_CFG,
)



def config2config(C: ry.Config):

    tmp_config = ry.Config()
    tmp_config.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

    initFrameNames = C.getFrameNames()     


    new_frames  = list(set(initFrameNames) ^ set(tmp_config.getFrameNames()))


    for name in new_frames:
        obj_frame = C.getFrame(name)
        ST = obj_frame.getShapeType()

        if ST == ry.ST.box:
            cfg_cuboid = sim_utils.MeshCuboidCfg(
                size=tuple(obj_frame.getSize()),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color = tuple(obj_frame.info()["color"])),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                physics_material=sim_utils.RigidBodyMaterialCfg(),
                collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True)
            )

            cfg_cuboid.func("/World/Origin_obj/Cuboid", cfg_cuboid, translation=tuple(obj_frame.getPosition()))
        
        
        if ST == ry.ST.sphere:
            cfg_cuboid = sim_utils.MeshSphereCfg(
            radius=tuple(obj_frame.getSize()),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color = tuple(obj_frame.info()["color"])),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            physics_material=sim_utils.RigidBodyMaterialCfg(),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True)
        )

            cfg_cuboid.func("/World/Origin_obj/Sphere", cfg_cuboid, translation=tuple(obj_frame.getPosition()))

        if ST == ry.ST.capsule:
            cfg_cuboid = sim_utils.MeshCapsuleCfg(
            radius= obj_frame.getSize()[1],
            height= obj_frame.getSize()[0], 
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color = tuple(obj_frame.info()["color"])),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            physics_material=sim_utils.RigidBodyMaterialCfg(),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True)
            )

            cfg_cuboid.func("/World/Origin_obj/Capsule", cfg_cuboid, translation=tuple(obj_frame.getPosition()))

        if ST == ry.ST.cylinder:
            cfg_cylinder = sim_utils.MeshCylinderCfg(
                radius= obj_frame.getSize()[1],
                height= obj_frame.getSize()[0], 
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color = tuple(obj_frame.info()["color"])),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                physics_material=sim_utils.RigidBodyMaterialCfg(),
                collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True)
            )

            cfg_cylinder.func("/World/Origin_obj/Cylinder", cfg_cylinder, translation=tuple(obj_frame.getPosition()))




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


    # Robot
    franka_arm_cfg = FRANKA_PANDA_CFG.replace(prim_path="/World/Origin1/Robot")
    franka_arm_cfg.init_state.pos = (0.0, -.2, .65)
    franka_arm_cfg.init_state.rot = (0.7071067811865476, 0.0, 0.0, 0.7071067811865475)
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
            print("[INFO]: Resetting robots state...")

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
    C = ry.Config()
    C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

    C.addFrame('box') \
        .setPosition([-.35,.1,1.]) \
        .setShape(ry.ST.box, size=[.06,.06,.06]) \
        .setColor([1,.5,0]) \
        .setContact(1)

    C.addFrame('cylinder') \
        .setPosition([-.15,.1,1.]) \
        .setShape(ry.ST.cylinder, size=[.06,.06]) \
        .setColor([.5,.1,.3]) \
        .setContact(1)

    C.addFrame('capsule') \
        .setPosition([.05,.1,1.]) \
        .setShape(ry.ST.capsule, [.15, .05]) \
        .setColor([.0,1.,.5]) 

    C.addFrame('sphere') \
        .setPosition([.25,.1,1.]) \
        .setShape(ry.ST.sphere, [.05]) \
        .setColor([.4, .3, .7]) \
        .setContact(1)
    

    C.view(True)
    
    config2config(C)

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([0, 3.5, 3.5], [0.0, 0.0, 0.5])
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
    main()

    simulation_app.close()
