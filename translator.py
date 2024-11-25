import robotic as ry

import argparse
from omni.isaac.lab.app import AppLauncher

# Initialize the simulation context

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


from omni.isaac.lab_assets import (
    FRANKA_PANDA_CFG,
)


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

    origin = [.0, .0, .0]

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
    sim.pause()

    # Simulate physics
    while simulation_app.is_running():
        # reset

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
    C.addFile('scenarios/twoPandasManyObjects.g')
    
    C.view(True)
    
    # translate scene
    scene_entities = config2config(C)

    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)

    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([0, 3.5, 3.5], [0.0, 0.0, 0.5])
    
    sim.reset()
    print("[INFO]: Setup complete...")
    run_simulator(sim, scene_entities)


if __name__ == "__main__":
    main()

    simulation_app.close()
