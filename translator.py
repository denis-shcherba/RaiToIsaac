import robotic as ry
import argparse
from omni.isaac.lab.app import AppLauncher
import numpy as np
import torch 
from utils import get_bspline_representation

class SimulationManager:
    def __init__(self, args, C):
        # Initialize the Isaac Sim app
        self.app_launcher = AppLauncher(args)
        self.simulation_app = self.app_launcher.app
        self.sim_utils = None
        self.prim_utils = None
        self.Articulation = None
        self.FRANKA_PANDA_CFG = None
        self.args_cli = args

        self.C = C


    def setup_imports(self):
        # Import Isaac modules post-launch
        global sim_utils, prim_utils, Articulation, FRANKA_PANDA_CFG
        import omni.isaac.core.utils.prims as prim_utils
        import omni.isaac.lab.sim as sim_utils
        from omni.isaac.lab.assets import Articulation
        from omni.isaac.lab_assets import FRANKA_PANDA_CFG
        self.sim_utils = sim_utils
        self.prim_utils = prim_utils
        self.Articulation = Articulation
        self.FRANKA_PANDA_CFG = FRANKA_PANDA_CFG

    def config_to_config(self) -> tuple[dict, list[float]]:
        """Translate `ry.Config` to simulation configuration."""
        self.setup_imports()  # Ensure imports are loaded here
        sim_utils = self.sim_utils
        prim_utils = self.prim_utils
        Articulation = self.Articulation
        FRANKA_PANDA_CFG = self.FRANKA_PANDA_CFG

        # Ground plane configuration
        cfg = sim_utils.GroundPlaneCfg()
        cfg.func("/World/defaultGroundPlane", cfg)

        # Dome light configuration
        cfg = sim_utils.DomeLightCfg(
            intensity=700.0,
            color=(1.0, 0.98, 0.95)  # Warm, slightly yellow-white
        )
        cfg.func("/World/Light", cfg)

        origin = [.0, .0, .0]
        prim_utils.create_prim("/World/Origin", "Xform", translation=origin)

        # Temporary config for adding a scenario
        tmp_config = ry.Config()
        tmp_config.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))

        frame_names = self.C.getFrameNames()
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

        for name in frame_names:
            frame = self.C.getFrame(name)

            if "table" in name:
                # Table configuration
                cfg_table = sim_utils.MeshCuboidCfg(
                    size=tuple(frame.getSize()[:3]),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(frame.info()["color"])),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                    physics_material=sim_utils.RigidBodyMaterialCfg(),
                    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True)
                )
                cfg_table.func(f"/World/Origin/{name}", cfg_table, translation=tuple(frame.getPosition()))

            elif "panda_base" in name:
                # Panda robot configuration
                franka_name = name
                franka_arm_cfg = FRANKA_PANDA_CFG.replace(prim_path=f"/World/Origin/{franka_name}")
                franka_arm_cfg.init_state.pos = tuple(frame.getPosition())
                franka_arm_cfg.init_state.rot = (1/np.sqrt(2), 0.0, 0.0, 1/np.sqrt(2))
                franka_arm_cfg.init_state.joint_pos = panda_init_joint_state
                franka_panda = Articulation(cfg=franka_arm_cfg)

                scene_entities[franka_name] = franka_panda

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

    def run_simulator(self, sim, entities, path=None):
        """Run the simulation loop."""
        sim_dt = sim.get_physics_dt()
        sim_time = 0.0
        count = 0

        if path:
            start = np.asarray([0, -.5,  0,  -2,  0 , 2, -.5, .04, .04])
            delta = np.asarray(path[0]) - np.asarray([0, -.5,  0,  -2,  0 , 2, -.5, .04, .04])

        for robot in entities.values():
            root_state = robot.data.default_root_state.clone()
            robot.write_root_state_to_sim(root_state)
            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()

        sim.pause()
        while self.simulation_app.is_running():

            """ botop exec """
            if path:
                for robot in entities.values():
                    if count < 300:  

                        # generate random joint positions
                        joint_pos_target = robot.data.default_joint_pos
                        joint_pos_target += torch.tensor(delta, device="cuda")/300
                        robot.set_joint_position_target(joint_pos_target)
                        robot.write_data_to_sim()

            sim.step()
            sim_time += sim_dt
            count+=1
            for robot in entities.values():
                robot.update(sim_dt)

    def load_robot_path(self, path_file: str) -> list[list[float]]:
        """
        Load a robot path from a file or other source. 
        Assumes the path is in the form of 7D joint vectors.
        
        Args:
            path_file (str): Path to the file containing the robot path.
            
        Returns:
            list[list[float]]: A list of 7D joint vectors representing the path.
        """
        pass


    def execute_robot_path(self,  path: list[list[float]], simulator="robotic"):
        """
        Execute the given robot path in the simulation.

        Args:
            robot_entity: The robot entity to execute the path on.
            path (list[list[float]]): The 7D joint path to execute.
            sim_dt (float): Simulation timestep for updating states.
        """
        # Convert path to B-spline 
        get_bspline_representation(path)
        
        # move with frequence through spline and pd control (p position d velocity) robot to follow spline motion
        # incorparate Timing cost?
        pass

    def visualize_robot_path(self, robot_entity, path: list[list[float]]):
        """
        Visualize the robot path in the simulation environment (e.g., using markers or overlays). Adds visual cues in the simulation to show the path, such as lines, markers, or ghosted robot positions.
        
        Args:
            robot_entity: The robot entity to visualize the path for.
            path (list[list[float]]): The 7D joint path to visualize.
        """
        pass

    def record_robot_execution(self, robot_entity) -> list[list[float]]:
        """
        Record the robot's execution of a path, capturing its joint states over time. Captures the robot's joint states as it executes the path in simulation, which can be used for analysis or debugging.
        
        Args:
            robot_entity: The robot entity whose execution to record.
            
        Returns:
            list[list[float]]: A list of 7D joint vectors representing the recorded execution.
        """
        pass

    def main(self):
        """Main function for running the simulation."""

        scene_entities = self.config_to_config()

        sim_cfg = self.sim_utils.SimulationCfg(device=self.args_cli.device)
        sim = self.sim_utils.SimulationContext(sim_cfg)

        sim.set_camera_view([0, 3.5, 3.5], [0.0, 0.0, 0.5])
        sim.reset()
        print("[INFO]: Setup complete...")
        self.run_simulator(sim, scene_entities)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    AppLauncher.add_app_launcher_args(parser)
    args_cli = parser.parse_args()
    
    C = ry.Config()
    C.addFile('scenarios/twoPandasManyObjects.g')
    C.view(True)

    sim_manager = SimulationManager(args_cli, C)
    #sim_manager.execute_robot_path( [[-0.09897264,  0.31354774 , 0.04517093, -2.67476797, -0.35052074 , 2.98068887,-0.49513032]], "robotic")
    sim_manager.main()
    sim_manager.simulation_app.close()
