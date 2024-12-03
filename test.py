import robotic as ry
import argparse
from omni.isaac.lab.app import AppLauncher
import numpy as np
import torch 
from translator import SimulationManager

if __name__ == "__main__":
    C = ry.Config()
    C.addFile('scenarios/twoPandasManyObjects.g')
    C.view(True)

    parser = argparse.ArgumentParser()
    AppLauncher.add_app_launcher_args(parser)
    args_cli = parser.parse_args()
    

    sim_manager = SimulationManager(args_cli, C)
    #sim_manager.execute_robot_path( [[-0.09897264,  0.31354774 , 0.04517093, -2.67476797, -0.35052074 , 2.98068887,-0.49513032]], "robotic")
    sim_manager.main()
    sim_manager.simulation_app.close()
