from translator import SimulationManager
import robotic as ry
import argparse
from omni.isaac.lab.app import AppLauncher
import manipulation as manip

C = ry.Config()
C.addFile('scenarios/pandaOneBlock.g')
C.view(True)


gripper = 'l_gripper'
palm = 'l_palm'
box = 'box'
table = 'table'



qHome = C.getJointState()

man = manip.ManipulationModelling()
man.setup_inverse_kinematics(C)
man.grasp_top_box(1., gripper, box, "yz")

# solve it
pose = man.solve()

print(man.ret.feasible)

C.setJointState(pose[0])
C.view(True)
C.setJointState(qHome)

bot = ry.BotOp(C, False)
bot.sync(C)

print(pose)
bot.moveTo(pose[0])
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)

bot.gripperMove(ry._left, 0)
while not bot.gripperDone(ry._left):
    bot.sync(C, .1)

bot.moveTo(qHome)
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)
# parser = argparse.ArgumentParser()
# AppLauncher.add_app_launcher_args(parser)
# args_cli = parser.parse_args()


# sim_manager = SimulationManager(args_cli, C)
# sim_manager.main()
# sim_manager.simulation_app.close()