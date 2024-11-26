import robotic as ry 
import numpy as np

C = ry.Config()
C.addFile('scenarios/pandaOneBlock.g')

C.view(True)

bot = ry.BotOp(C, False)

qHome = C.getJointState()

komo = ry.KOMO(C, 1,1,0, True)
komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], qHome)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
komo.addObjective([], ry.FS.positionDiff, ['l_gripper', 'box'], ry.OT.eq, [1e1])
komo.addObjective([], ry.FS.scalarProductXX, ['l_gripper', 'box'], ry.OT.eq, [1e1], [0])
komo.addObjective([], ry.FS.scalarProductXZ, ['l_gripper', 'box'], ry.OT.eq, [1e1], [0])
komo.addObjective([], ry.FS.distance, ['l_palm', 'box'], ry.OT.ineq, [1e1])

ret = ry.NLP_Solver(komo.nlp(), verbose=0 ) .solve()

q = komo.getPath()
C.setJointState(q[0])
C.view(True, "IK solution")


print(q)
bot.moveTo(q[0])
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)


delta = np.asarray([-0.09897264,  0.31354774 , 0.04517093, -2.67476797, -0.35052074 , 2.98068887,-0.49513032, .04, .04]) - np.asarray([0, -.5,  0,  -2,  0 , 2, -.5, .04, .04])
start = np.asarray([0, -.5,  0,  -2,  0 , 2, -.5, .04, .04])

for i in range (11):
    print(start + i*delta/10)
