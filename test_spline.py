import matplotlib.pyplot as plt
import numpy as np 
from utils import get_bspline_representation, construct_discretized_bspline
import robotic as ry
import time

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))
C.addFrame('way1'). setShape(ry.ST.marker, [.1]) .setPosition([.4, .2, 1.])
C.addFrame('way2'). setShape(ry.ST.marker, [.1]) .setPosition([.4, .2, 1.4])
C.addFrame('way3'). setShape(ry.ST.marker, [.1]) .setPosition([-.4, .2, 1.])
C.addFrame('way4'). setShape(ry.ST.marker, [.1]) .setPosition([-.4, .2, 1.4])
C.view()


# only differences: the kOrder=2, control objective order 2, constrain final jointState velocity to zero
komo = ry.KOMO(C, 4, 10, 2, False)
komo.addControlObjective([], 0, 1e-1) # what happens if you change weighting to 1e0? why?
komo.addControlObjective([], 2, 1e0)
komo.addObjective([1], ry.FS.positionDiff, ['l_gripper', 'way1'], ry.OT.eq, [1e1])
komo.addObjective([2], ry.FS.positionDiff, ['l_gripper', 'way2'], ry.OT.eq, [1e1])
komo.addObjective([3], ry.FS.positionDiff, ['l_gripper', 'way3'], ry.OT.eq, [1e1])
komo.addObjective([4], ry.FS.positionDiff, ['l_gripper', 'way4'], ry.OT.eq, [1e1])
komo.addObjective([4], ry.FS.jointState, [], ry.OT.eq, [1e1], [], order=1)

ret = ry.NLP_Solver(komo.nlp(), verbose=0 ) .solve()
print(ret)
q = komo.getPath()
print('size of path:', q.shape)

# for t in range(q.shape[0]):
#     C.setJointState(q[t])
#     C.view(False, f'waypoint {t}')
#     time.sleep(.1)

import matplotlib.pyplot as plt
plt.plot(q)
plt.show()

# Generate interpolated spline path
num_knots = q.shape[0]
num_points = 100
knots, t, degree = get_bspline_representation(q)
spline_path=construct_discretized_bspline(knots, t, degree, num_points)

# Plot the path for each dimension
dimensions = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
plt.figure(figsize=(12, 10))
for i in range(7):
    plt.subplot(4, 2, i + 1)
    plt.plot(np.linspace(0, 1, num_points), spline_path[:, i], label=f"Dimension {dimensions[i]}")
    plt.scatter(np.linspace(0, 1, num_knots), q[:, i], color="red", label="Knots")
    plt.xlabel("Parameter")
    plt.ylabel(f"{dimensions[i]} Value")
    plt.legend()
    plt.grid()

plt.tight_layout()
plt.show()