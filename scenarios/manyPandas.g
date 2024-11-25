world: {}

### table

origin (world): { Q: [0, 0, .6], shape: marker, size: [.03] }
table (origin): { Q: [0, 0, -.05], shape: ssBox, size: [2.3, 5.24, .1, .02], color: [.3, .3, .3], contact, logical:{ } }

## two pandas
Prefix: "l_"
Include: </home/denis/miniconda3/envs/isaaclab/lib/python3.10/site-packages/robotic/rai-robotModels/panda/panda.g>
Prefix: "m_"
Include: </home/denis/miniconda3/envs/isaaclab/lib/python3.10/site-packages/robotic/rai-robotModels/panda/panda.g>
Prefix: "n_"
Include: </home/denis/miniconda3/envs/isaaclab/lib/python3.10/site-packages/robotic/rai-robotModels/panda/panda.g>
Prefix: "o_"
Include: </home/denis/miniconda3/envs/isaaclab/lib/python3.10/site-packages/robotic/rai-robotModels/panda/panda.g>
Prefix: "p_"
Include: </home/denis/miniconda3/envs/isaaclab/lib/python3.10/site-packages/robotic/rai-robotModels/panda/panda.g>
Prefix: "q_"
Include: </home/denis/miniconda3/envs/isaaclab/lib/python3.10/site-packages/robotic/rai-robotModels/panda/panda.g>
Prefix: "r_"
Include: </home/denis/miniconda3/envs/isaaclab/lib/python3.10/site-packages/robotic/rai-robotModels/panda/panda.g>
Prefix: "s_"
Include: </home/denis/miniconda3/envs/isaaclab/lib/python3.10/site-packages/robotic/rai-robotModels/panda/panda.g>
Prefix: False

## position them on the table
Edit l_panda_base (origin): { Q: "t(-.4 -.8 .0) d(90 0 0 1)", motors, joint: rigid }
Edit m_panda_base (origin): { Q: "t( .4 -.8 .0) d(90 0 0 1)", motors, joint: rigid }
Edit n_panda_base (origin): { Q: "t(-.4 -.1 .0) d(90 0 0 1)", motors, joint: rigid }
Edit o_panda_base (origin): { Q: "t(.4 -.1 .0) d(90 0 0 1)", motors, joint: rigid }
Edit p_panda_base (origin): { Q: "t(-.4 .6 .0) d(90 0 0 1)", motors, joint: rigid }
Edit q_panda_base (origin): { Q: "t(.4 .6 .0) d(90 0 0 1)", motors, joint: rigid }
Edit r_panda_base (origin): { Q: "t(-.4 1.3 .0) d(90 0 0 1)", motors, joint: rigid }
Edit s_panda_base (origin): { Q: "t(.4 1.3 .0) d(90 0 0 1)", motors, joint: rigid }

## make gripper dofs inactive (unselected)
Edit l_panda_finger_joint1: { joint_active: False }
Edit m_panda_finger_joint1: { joint_active: False }
Edit n_panda_finger_joint1: { joint_active: False }
Edit o_panda_finger_joint1: { joint_active: False }
Edit p_panda_finger_joint1: { joint_active: False }
Edit q_panda_finger_joint1: { joint_active: False }
Edit r_panda_finger_joint1: { joint_active: False }
Edit s_panda_finger_joint1: { joint_active: False }

### camera

camera(world): {
 Q: "t(-0.01 -.2 2.) d(-150 1 0 0)",
 shape: camera, size: [.1],
 focalLength: 0.895, width: 640, height: 360, zRange: [.5, 100]
}

