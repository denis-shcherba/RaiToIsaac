world: {}

### table

origin (world): { Q: [0, 0, .6], shape: marker, size: [.03] }
table (origin): { Q: [0, 0, -.05], shape: ssBox, size: [2.3, 1.24, .1, .02], color: [.3, .3, .3], contact, logical:{ } }

## many pandas
Prefix: "l_"
Include: </home/denis/miniconda3/envs/isaaclab/lib/python3.10/site-packages/robotic/rai-robotModels/panda/panda.g>
Prefix: "r_"
Include: </home/denis/miniconda3/envs/isaaclab/lib/python3.10/site-packages/robotic/rai-robotModels/panda/panda.g>
Prefix: False

## position them on the table
Edit l_panda_base (origin): { Q: "t(-.4 -.3 .0) d(90 0 0 1)", motors, joint: rigid }
Edit r_panda_base (origin): { Q: "t( .4 -.3 .0) d(90 0 0 1)", motors, joint: rigid }

## make gripper dofs inactive (unselected)
Edit l_panda_finger_joint1: { joint_active: False }
Edit r_panda_finger_joint1: { joint_active: False }

### camera

camera(world): {
 Q: "t(-0.01 -.2 2.) d(-150 1 0 0)",
 shape: camera, size: [.1],
 focalLength: 0.895, width: 640, height: 360, zRange: [.5, 100]
}


box: { X: [-.35 .1 1], shape: box, size: [.06 .06 .06], color: [1,.5,0] , contact: 1}
cylinder: { X: [-.15,.1,1.], shape: cylinder, size: [.06,.06], color: [.5,.1,.3], contact: 1}
capsule: { X: [.05,.1,1.], shape: capsule, size: [.15, .05], color: [0,1.,.5], contact: 1}
sphere: { X: [.25,.1,1.], shape: sphere, size: [.05], color: [.4, .3, .7], contact: 1}