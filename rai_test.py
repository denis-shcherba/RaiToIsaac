import robotic as ry 


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

    # class robotic.ry.Frame

    #     A (coordinate) of a configuration, which can have a parent, and associated shape, joint, and/or inertia

    #     addAttributes(self: robotic.ry.Frame, arg0: dict) → None

    #     add/set attributes for the frame

    #     getAttributes(self: robotic.ry.Frame) → dict

    #     get frame attributes

    #     getJointState(self: robotic.ry.Frame) → arr

    #     getMeshPoints(self: robotic.ry.Frame) → arr

    #     getMeshTriangles(self: robotic.ry.Frame) → uintA

    #     getPosition(self: robotic.ry.Frame) → arr

    #     getQuaternion(self: robotic.ry.Frame) → arr

    #     getRelativePosition(self: robotic.ry.Frame) → arr

    #     getRelativeQuaternion(self: robotic.ry.Frame) → arr

    #     getRotationMatrix(self: robotic.ry.Frame) → arr

    #     getSize(self: robotic.ry.Frame) → arr

    #     info(self: robotic.ry.Frame) → dict

    #     setAttribute(self: robotic.ry.Frame, arg0: str, arg1: float) → robotic.ry.Frame

    #     setColor(self: robotic.ry.Frame, arg0: arr) → robotic.ry.Frame

    #     setContact(self: robotic.ry.Frame, arg0: int) → robotic.ry.Frame

    #     setJoint(self: robotic.ry.Frame, arg0: robotic.ry.JT) → robotic.ry.Frame

    #     setJointState(self: robotic.ry.Frame, arg0: arr) → robotic.ry.Frame

    #     setMass(self: robotic.ry.Frame, arg0: float) → robotic.ry.Frame

    #     setMeshAsLines(self: robotic.ry.Frame, arg0: List[float]) → None

    #     setParent(self: robotic.ry.Frame, parent: robotic.ry.Frame, keepAbsolutePose_and_adaptRelativePose: bool = False, checkForLoop: bool = False) → robotic.ry.Frame

    #     setPointCloud(self: robotic.ry.Frame, points: numpy.ndarray, colors: numpy.ndarray[numpy.uint8] = array([], dtype=uint8)) → None

    #     setPose(self: robotic.ry.Frame, arg0: str) → None

    #     setPosition(self: robotic.ry.Frame, arg0: arr) → robotic.ry.Frame

    #     setQuaternion(self: robotic.ry.Frame, arg0: arr) → robotic.ry.Frame

    #     setRelativePose(self: robotic.ry.Frame, arg0: str) → None

    #     setRelativePosition(self: robotic.ry.Frame, arg0: arr) → robotic.ry.Frame

    #     setRelativeQuaternion(self: robotic.ry.Frame, arg0: arr) → robotic.ry.Frame

    #     setShape(self: robotic.ry.Frame, type: robotic.ry.ST, size: arr) → robotic.ry.Frame

    #     unLink(self: robotic.ry.Frame) → robotic.ry.Frame






# MeshCfg
	
# Configuration parameters for a USD Geometry or Geom prim.


# MeshCapsuleCfg
	
# Configuration parameters for a capsule mesh prim.


# MeshConeCfg
	
# Configuration parameters for a cone mesh prim.


# MeshCuboidCfg	

# Configuration parameters for a cuboid mesh prim with deformable properties.


# MeshCylinderCfg	

# Configuration parameters for a cylinder mesh prim with deformable properties.


# MeshSphereCfg
	
# Configuration parameters for a sphere mesh prim with deformable properties.
    




# ------------- CUBOID ------------------

# size
	
# Size of the cuboid (in m).


# visible
	
# Whether the spawned asset should be visible.


# semantic_tags
	
# List of semantic tags to add to the spawned asset.


# copy_from_source
	
# Whether to copy the asset from the source prim or inherit it.


# mass_props
	
# Mass properties.


# deformable_props
	
# Deformable body properties.


# rigid_props
	
# Rigid body properties.


# collision_props
	
# Properties to apply to all collision meshes.


# activate_contact_sensors
	
# Activate contact reporting on all rigid bodies.


# visual_material_path

# Path to the visual material to use for the prim.


# visual_material
	
# Visual material properties.


# physics_material_path

# Path to the physics material to use for the prim.


# physics_material
	
# Physics material properties.