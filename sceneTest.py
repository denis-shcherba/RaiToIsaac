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
    .setColor([.0,1.,.5]) \
    .setQuaternion([1, 0, 1, 0])

C.addFrame('sphere') \
    .setPosition([.25,.1,1.]) \
    .setShape(ry.ST.sphere, [.05]) \
    .setContact(1) \
    .setMass(1) \
    .setColor([.4, .3, .7]) \

C.view(True)

print(C.getFrame("box").info())

new_frames  = C.getFrameNames()


for name in new_frames:
    # if ("panda_base" in name or "table" in name) and C.getFrame(name).getParent().info()["name"]=="origin":
    #     print(name, (C.getFrame(name).getParent().info()["name"], C.getFrame(name).getParent().info()["X"]) if C.getFrame(name).getParent() else "zo")
    #     print(7*"\n")    
    print(name, (C.getFrame(name).getParent().info()["name"], C.getFrame(name).getParent().info()["X"]) if C.getFrame(name).getParent() else "zo")
    print(tuple(C.getFrame(name).getPosition()))
