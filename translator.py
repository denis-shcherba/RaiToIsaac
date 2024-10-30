import robotic as ry

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

initFrameNames = C.getFrameNames()


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

new_frames  = list(set(initFrameNames) ^ set(C.getFrameNames()))


for name in new_frames:
    ST = C.getFrame(name).getShapeType()
    if ST == ry.ST.box:
        print(2)
    
    if ST == ry.ST.sphere:
        print(3)

    if ST == ry.ST.capsule:
        print(5)

    if ST == ry.ST.cylinder:
        print(6)