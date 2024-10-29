import robotic as ry 


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))


C.addFrame('box') \
    .setPosition([-.35,.1,1.]) \
    .setShape(ry.ST.box, size=[.06,.06,.06]) \
    .setColor([1,.5,0]) \
    .setContact(1)
