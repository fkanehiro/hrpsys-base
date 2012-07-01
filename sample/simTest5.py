import hrpsys

sim = hrpsys.Simulator()
sim.initViewer()

body = sim.createBody("body")

root = body.rootLink()
s1 = root.addCube(0.2, 1.0, 0.2)
s1.diffuse = [1,0,0,1]

child = root.addChildLink("child")
child.b = [0,0.5,0]
child.jointType = "rotate"
child.a = [1,0,0]
child.m = 1.0
child.c = [0,0.5,0]
child.I = [1,0,0,0,1,0,0,0,1]
s2 = child.addCube(0.5, 1.0, 0.1)
s2.b = [0,0.5,0]
s2.diffuse = [0,1,0,1]

sim.initialize()
sim.simulate(3)











