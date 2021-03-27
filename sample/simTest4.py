import hrpsys
import commands

openhrp_dir = commands.getoutput("pkg-config --variable=prefix openhrp3.2")
model_dir="file://"+openhrp_dir+"/share/OpenHRP-3.2/sample/model/"

sim = hrpsys.Simulator()
sim.initViewer()
box  = sim.loadBody("box",  model_dir+"box.wrl")
box.rootLink().R = [0,0,0]
box2 = sim.loadBody("box2", model_dir+"box2.wrl")
sim.addCollisionCheckPair(box, box2)
sim.initialize()
sim.simulate(1.0)
