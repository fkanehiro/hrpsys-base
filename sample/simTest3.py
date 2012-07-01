import hrpsys
import commands

openhrp_dir = commands.getoutput("pkg-config --variable=prefix openhrp3.1")
project = "file://"+openhrp_dir+"/share/OpenHRP-3.1/sample/project/Sample.xml"

sim = hrpsys.Simulator()
sim.initViewer()
sim.loadProject(project)
sim.simulate()
