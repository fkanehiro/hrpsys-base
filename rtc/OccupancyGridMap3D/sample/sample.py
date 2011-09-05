import rtm
from rtm import *
from OpenHRP import *

ogm = findRTC("OccupancyGridMap3D0")
ogm.setProperty("initialMap", "sample.bt")
ogm.start()
ogm_svc = OGMap3DServiceHelper.narrow(ogm.service("service1"))
aabb = AABB()
aabb.pos = Point3D()
aabb.pos.x = -3.5
aabb.pos.y = -5
aabb.pos.z = 0.5
aabb.size = Size3D()
aabb.size.l = 2.0
aabb.size.w = 4.0
aabb.size.h = 0.1
map = ogm_svc.getOGMap3D(aabb)
print "resolution = ",map.resolution
print "number of voxels = ",map.nx,"x",map.ny,"x",map.nz
for x in range(map.nx):
    for y in range(map.ny): 
        for z in range(map.nz): 
            print map.cells[x*map.ny*map.nz+y*map.nz+z],
    print


