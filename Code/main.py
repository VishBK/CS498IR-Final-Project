#Imports

# use pip3 (klampt, PyOpenGL, numpy)
# http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/

import time
from klampt import *
from klampt import vis
from klampt import WorldModel
from klampt.math import vectorops,so3,se3
from klampt.model.trajectory import Trajectory
from klampt.io import numpy_convert
import numpy as np
import math
import random
import sys
import scipy
sys.path.append('../common')
import gripper
import drone_gripper
vis.init('IPython')


fn = "../drone-data/world.xml"
world = WorldModel()
res = world.readFile(fn)

#print(world.numRobots())

if not res:
    print("Unable to read file",fn)
    exit(0)

obj = world.robot(0)
#print(obj)

#m = obj.getMass()
#m.estimate(obj.geometry(),mass=0.454,surfaceFraction=0.2)
#obj.setMass(m)

#make the object transparent yellow
#obj.appearance().setColor(0.8,0.8,0.2,0.5)
#draw center of mass
vis.createWindow()
closeup_viewport = {'up': {'z': 0, 'y': 1, 'x': 0}, 'target': {'z': 0, 'y': 0, 'x': 0}, 'near': 0.1, 'position': {'z': 1.0, 'y': 0.5, 'x': 0.0}, 'far': 1000}
vis.setViewport(closeup_viewport)
vis.add("world",world)
#vis.add("COM",m.getCom(),color=(1,0,0,1),size=0.01)
vis.show()
