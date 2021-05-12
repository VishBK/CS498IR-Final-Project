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
from stable_faces import stable_faces,debug_stable_faces
# vis.init('IPython')
ycb_stable_faces = dict()

def sample_object_pose_table(obj,stable_fs,bmin,bmax):
    """Samples a transform of the object so that it lies on in the given
    bounding box bmin,bmax.

    Args:
        obj (RigidObjectModel)
        stable_fs (list of lists): giving the stable faces of the object,
            as in MP2.
        bmin,bmax (3-vectors): the bounding box of the area in which the
            objects should lie.
    """
    table_height = bmin[2] + 0.001
    face = random.choice(stable_fs)
    normal = np.cross(face[1] - face[0],face[2]-face[0])
    normal = normal / np.linalg.norm(normal)
    centroid = np.sum(face,axis=0)/len(face)
    x = random.uniform(bmin[0],bmax[0])
    y = random.uniform(bmin[1],bmax[1])
    z = table_height + np.dot(centroid,normal)
    Rnormal = so3.canonical((-normal).tolist())
    Rx = so3.rotation((1,0,0),random.uniform(0,math.pi*2))
    Rxz = so3.rotation((0,1,0),-math.pi*0.5)
    R = so3.mul(Rxz,so3.mul(Rx,so3.inv(Rnormal)))
    #R*com + t = [x,y,_]
    t = vectorops.sub([x,y,z],so3.apply(R,obj.getMass().getCom()))
    t[2] = z
    obj.setTransform(R,t)

def arrange_objects(world,objs,bmin,bmax,interior=False):
    global ycb_stable_faces
    for o,obj in enumerate(objs):
        if obj not in ycb_stable_faces:
            world.rigidObject(o).setTransform(*se3.identity())
            ycb_stable_faces[obj] = stable_faces(world.rigidObject(o),stability_tol=0.01,merge_tol=0.05)
            if len(ycb_stable_faces[obj]) == 0:
                print("Object",obj,"has no stable faces with robustness 0.01, trying 0.0")
                ycb_stable_faces[obj] = stable_faces(world.rigidObject(o),stability_tol=0.0,merge_tol=0.05)
                #debug_stable_faces(world.rigidObject(o),ycb_stable_faces[obj])
    i = 0
    while i < world.numRigidObjects():
        faces = ycb_stable_faces[objs[i]]
        samples = 0
        feasible = False
        while not feasible and samples < 100:
            sample_object_pose_table(world.rigidObject(i),faces,bmin,bmax)
            
            samples += 1
            feasible = True
            for j in range(i):
                if world.rigidObject(i).geometry().collides(world.rigidObject(j).geometry()):
                    feasible = False
                    break
            if feasible:
                for j in range(world.numTerrains()):
                    if world.rigidObject(i).geometry().collides(world.terrain(j).geometry()):
                        feasible = False
                        break
        if not feasible:
           world.remove(world.rigidObject(i))
           print("Couldn't find feasible placement for",i,"th object")
        else:
            i += 1

def gen_objs(world, objs):
    obj_ids = []
    for i,obj_key in enumerate(objs.keys()):
        obj = world.makeRigidObject(obj_key)
        obj.geometry().loadFile(objs[obj_key])
        obj.geometry().scale(0.2)
        #print(obj)

        m = obj.getMass()
        m.estimate(obj.geometry(),mass=0.454,surfaceFraction=0.2)
        obj.setMass(m)

        obj.appearance().setColor(np.random.random(),np.random.random(),np.random.random(),1)
        obj_ids.append(i)
    bmin = 0
    bmax = 0.5
    # arrange_objects(world, obj_ids, bmin, bmax)


fn = "../drone-data/world.xml"
world = WorldModel()
res = world.readFile(fn)

#print(world.numRobots())

if not res:
    print("Unable to read file",fn)
    exit(0)

drone = world.robot(0)
objs = {"object1": "../data/objects/cube.off"}
gen_objs(world, objs)


# visualization
vis.createWindow()
closeup_viewport = {'up': {'z': 0, 'y': 1, 'x': 0}, 'target': {'z': 0, 'y': 0, 'x': 0}, 'near': 0.1, 'position': {'z': 1.0, 'y': 0.5, 'x': 0.0}, 'far': 1000}
# vis.setViewport(closeup_viewport)
vis.add("world",world)
#vis.add("COM",m.getCom(),color=(1,0,0,1),size=0.01)
vis.show()
<<<<<<< HEAD
vis.loop()
=======
>>>>>>> f9f675443dd41573312012e0dd3bd7b69260ca89
