#Imports

# use pip3 (klampt, PyOpenGL, numpy)
# http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/

import time
from klampt import *
from klampt import vis
from klampt import WorldModel
from klampt.math import vectorops,so3,se3
from klampt.model import trajectory
from klampt.io import numpy_convert
import numpy as np
import math
import random
import sys
import scipy
import gripper
import drone_gripper
import planning
import grasp
from stable_faces import stable_faces,debug_stable_faces
# vis.init('IPython')
sys.path.append('../common')
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
        if i == 0:
            obj.geometry().scale(0.05)
        
        obj_x = np.random.random()
        obj_y = np.random.random()
        obj_z = 0
        obj.setTransform(obj.geometry().getCurrentTransform()[0], [obj_x, obj_y, obj_z])

        m = obj.getMass()
        m.estimate(obj.geometry(),mass=0.454,surfaceFraction=0.2)
        obj.setMass(m)

        obj.appearance().setColor(np.random.random(),np.random.random(),np.random.random(),1)
        obj_ids.append(i)
    bmin = 0
    bmax = 0.5
    # arrange_objects(world, obj_ids, bmin, bmax)


# load the world and robot models
fn = "../drone-data/world.xml"
world = WorldModel()
res = world.readFile(fn)

if not res:
    print("Unable to read file",fn)
    exit(0)

# generate a random world
drone = world.robot(0)
print('# links:', drone.numLinks())
objs = {"object1": "../data/objects/cube.off"}
gen_objs(world, objs)

#define some quantities of the gripper
gripper_center = vectorops.madd(drone_gripper.robotiq_85.center,drone_gripper.robotiq_85.primary_axis,drone_gripper.robotiq_85.finger_length-0.005)
gripper_closure_axis = drone_gripper.robotiq_85.secondary_axis

state = 'to_object'

# set drone home
home_coord = [2,2,1.5]
#TODO figure out config numbers: [drone_x, drone_y, drone_z, yaw, pitch, roll, top_left, top_right, bottom_right, bottom_left, ?, ?, ?, ?, ?]
start_config = [home_coord[0], home_coord[1], home_coord[2], 2.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
drone.setConfig(start_config)

obj_1 = world.rigidObject(0)
# obj_1.setTransform(obj_1.getTransform()[0], [0.2, 0, 0])
obj_tform = obj_1.getTransform()
obj_com = obj_1.getMass().getCom()

obj_x, obj_y, obj_z = obj_tform[1]
obj_cent_x, obj_cent_y, obj_cent_z = se3.apply(obj_tform, obj_com)
obj_cent_z = obj_1.geometry().getBB()[1][2] + gripper_center[2]
print('BB:',obj_1.geometry().getBB())
cur_x, cur_y, cur_z = home_coord

# motion planning
target_config = [obj_cent_x, obj_cent_y, obj_cent_z-0.01, so3.rpy(obj_tform[0])[2], 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
print(target_config)
path = planning.feasible_plan(world, drone, target_config)
print(path)
if path is None:
    print('No path found')
    exit()

drone.setConfig(start_config)
ptraj = trajectory.RobotTrajectory(drone,milestones=path)
ptraj.times = [5*t / len(ptraj.times) * 1.0 for t in ptraj.times]
traj = trajectory.path_to_trajectory(ptraj,timing='robot',smoothing=None)
paths = [traj.milestones,traj.milestones[::-1]]

# find grasps
drone.setConfig(target_config)

grip = drone_gripper.robotiq_85
gripper_tform = drone.link(0).geometry().getCurrentTransform()
gripper_center = vectorops.madd(grip.center,grip.primary_axis,grip.finger_length-0.005)
gripper_centerW = se3.apply(gripper_tform, gripper_center)
gripper_axis = so3.apply(gripper_tform[0], grip.secondary_axis)
finger_pt = vectorops.sub(vectorops.mul(gripper_axis, grip.maximum_span), gripper_centerW)
dist = vectorops.norm(obj_1.geometry().rayCast(finger_pt,gripper_axis)[1])

opening_width = grip.maximum_span - dist*2 - grip.finger_depth*2
drone.setConfig(start_config)

vis.add("world", world)

# drone flies to object
tol = 0.01
path_index = 0
path_progress = 0
nxt_config = start_config
prev_config = []

vis.show()              #open the window
t0 = time.time()
while vis.shown():
    cur_path = paths[path_index]
    if state == 'to_object':
        nxt_config = cur_path[path_progress]
        path_progress += 1

        if path_progress >= len(cur_path):
            path_progress = 0
            state = 'grasp'
    
    elif state == 'grasp':
        nxt_config = target_config[:7]+grip.partway_open_config(grip.width_to_opening(opening_width))        
        path_index += 1
        state = 'to_home'
        time.sleep(1)
    
    elif state == 'to_home':
        prev_config = nxt_config
        nxt_config = cur_path[path_progress]
        nxt_config = nxt_config[:7]+grip.partway_open_config(grip.width_to_opening(opening_width))

        cur_obj_t = obj_1.geometry().getCurrentTransform()
        obj_trans = vectorops.add(cur_obj_t[1],vectorops.sub(nxt_config[:3], prev_config[:3]))
        obj_rpy = so3.rpy(cur_obj_t[0])
        obj_rot = so3.from_rpy(vectorops.add(obj_rpy, [vectorops.sub(nxt_config, prev_config)[5], vectorops.sub(nxt_config, prev_config)[4], vectorops.sub(nxt_config, prev_config)[3]]))
        obj_1.setTransform(cur_obj_t[0],obj_trans)

        path_progress += 1
        if path_progress >= len(cur_path):
            path_progress = 0
            path_index += 1
            state = 'done'
    
    drone.setConfig(nxt_config)
    if path_index >= len(paths):
        break

    time.sleep(0.02)    #loop is called ~100x times per second
vis.loop()
vis.kill()              #safe cleanup
