import time
from klampt import *
from klampt import vis
from klampt.math import vectorops,so3,se3
from klampt.model.trajectory import Trajectory
from klampt.io import numpy_convert
import numpy as np
import math
import random
import sys
sys.path.append('../common')
import gripper
import drone_gripper
from klampt.model.contact import ContactPoint
finger_radius = 0.01

class AntipodalGrasp:
    """A structure containing information about antipodal grasps.
    
    Attributes:
        center (3-vector): the center of the fingers (object coordinates).
        axis (3-vector): the direction of the line through the
            fingers (object coordinates).
        approach (3-vector, optional): the direction that the fingers
            should move forward to acquire the grasp.
        finger_width (float, optional): the width that the gripper should
            open between the fingers.
        contact1 (ContactPoint, optional): a point of contact on the
            object.
        contact2 (ContactPoint, optional): another point of contact on the
            object.
    """
    def __init__(self,center,axis):
        self.center = center
        self.axis = axis
        self.approach = None
        self.finger_width = None
        self.contact1 = None
        self.contact2 = None

    def add_to_vis(self,name,color=(1,0,0,1)):
        if self.finger_width == None:
            w = 0.05
        else:
            w = self.finger_width*0.5+finger_radius
        a = vectorops.madd(self.center,self.axis,w)
        b = vectorops.madd(self.center,self.axis,-w)
        vis.add(name,[a,b],color=color)
        if self.approach is not None:
            vis.add(name+"_approach",[self.center,vectorops.madd(self.center,self.approach,0.05)],color=(1,0.5,0,1))

def match_grasp(gripper_center,gripper_closure_axis,grasp):
    """
    Args:
        gripper_center (3-vector): local coordinates of the center-point between the gripper's fingers.
        gripper_closure_axis (3-vector): local coordinates of the axis connecting the gripper's fingers.
        grasp (AntipodalGrasp): the desired grasp
        
    Returns:
        (R,t): a Klampt se3 element describing the matching gripper transform
    """
    t_center = grasp.center
    t_axis = grasp.axis
    R = so3.vector_rotation(gripper_closure_axis, t_axis)
    t = vectorops.sub(t_center, so3.apply(R, gripper_center))
    return R,t

def match_grasp_sample(gripper_center,gripper_closure_axis,grasp):
    """Sample a transform for the gripper that meets the grasp constraints.
    
    Args:
        gripper_center (3-vector): local coordinates of the center-point between the gripper's fingers.
        gripper_closure_axis (3-vector): local coordinates of the axis connecting the gripper's fingers.
        grasp (AntipodalGrasp): the desired grasp
    
    Returns:
        (R,t): a Klampt se3 element describing a randomly sampled matching gripper transform
    """
    R = so3.rotation(grasp.axis, random.randint(0,360))
    t_center = grasp.center
    t_axis = grasp.axis
    R0 = so3.vector_rotation(gripper_closure_axis, t_axis)
    R0 = so3.mul(R,R0)
    t0 = vectorops.sub(t_center, so3.apply(R0, gripper_center))
    return R0,t0

def dosample():
    T = match_grasp_sample(gripper_center,gripper_closure_axis,grasp1)
    gripper_geom.setCurrentTransform(*T)
    vis.nativeWindow().setTransform("gripper",R=T[0],t=T[1])
    feasible = False
    if not gripper_geom.collides(world2.terrain(0).geometry()):
        if not gripper_geom.collides(obj2.geometry()):
            feasible = True
    if not feasible:
        vis.setColor("gripper",1,0,0)
    else:
        vis.setColor("gripper",0,1,0)
    
def dofeas():
    feasible = False
    for i in range(100):
        T = match_grasp_sample(gripper_center,gripper_closure_axis,grasp1)
        gripper_geom.setCurrentTransform(*T)
        if not gripper_geom.collides(world2.terrain(0).geometry()):
            if not gripper_geom.collides(obj2.geometry()):
                #success!
                feasible = True
                vis.nativeWindow().setTransform("gripper",R=T[0],t=T[1])
                break
    if not feasible:
        print("Unable to sample a feasible pose in 100 samples??")
    if not feasible:
        vis.setColor("gripper",1,0,0)
    else:
        vis.setColor("gripper",0,1,0)

def object_centric_match(gripper_center,gripper_closure_axis,grasp_local,obj):
    """Sample a transform for the gripper that meets the desired grasp
    for a RigidObjectModel. 
    
    Args:
        gripper_center (3-vector): local coordinates of the center-point between the gripper's fingers.
        gripper_closure_axis (3-vector): local coordinates of the axis connecting the gripper's fingers.
        grasp_local (AntipodalGrasp): the desired grasp, with coordinates given in the local frame of
            obj.
        obj (RigidObjectModel): the object to be grasped, posed in the world according to its
            estimated transform.
    
    Returns:
        (R,t): a Klampt se3 element describing a randomly sampled maching gripper transform.
    """
    #TODO: code goes here
    T_obj = obj.getTransform()
    t_center = se3.apply(T_obj, grasp_local.center)
    t_axis = se3.apply(T_obj, grasp_local.axis)
    R = so3.rotation(t_axis, random.randint(0,360))
    R0 = so3.vector_rotation(gripper_closure_axis, t_axis)
    R0 = so3.mul(R,R0)
    t0 = vectorops.sub(t_center, so3.apply(R0, gripper_center))
    return R0,t0

def do_randomize_object():
    if random.random() < 0.5:
        flipz = so3.rotation((1,0,0),math.pi)
    else:
        flipz = so3.identity()
    obj2.setTransform(so3.mul(flipz,so3.rotation((0,0,1),random.uniform(0,math.pi*2))),
                             [random.uniform(-0.4,0.4),random.uniform(-0.4,0.4),0.02])
    vis.update()

def do_feas_grasp():    
    feasible = False
    for i in range(100):
        T = object_centric_match(gripper_center,gripper_closure_axis,grasp1,obj2)
        gripper_geom.setCurrentTransform(*T)
        if not gripper_geom.collides(world2.terrain(0).geometry()):
            if not gripper_geom.collides(obj2.geometry()):
                #success!
                feasible = True
                vis.nativeWindow().setTransform("gripper",R=T[0],t=T[1])
                break
    if not feasible:
        print("Unable to sample a feasible pose in 100 samples??")
    if not feasible:
        vis.setColor("gripper",1,0,0)
    else:
        vis.setColor("gripper",0,1,0)
