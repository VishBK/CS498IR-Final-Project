from klampt.plan import robotplanning
from klampt.plan.cspace import MotionPlan
from klampt.model import trajectory
from klampt import vis
from klampt import RobotModel
import math
import time

def feasible_plan(world,robot,qtarget):
    """Plans for some number of iterations from the robot's current configuration to
    configuration qtarget.  Returns the first path found.

    Returns None if no path was found, otherwise returns the plan.
    """
    t0 = time.time()

    moving_joints = [0,1,2,3]
    # ignored_collisions = [world.terrain(0)]
    # for i in range(world.numRigidObjects()):
        # for j in range(world.numTerrains()):
            # ignored_collisions.append((world.rigidObject(i), world.terrain(j)))
    # space = robotplanning.makeSpace(world=world,robot=robot,edgeCheckResolution=1e-2,movingSubset=moving_joints)
    # plan = MotionPlan(space,type='prm')
    plan = robotplanning.planToConfig(world=world,robot=robot,target=qtarget,movingSubset=moving_joints,\
        type='sbl')
    if not plan:
        return None

    numIters = 100
    while time.time() - t0 < 10:
        plan.planMore(numIters)
        path = plan.getPath()
        if len(path) > 0:
            break
    t1 = time.time()
        
    print("Planning time,",numIters,"iterations",t1-t0)
    
    #to be nice to the C++ module, do this to free up memory
    plan.space.close()
    plan.close()
    if len(path) == 0:
        return None
    return path


def optimizing_plan(world,robot,qtarget):
    """Plans for some number of iterations from the robot's current configuration to
    configuration qtarget.

    Returns None if no path was found, otherwise returns the best plan found.
    """
    #TODO: copy what's in feasible_plan, but change the way in which you to terminate
    t0 = time.time()

    moving_joints = [0,1,2,3]
    plan = robotplanning.planToConfig(world=world,robot=robot,target=qtarget,movingSubset=moving_joints,\
        type='sbl',shortcut=True,perturbationRadius=0.5)
    # plan = robotplanning.planToConfig(world=world,robot=robot,target=qtarget,movingSubset=moving_joints,\
    #     type='sbl',perturbationRadius=0.5,shortcut=True,restart=True,restartTermCond="{foundSolution:1,maxIters:100}")
    # plan = robotplanning.planToConfig(world=world,robot=robot,target=qtarget,movingSubset=moving_joints,type='rrt*')
    # plan = robotplanning.planToConfig(world=world,robot=robot,target=qtarget,movingSubset=moving_joints,type='lazyrrg*')
    if not plan:
        return None

    numIters = 100
    while time.time() - t0 < 5:
        plan.planMore(numIters)
        path = plan.getPath()
    
    t1 = time.time()
    
    print("Planning time,",numIters,"iterations",t1-t0)
    
    #to be nice to the C++ module, do this to free up memory
    plan.space.close()
    plan.close()
    if len(path) == 0:
        print("Failed")
        return None
    else:
        print("Path Length:", len(path))
        return path

def debug_plan_results(plan,robot):
    """Potentially useful for debugging planning results..."""
    assert isinstance(plan,MotionPlan)
    #this code just gives some debugging information. it may get expensive
    V,E = plan.getRoadmap()
    print(len(V),"feasible milestones sampled,",len(E),"edges connected")

    print("Plan stats:")
    pstats = plan.getStats()
    for k in sorted(pstats.keys()):
        print("  ",k,":",pstats[k])

    print("CSpace stats:")
    sstats = plan.space.getStats()
    for k in sorted(sstats.keys()):
        print("  ",k,":",sstats[k])
    """
    print("  Joint limit failures:")
    for i in range(robot.numLinks()):
        print("     ",robot.link(i).getName(),":",plan.space.ambientspace.joint_limit_failures[i])
    """

    path = plan.getPath()
    if path is None or len(path)==0:
        print("Failed to plan path between configuration")
        #debug some sampled configurations
        numconfigs = min(10,len(V))
        vis.debug("some milestones",V[2:numconfigs],world=world)
        pts = []
        for i,q in enumerate(V):
            robot.setConfig(q)
            pt = robot.link(9).getTransform()[1]
            pts.append(pt)
        for i,q in enumerate(V):
            vis.add("pt"+str(i),pts[i],hide_label=True,color=(1,1,0,0.75))
        for (a,b) in E:
            vis.add("edge_{}_{}".format(a,b),trajectory.Trajectory(milestones=[pts[a],pts[b]]),color=(1,0.5,0,0.5),width=1,pointSize=0,hide_label=True)
        return None

    print("Planned path with length",trajectory.RobotTrajectory(robot,milestones=path).length())
