from gripper import GripperInfo
import os
data_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),'../data'))
drone_data_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),'../drone-data'))

# airobot_grip = GripperInfo("airobot_grip",0,[1,2],[0], type='parallel',
#     klampt_model=os.path.join(drone_data_dir,"airobot_grip.rob"))
# airobot_grip.center = (0,0,-0.06)
# airobot_grip.primary_axis = (0,0,1)
# airobot_grip.secondary_axis = (0,1,0)
# airobot_grip.finger_length = 0.06
# airobot_grip.finger_depth = 0.01
# airobot_grip.finger_width = 0.02
# airobot_grip.maximum_span = 0.085 - 0.01
# airobot_grip.minimum_span = 0
# airobot_grip.open_config = [0,0]
# airobot_grip.closed_config = [0.5,-0.5]

robotiq_85 = GripperInfo("robotiq_85",0,[1,2,3,4,5,6,7,8],[0],
    klampt_model=os.path.join(data_dir,"robots/robotiq_85.rob"))
robotiq_85.center = (0,0,0.1)
robotiq_85.primary_axis = (0,0,1)
robotiq_85.secondary_axis = (1,0,0)
robotiq_85.finger_length = 0.06
robotiq_85.finger_depth = 0.01
robotiq_85.finger_width = 0.02
robotiq_85.maximum_span = 0.085 - 0.01
robotiq_85.minimum_span = 0
robotiq_85.open_config = [0]*8
robotiq_85.closed_config = [0.723,0,0.723,-0.723,-0.723,0.723,-0.723,0 ]

airobot = GripperInfo.mounted(robotiq_85,os.path.join(drone_data_dir,"airobot.urdf"),"gripper:Link_0","airobot")

if __name__ == '__main__':
    from klampt import vis
    import sys
    if len(sys.argv) == 1:
        grippers = [i for i in GripperInfo.all_grippers]
        print("ALL GRIPPERS",grippers)
    else:
        grippers = sys.argv[1:]
    
    for i in grippers:
        g = GripperInfo.get(i)
        print("SHOWING GRIPPER",i)
        g.add_to_vis()
        vis.setWindowTitle(i)
        def setup():
            vis.show()
        def cleanup():
            vis.show(False)
            vis.clear()
        vis.loop(setup=setup,cleanup=cleanup)
