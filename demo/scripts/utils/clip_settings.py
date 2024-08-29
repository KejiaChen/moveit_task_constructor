import numpy as np

# XY offset between center of aruco and clip position
Z_M_CLIP_OFFSET = 0.03
X_S_CLIP_OFFEST = [0.045, -0.025, 0]

# Size
Z_L_CLIP_SIZE = [0.06, 0.03, 0.12]
Z_M_CLIP_SIZE = [0.06, 0.03, 0.10]
Z_S_CLIP_SIZE = ()

X_S_CLIP_SIZE = [0.04, 0.04, 0.06]

BOARD_SIZE = [0.61, 0.61, 0.035]
BOARD_BASE_SIZE = [0.8, 0.8, 0.048]

# Safety: obstacle expansion
EXPAN = [0.01, 0.01, 0.01]

'''
grasp: the order of the left and right grasping point. If false, left and right are exchanged.
dir: the direction of clip fixing. values {1, -1}, where 1 means pushing, -1 means pulling.
offset: height offset for grasping.
fix_offset: heigh offset for clip fixing.
'''

TRAJ_SEQ_2 = {7: {"grasp":True, "dir":1, "offset":0, "fix_offset":0},
            9: {"grasp":True, "dir":1, "offset":-0.01, "fix_offset":0},
            6: {"grasp":True, "dir":1, "offset":-0.005, "fix_offset":-0.005},
            5: {"grasp":True, "dir":1, "offset":0, "fix_offset":0}}
            # 0: {"grasp":True, "dir":1}}

TRAJ_SEQ_1 = {7: {"grasp":True, "dir":1, "offset":0, "fix_offset":0},
            9: {"grasp":True, "dir":-1, "offset":-0.01, "fix_offset":-0.01},
            5: {"grasp":True, "dir":1, "offset":-0.01, "fix_offset":-0.01},
            6: {"grasp":True, "dir":-1, "offset":-0.005, "fix_offset":-0.005}}

CLIP_FIXING_TRAJ_1 = {
                        8: {"grasp":True, "dir":1, "pos_offset":[0, 0.01, 0.04], "height_offset":0.04, "y_axis":[0,0,1], "z_axis":[0,1,0], "contact_force":3, "r2_griplen":[0,0,0.0], "sample_list":[0, -0.0168]},
                        5: {"grasp":True, "dir":1, "pos_offset":[0, 0, 0],  "height_offset":-0.005, "y_axis":[0,1,0], "z_axis":[0,0,1], "contact_force":6, "r2_griplen":[0,0,0.015], "sample_list":[-0.005, 0.012, 0.0098]},
                        7: {"grasp":True, "dir":1, "pos_offset":[0, 0, 0],  "height_offset":-0.005, "y_axis":[0,1,0], "z_axis":[0,0,1], "contact_force":6, "r2_griplen":[0,0,0.01], "sample_list":[0, 0.016,0.0089]},
                        9: {"grasp":True, "dir":1, "pos_offset":[0, 0, 0], "height_offset":0, "y_axis":[0,1,0], "z_axis":[0,0,1], "contact_force":6, "r2_griplen":[0,0,0.003], "sample_list":[0]}
                     }

DEMO_FIXING_TRAJ_1 = {
                  # 6: {"grasp":True, "dir":1, "pos_offset":[0,0,0], "height_offset":0.04, 
                  #       "y_axis":[0,0,1], "z_axis":[0,1,0], "r2_griplen":[0,0,0.005], "sample_list":[0, 0], 
                  #       "clip_aruco_offset":[0.045, 0, 0], "bottom_aruco_offset":[-0.065, 0, 0], "grasp_offset_r" :0.065, "grasp_offset_l": 0.04,
                  #       "contact_force_magnitude": 4.0, "push_force_magnitude": 20.0, "stretch_force_magnitude": 12.0},
                  8: {"grasp":True, "dir":1, "pos_offset":[0, 0, 0],  "height_offset":-0.005, 
                        "y_axis":[0,1,0], "z_axis":[0,0,1], "r2_griplen":[0,0,0.005], "sample_list":[-0.01, 0.012, 0.0060], 
                        "clip_aruco_offset":[0.06, 0, 0], "bottom_aruco_offset":[-0.045, 0, 0], "grasp_offset_r" :0.05, "grasp_offset_l": 0.05,
                        "contact_force_magnitude": 10.0, "push_force_magnitude": 25.0, "stretch_force_magnitude": 12.0},
                  5: {"grasp":True, "dir":1, "pos_offset":[0, 0, 0],  "height_offset":-0.005, 
                        "y_axis":[0,1,0], "z_axis":[0,0,1], "r2_griplen":[0,0,0.005], "sample_list":[0.004], 
                        "clip_aruco_offset":[0.06, 0, 0], "bottom_aruco_offset":[-0.08, 0, 0], "grasp_offset_r" :0.08, "grasp_offset_l": 0.05,
                        "contact_force_magnitude": 10.0, "push_force_magnitude": 25.0, "stretch_force_magnitude": 12.0},
                  0: {"grasp":True, "dir":1, "pos_offset":[0, 0, 0],  "height_offset":-0.005, 
                        "y_axis":[0,1,0], "z_axis":[0,0,1], "r2_griplen":[0,0,0.005], "sample_list":[0.004], 
                        "clip_aruco_offset":[0.035, 0, 0], "bottom_aruco_offset":[-0.06, 0, 0], "grasp_offset_r" :0.05, "grasp_offset_l": 0.05,
                        "contact_force_magnitude": 10.0, "push_force_magnitude": 25.0, "stretch_force_magnitude": 12.0}
                  }

DEMO_FIXING_TRAJ_2 = {
                  6: {"grasp":True, "dir":1, "pos_offset":[0,0,0], "height_offset":0.04, 
                        "y_axis":[0,0,1], "z_axis":[0,1,0], "r2_griplen":[0,0,0.005], "sample_list":[0, 0], 
                        "clip_aruco_offset":[0.045, 0, 0], "bottom_aruco_offset":[-0.065, 0, 0], "grasp_offset_r" :0.065, "grasp_offset_l": 0.04,
                        "contact_force_magnitude": 4.0, "push_force_magnitude": 20.0, "stretch_force_magnitude": 12.0},
                  8: {"grasp":True, "dir":1, "pos_offset":[0, 0, 0],  "height_offset":-0.005, 
                        "y_axis":[0,1,0], "z_axis":[0,0,1], "r2_griplen":[0,0,0.005], "sample_list":[-0.01, 0.012, 0.0060], 
                        "clip_aruco_offset":[0.06, 0, 0], "bottom_aruco_offset":[-0.045, 0, 0], "grasp_offset_r" :0.05, "grasp_offset_l": 0.05,
                        "contact_force_magnitude": 10.0, "push_force_magnitude": 25.0, "stretch_force_magnitude": 12.0},
                  5: {"grasp":True, "dir":1, "pos_offset":[0, 0, 0],  "height_offset":-0.005, 
                        "y_axis":[0,1,0], "z_axis":[0,0,1], "r2_griplen":[0,0,0.005], "sample_list":[0.004], 
                        "clip_aruco_offset":[0.06, 0, 0], "bottom_aruco_offset":[-0.08, 0, 0], "grasp_offset_r" :0.06, "grasp_offset_l": 0.05,
                        "contact_force_magnitude": 10.0, "push_force_magnitude": 25.0, "stretch_force_magnitude": 12.0},
                  }

DEMO_FIXING_TRAJ_3 = {
                  0: {"grasp":True, "dir":1, "pos_offset":[0,0,0], "height_offset":0.04, 
                        "y_axis":[0,0,1], "z_axis":[0,1,0], "r2_griplen":[0,0,0.005], "sample_list":[0, 0], 
                        "clip_aruco_offset":[0.06, 0, 0], "bottom_aruco_offset":[-0.065, 0, 0], "grasp_offset_r" :0.05, "grasp_offset_l": 0.04,
                        "contact_force_magnitude": 4.0, "push_force_magnitude": 20.0, "stretch_force_magnitude": 12.0},
                  8: {"grasp":True, "dir":1, "pos_offset":[0, 0, 0],  "height_offset":-0.005, 
                        "y_axis":[0,1,0], "z_axis":[0,0,1], "r2_griplen":[0,0,0.005], "sample_list":[-0.01, 0.012, 0.0060], 
                        "clip_aruco_offset":[0.06, 0, 0], "bottom_aruco_offset":[-0.045, 0, 0], "grasp_offset_r" :0.045, "grasp_offset_l": 0.06,
                        "contact_force_magnitude": 10.0, "push_force_magnitude": 25.0, "stretch_force_magnitude": 12.0},
                  # 5: {"grasp":True, "dir":1, "pos_offset":[0, 0, 0],  "height_offset":-0.005, 
                  #       "y_axis":[0,1,0], "z_axis":[0,0,1], "r2_griplen":[0,0,0.005], "sample_list":[0.004], 
                  #       "clip_aruco_offset":[0.06, 0, 0], "bottom_aruco_offset":[-0.08, 0, 0], "grasp_offset_r" :0.06, "grasp_offset_l": 0.05,
                  #       "contact_force_magnitude": 10.0, "push_force_magnitude": 25.0, "stretch_force_magnitude": 12.0},
                  }

CONFIG = {1: DEMO_FIXING_TRAJ_1, 2: DEMO_FIXING_TRAJ_2, 3: DEMO_FIXING_TRAJ_3}

TRAJ_SEQ_TEST = {7: {"grasp":True, "dir":1, "offset":0, "fix_offset":0},
            6: {"grasp":True, "dir":1, "offset":-0.01, "fix_offset":-0.01}}


TRAJ_SEQ = TRAJ_SEQ_TEST
CLIP_PATH = list(TRAJ_SEQ.keys())
