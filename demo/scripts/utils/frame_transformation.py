import numpy as np
from utils.math_utils import *

# load calibrated scaling factor and transformation matrix
# global_variables = np.load('python/transformation.npz')
# T_R1toR2_3d = global_variables['T_R1toR2_3d']
# T_R2toR1_3d = global_variables['T_R2toR1_3d']
# R_KptoRobot_3d = global_variables['R_KptoRobot_3d']
# T_KctoR2_2d = global_variables['T_KctoR2_2d']
# T_KctoR1_2d= global_variables['T_KctoR1_2d']  # TODO: CHECK
# R_RtoW = global_variables['R_RtoW']
# T_R1toW_3d = global_variables['T_R1toW_3d']
# T_R2toW_3d = global_variables['T_R2toW_3d']


# scales = np.load('python/scale2d_QBboard.npz')
# S_PIXtoREAL = scales['S_PIXtoREAL'] # 0.00123416 0.00123141 1.     
# # S_PIXtoREAL = np.array([0.00121, 0.00121, 1.        ])   
# S_REALtoPIX = scales['S_REALtoPIX'] # 810.27035084 812.07660067   1.    
# # S_REALtoPIX = np.array([825.88888889, 824.03703704,   1.        ])    
# print("loaded scale S_PIXtoREAL", S_PIXtoREAL)
# print("loaded scale S_REALtoPIX", S_REALtoPIX)
# R_KptoKc_2d = np.array([[0,1],[1,0]])

# # Height
# DESK_HEIGHT = 1.0

# moveit
t_TCPtoEE =  [0, 0, -0.1034, 0, 0, -0.7853]
EE_in_TCP = np.array([[0], [0], [-0.1034], [1]])

R_RtoW = np.array([[1, 0, 0],
                   [0, 1, 0],
                   [0, 0, 1]])

R_KptoRobot_3d = np.array([[0, 1, 0],
                           [1, 0, 0],
                           [0, 0, 1]])

T_R1toW_3d= np.concatenate((R_RtoW, np.reshape(np.array([0, 0.281, 1.025]), (3, 1))), axis=1)
T_R1toW_3d = np.concatenate((T_R1toW_3d, np.array([[0, 0, 0, 1]])), axis=0)

T_R2toW_3d= np.concatenate((R_RtoW, np.reshape(np.array([0.0, -0.281, 1.025]), (3, 1))), axis=1)
T_R2toW_3d = np.concatenate((T_R2toW_3d, np.array([[0, 0, 0, 1]])), axis=0)

T_R2toW_2d = np.concatenate((T_R2toW_3d[:2, :2], np.reshape(T_R2toW_3d[:2, 3], (2, 1))), axis=1)
T_R2toW_2d = np.concatenate((T_R2toW_2d, np.array([[0, 0, 1]])), axis=0)

R_KptoW = np.array([[0, 0, 1],
                    [1, 0, 0],
                    [0, 1, 0]])
# T_KctoW_2d = T_R2toW_2d.dot(T_KctoR2_2d)

# # Robot in image
# T_R1toKc_2d = inverse_transformation(T_KctoR1_2d)
# R1_in_Kc = T_R1toKc_2d.dot(np.array([[0], [0], [1]]))
# R1_in_Kp = np.multiply(S_REALtoPIX[:2], np.array([R1_in_Kc[0, 0], R1_in_Kc[1, 0]]))
# R1_in_Kp = R1_in_Kp.astype(int)

# T_R2toKc_2d = inverse_transformation(T_KctoR2_2d)
# R2_in_Kc = T_R2toKc_2d.dot(np.array([[0], [0], [1]]))
# R2_in_Kp =np.multiply(S_REALtoPIX[:2], np.array([R2_in_Kc[0, 0], R2_in_Kc[1, 0]]))
# R2_in_Kp = R2_in_Kp.astype(int)

# Transformation from flange to the finger tip
# Obtained from Franka official instructions
# ATTENTION: this is in robot frame
T_FtoTCP_3d = np.array([[0.707, 0.707, 0, 0],
                        [-0.707, 0.707, 0, 0],
                        [0, 0, 1, 0.1034],
                        [0, 0, 0, 1]])

T_TCPtoF_3d = inverse_transformation(T_FtoTCP_3d, dim=3)