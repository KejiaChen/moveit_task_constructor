import numpy as np
from scipy.spatial.transform import Rotation as R

def ee_array2matrix(ee_array):
	'''convert ee_array with size (16, ) to ee_matrix with size (4, 4)'''
	return (ee_array.reshape((4, 4))).T

def ee_matrix2array(ee_matrix):
	'''convert ee_matrix with size (4, 4) to ee_array with size (16, )'''
	return ee_matrix.T.flatten("F")

def vector_angle(vector):
	'''heading direction of the vector'''
	origin = np.zeros(vector.size)
	origin[1] = 1
	unit_vector = vector/ np.linalg.norm(vector)
	unit_origin = origin/ np.linalg.norm(origin)
	dot_product = np.dot(unit_vector , unit_origin)
	angle = np.arccos(dot_product)
	
	return angle

def two_vector_angle(vec1, vec2):
	unit_vec1 = vec1/np.linalg.norm(vec1)
	unit_vec2 = vec2/np.linalg.norm(vec2)
	dot_product = np.dot(unit_vec1, unit_vec2)
	angle = np.arccos(dot_product)
	normal_vector = np.cross(unit_vec1, unit_vec2)
	sign = np.sign(np.sum(normal_vector))
	return angle, sign

def ee_array2posemsg(ee_matrix_in_W):
	rotation_matrix = R.from_matrix(ee_matrix_in_W[:3, :3])
	rotation_quat = rotation_matrix.as_quat()
	position = ee_matrix_in_W[:3, 3]
	return position, rotation_quat

def project_vec_on_vec(original_vec, ref_vec):
	projected_norm = np.dot(original_vec, ref_vec)/np.linalg.norm(ref_vec)
	projected_vector = projected_norm*ref_vec
	return projected_norm, projected_vector
	
def project_vec_on_plane(original_vec, normal_vec):
	_, projected_vector_normal = project_vec_on_vec(original_vec, normal_vec)
	projected_vector = original_vec - projected_vector_normal
	return projected_vector

def form_transform_matrix(rotation_matrix, translation_vec):
    transform_matrix = np.concatenate((rotation_matrix, np.reshape(translation_vec, (3, 1))), axis=1)
    transform_matrix = np.concatenate((transform_matrix, np.array([[0, 0, 0, 1]])), axis=0)
    return transform_matrix

def inverse_transformation(T_a2b, dim=2):
    R_a2b = T_a2b[:dim, :dim]
    t_a2b = np.reshape(T_a2b[:dim, dim], (dim,1))
    
    R_b2a = np.transpose(R_a2b)
    t_b2a = -R_b2a.dot(t_a2b)
    
    constant_zero= np.zeros((1,dim))
    constant = np.concatenate((constant_zero, np.array([[1]])), axis=1)
    
    T_b2a = np.concatenate((R_b2a, t_b2a), axis=1)
    T_b2a = np.concatenate((T_b2a, constant), axis=0)
    return T_b2a