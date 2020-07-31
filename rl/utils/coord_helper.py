import numpy as np
import pybullet as p
import math
EPS = 1e-8

def normalize_vector(v):
    v = v / (EPS + np.linalg.norm(v))
    return v
def rpy_rotmat(rpy):
    rotmat = np.zeros((3,3))
    roll   = rpy[0]
    pitch  = rpy[1]
    yaw    = rpy[2]
    rotmat[0,0] = np.cos(yaw) * np.cos(pitch)
    rotmat[0,1] = np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll)
    rotmat[0,2] = np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)
    rotmat[1,0] = np.sin(yaw) * np.cos(pitch)
    rotmat[1,1] = np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll)
    rotmat[1,2] = np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)
    rotmat[2,0] = - np.sin(pitch)
    rotmat[2,1] = np.cos(pitch) * np.sin(roll)
    rotmat[2,2] = np.cos(pitch) * np.cos(roll)
    return rotmat

def local_coord_to_global(local_pos, local_origin_world_pos, local_origin_world_rpy):
    rotmat = rpy_rotmat(local_origin_world_rpy)
    T = np.zeros((4, 4))
    T[:3, :3] = rotmat
    T[:, 3] = np.append(local_origin_world_pos, [1], axis=0)

    local_pos_after = np.matmul(T, np.append(local_pos, [1]))

    assert local_pos_after[3] == 1
    return local_pos_after[:3] 

def local_coord_to_global_bl_unit_vec(local_pos, object_id, link_id=0):
    local_origin_world_pos = [0, 0, 0]
    local_origin_world_rpy = p.getEulerFromQuaternion(p.getLinkState(object_id, link_id)[1])

    normal = local_coord_to_global(local_pos, local_origin_world_pos, local_origin_world_rpy)
    normal = normalize_vector(normal)
    return normal

def local_coord_to_global_bl(local_pos, object_id, link_id=0, obj_scaling=1):
    local_origin_world_pos = p.getLinkState(object_id, link_id)[0]
    local_origin_world_rpy = p.getEulerFromQuaternion(p.getLinkState(object_id, link_id)[1])

    return local_coord_to_global(local_pos * obj_scaling, local_origin_world_pos, local_origin_world_rpy)

def filter_within_bb(points, lower_corner, upper_corner):
    mask = np.sum(points >= lower_corner, axis=1) * np.sum(points <= upper_corner, axis=1) == lower_corner.shape[0]**2
    return mask
def filter_outside_bb(points, lower_corner, upper_corner):
    mask = np.sum(points >= lower_corner, axis=1) * np.sum(points <= upper_corner, axis=1) < lower_corner.shape[0]**2
    return mask

friction_coef = 0.3

def two_contacts_fc_Nguyen(p1, n1, p2, n2):

    vector_between_contacts = np.array(p1-p2)
    vector_between_contacts = vector_between_contacts/(EPS + np.linalg.norm(vector_between_contacts))
    cosine1 = np.abs(np.sum(vector_between_contacts * n1))
    cosine2 = np.abs(np.sum(vector_between_contacts * n2))
    thre = math.cos(math.atan(friction_coef))
    cosine = min(cosine1,cosine2)
    # print(cosine)
    if cosine > thre:
        return True
    else:
        return False

def two_points_normal_feasible(p1, n1, p2, n2):
    # check if n1 and n2 point opposite (angle > 120 deg)
    cosine = np.sum(n1 * n2)
    if_opposite = cosine < -0.5
    if_colinear = two_contacts_fc_Nguyen(p1, n1, p2, n2)

    return if_opposite and if_colinear

if __name__ == '__main__':
    points = np.array([[1, 1, 1], [2, 4, 2]])
    lower_corner = np.array([0, 0, 0])
    upper_corner = np.array([3, 3, 3])
    print(filter_within_bb(points, lower_corner, upper_corner))