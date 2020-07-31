import pybullet as p
import time
import numpy as np
import random
np.random.seed(5)
random.seed(5)
import sys
import os
from mayavi import mlab as mayalab 

simulation_dir = '../simulation'
sys.path.insert(0, simulation_dir)

utils_dir = './utils/'
sys.path.insert(0, utils_dir)

from obj_file import OBJ, plot_pc_with_normal
from Microwave_Manipulation_Env import RobotEnv
from coord_helper import local_coord_to_global_bl, local_coord_to_global_bl_unit_vec, local_coord_to_global, filter_outside_bb, two_points_normal_feasible, normalize_vector
from debug_helper import debug_sphere
import robot_no_Wrench as robot

p_id = p.connect(p.GUI)
env = RobotEnv(0, p)

OBJ_SCALING = 0.3
def interpolate(x, y, coef):
    assert coef <= 1 and coef >= 0
    return x * (1 - coef) + y * coef
def check_pair_feasible(p1, n1, p2, n2):
    dist_3d = np.linalg.norm(p1 - p2)
    # return dist_3d < 0.05 and two_points_normal_feasible(p1, n1, p2, n2)
    if two_points_normal_feasible(p1, n1, p2, n2):
        # print(dist_3d)
        return True
    return two_points_normal_feasible(p1, n1, p2, n2)

def propose_grasp_pairs(obj, pc_bl, pc_n_bl, pc, pc_n):
    n_points = pc_bl.shape[0]
    arr_candidates_bl = []
    arr_candidates = []
    for i in range(0, n_points):
        for j in range(i + 1, n_points):
            p1, p2 = pc_bl[i], pc_bl[j]
            p1_n, p2_n = pc_n_bl[i], pc_n_bl[j]
            if check_pair_feasible(p1, p1_n, p2, p2_n):
                arr_candidates_bl.append([p1, p1_n, p2, p2_n])
                arr_candidates.append([pc[i], pc_n[i], pc[j], pc_n[j]])
    return arr_candidates_bl, arr_candidates

def convert_to_bl_coord(pc, pc_n):
    pc_after = np.zeros_like(pc)
    pc_n_after = np.zeros_like(pc_n)
    for i in range(0, pc.shape[0]):
        pc_after[i] = local_coord_to_global_bl(pc[i], env.obj_id, obj_scaling=OBJ_SCALING)
        pc_n_after[i] = local_coord_to_global_bl_unit_vec(pc_n[i], env.obj_id)
    return pc_after, pc_n_after

def apply_force_along_direction(object_id, link_id, f_c, env, control_link_id, total_t=100, debug_lines=False):
    f_c = normalize_vector(f_c)
    force_magnitude = 50000
    last_proxy_goal_pos = np.array([0, 0, 0])
    
    # control with proxy point
    for i in range(total_t):
        f_c_world = local_coord_to_global_bl_unit_vec(f_c, object_id, link_id=link_id)
        proxy_goal_pos = env.robot.positionDeltaControlWithId(control_link_id, f_c_world * force_magnitude)
        p.stepSimulation()
        time.sleep(1./240.)
        if debug_lines and i%20 == 0:
            p.addUserDebugLine(last_proxy_goal_pos, proxy_goal_pos, lineWidth=19)
        last_proxy_goal_pos = proxy_goal_pos
        print(proxy_goal_pos)

def apply_external_force(object_id, link_id, f_c, force_local_pos, obj_init_rpy, total_t=100):
    f_c = normalize_vector(f_c)
    force_magnitude = 50000
    
    for i in range(total_t):
        f_c_world = local_coord_to_global_bl_unit_vec(f_c, object_id, link_id=link_id)
        # force_world_pos = local_coord_to_global_bl(force_local_pos, object_id, link_id)
        obj_world_pos = p.getLinkState(object_id, link_id)[0]
        obj_world_rpy = np.array(p.getEulerFromQuaternion(p.getLinkState(object_id, link_id)[1])) - np.array(obj_init_rpy) 
        force_world_pos = local_coord_to_global(force_local_pos, obj_world_pos, obj_world_rpy)
        p.applyExternalForce(object_id, link_id, np.array(f_c_world) * force_magnitude, force_world_pos, flags=p.WORLD_FRAME)
        if i % 10 == 0:
            p.addUserDebugLine(force_world_pos, force_world_pos + f_c_world, lineWidth=19)
        # p.applyExternalForce(object_id, link_id, np.array(f_c) * force_magnitude, force_pos, flags=p.LINK_FRAME)
        print(f_c)
        p.stepSimulation()
        time.sleep(1/10.)


def load_obj_and_pc():
    # obj 0: microwave body
    # obj_0 = OBJ(file_name="/home/yifany/ScaffoldLearning/resource/urdf/obj_libs/microwaves/yifan/body.obj",rotation_degree=0.0,normalize=False,normal_normalized=False)
    # pc_0, pc_n_0, _ = obj_0.sample_points(100, with_normal=True)
    # bb_lower = np.array([-0.66, -0.55, -0.65])
    # bb_upper = np.array([0.61, 0.45, 0.57])

    # filter_mask = filter_outside_bb(pc_0, bb_lower, bb_upper)
    # pc_0, pc_n_0 = pc_0[filter_mask], pc_n_0[filter_mask]

    # obj 1: microwave door
    obj_1 = OBJ(file_name="/home/yifany/ScaffoldLearning/resource/urdf/obj_libs/microwaves/yifan/new_door_handle_space.obj",rotation_degree=0.0,normalize=False,normal_normalized=False)
    pc_1, pc_n_1, _ = obj_1.sample_points(100, with_normal=True)
    pc_1, pc_n_1 = pc_1[pc_1[:, 2] > 0.6], pc_n_1[pc_1[:, 2] > 0.6]

    return [
        # (obj_0, pc_0, pc_n_0),
        (obj_1, pc_1, pc_n_1),
    ]

def grab_and_pull_control(p1, p2, p1_local, p2_local, f_c):
    if p1[1] > p2[1]:
        p3 = np.copy(p2)
        p2 = np.copy(p1)
        p1 = p3
    # print(p1, p2)
    for _ in range(10):
        env.robot.gripperControl(0)
        p.stepSimulation()
    p.addUserDebugLine(p1, p2, lineWidth=19)
    p2_pre = np.copy(p2)
    p2_pre[0] -= 0.1
    p2_pre[1] += 0.03
    p2_start = np.array(p.getLinkState(env.robot.robotId, env.robot.gripper_left_tip_index)[0])

    # p.addUserDebugLine(p2, p2_start, lineWidth=19)

    # for i in range(300):
    #     env.robot.positionControlWithId(env.robot.gripper_left_tip_index, interpolate(p2_start, p2_pre, i/300.), [0, 1.57, 0])
    #     p.stepSimulation()
    #     time.sleep(1./240.)

    # for i in range(100):
    #     env.robot.positionControlWithId(env.robot.gripper_left_tip_index, p2 * i/100. + (1-i/100.)*p2_pre, [0, 1.57, 0], False)
    #     p.stepSimulation()
    #     time.sleep(1./240.)

    # for i in range(20):
    #     env.robot.gripperControl(interpolate(0, 135, 1. * i/20))
    #     # env.robot.positionControlWithId(env.robot.gripper_right_tip_index, p1, control_gripper=True)
    #     p.stepSimulation()
    #     time.sleep(1./240.)
    # apply_force_along_direction(env.obj_id, 1, [0, 0, 1], env, env.robot.gripper_left_tip_index, 1000, True)
    
    door_world_pos = p.getLinkState(env.obj_id, 1)[0]
    door_world_rpy = p.getEulerFromQuaternion(p.getLinkState(env.obj_id, 1)[1])
    p.addUserDebugLine(door_world_pos, [0, 0, 0], lineWidth=19)
    print(p2, local_coord_to_global_bl(p2_local, env.obj_id, obj_scaling=OBJ_SCALING))
    print(p.getJointInfo(env.obj_id, 1))
    # print(local_coord_to_global_bl(p2 - door_world_pos, env.obj_id, 1))
    # print(local_coord_to_global(p2 - door_center, door_center, [0, 0, 0]))
    apply_external_force(env.obj_id, 1, [0, 0, 1], p2 - door_world_pos, door_world_rpy, total_t=500)

    # for i in range(500):
        # env.robot.positionControlWithId(env.robot.gripper_left_tip_index, p1, [0, 1.57, 0], False)
        # p.stepSimulation()
        # time.sleep(1./240.)
    # p2_pre[0] -= 0.1 * 200
    # p2_pre[1] += 0.05 * 200
    # p.addUserDebugLine(p2_pre, [0, 0, 0], lineWidth=19)

    # for i in range(20):
        # env.robot.positionControlWithId(env.robot.gripper_left_tip_index, p2_pre, control_gripper=False)
        # p.stepSimulation()
        # time.sleep(1./240.)
    # env.robot.positionControlWithId(env.robot.gripper_right_tip_index, p1, [0, 1.57, 0], False)
    # env.robot.gripperTipPosControl(p2, p1)
    # for _ in range(100):
        # p.stepSimulation()
        # time.sleep(1./240.)
    

obj_and_pc = load_obj_and_pc()
all_grasp_candidates_bl = []
all_grasp_candidates = []
for obj, pc, pc_n in obj_and_pc:
    # grasp_candidates = propose_grasp_pairs(obj, pc, pc_n)
    pc_bl, pc_n_bl = convert_to_bl_coord(pc, pc_n)
    grasp_candidates_bl, grasp_candidates = propose_grasp_pairs(obj, pc_bl, pc_n_bl, pc, pc_n)
    all_grasp_candidates_bl.append(grasp_candidates_bl)
    all_grasp_candidates.append(grasp_candidates)
    print(len(grasp_candidates_bl), len(pc))
    # print(len(grasp_candidates_bl), len(grasp_candidates), len(pc))

    # if len(grasp_candidates) > 0:
        # for i in range(len(grasp_candidates)):
            # plot_pc_with_normal(np.array([grasp_candidates[i][0]]), np.array([grasp_candidates[i][1]]))
            # plot_pc_with_normal(np.array([grasp_candidates[i][2]]), np.array([grasp_candidates[i][3]]))
        # obj.plot()
        # mayalab.show()

f_c = [0, 0, 0]
p.setRealTimeSimulation(0)

grab_and_pull_control(all_grasp_candidates_bl[0][1][0], all_grasp_candidates_bl[0][1][2],all_grasp_candidates[0][1][0], all_grasp_candidates[0][1][2], f_c)

for j in range(10000):
    p.stepSimulation()
    # print(env.robot.getJointValue())
    time.sleep(1./240.)
p.disconnect()
