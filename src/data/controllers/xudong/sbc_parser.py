import matplotlib.pyplot as plt
import numpy as np
import os


'''
parse the joint target trajectory in simbicon
'''

STATE_BEGIN_KEY = "ConState"
STATE_END_KEY = "/ConState"

TRAJ_BEGIN_KEY = "trajectory"
TRAJ_END_KEY = "/trajectory"

COMP_BEGIN_KEY = "component"
COMP_END_KEY = "/component"


def find_lst(full_cont, key):
    lst = []
    for id, line in enumerate(full_cont):
        empty_line = line.strip()
        if empty_line[0: len(key)] == key:
            lst.append(id)
    return lst


def find_begin_and_end_lst(full_cont, begin_key, end_key):
    '''
        given the file content "full_cont", find the begin/end index by the given keys in them
        return two index list
    '''
    begin_lst, end_lst = [], []
    cur_begin, cur_end = -1, -1

    for id, line in enumerate(full_cont):
        empty_line = line.strip()
        if empty_line[0:len(begin_key)] == begin_key:
            print(f"[debug] traj begin {begin_key}, idx = {id}")
            assert cur_begin == -1
            assert cur_end == -1
            cur_begin = id
            begin_lst.append(cur_begin)
        elif empty_line[0:len(end_key)] == end_key:
            print(f"[debug] traj end {end_key}, idx = {id}")
            assert cur_begin != -1
            assert cur_end == -1
            cur_end = id
            end_lst.append(cur_end)

            cur_begin = -1
            cur_end = -1
    assert len(begin_lst) == len(end_lst)
    return begin_lst, end_lst

def parse_component(cont):
    '''
        return rotation axis, and base trajectory
    '''
    ROTATION_AXIS_KEY = "rotationAxis"
    BASETRAJ_BEGIN_KEY = "baseTrajectory"
    BASETRAJ_END_KEY = "/baseTrajectory"

    basetraj_begin_lst, basetraj_end_lst = find_begin_and_end_lst(
        cont, BASETRAJ_BEGIN_KEY, BASETRAJ_END_KEY)

    rotation_axis_lst = find_lst(cont, ROTATION_AXIS_KEY)

    assert len(rotation_axis_lst) == 1
    assert len(basetraj_begin_lst) == 1

    # 1. get the rotation axis
    rotation_axis_id = rotation_axis_lst[0]
    basetraj_begin_id = basetraj_begin_lst[0]
    basetraj_end_id = basetraj_end_lst[0]

    axis = np.array([float(i)
                     for i in cont[rotation_axis_id].strip().split()[1:]])

    # print(f"axis = {axis}")
    phase_lst = []
    angle_lst = []
    for i in range(basetraj_begin_id + 1, basetraj_end_id):
        num = [float(i) for i in cont[i].strip().split()]
        assert len(num) == 2
        phase_lst.append(num[0])
        angle_lst.append(num[1])

    # phase_lst, angle_lst = handle_phase_angle_catmull_rom(phase_lst, angle_lst)
    return axis, phase_lst, angle_lst


def parse_trajectory(cont):
    '''
        Given the trajectory content, parse the trajectory, return the angle-time knots
        return trajectory
    '''
    print(f"parse traj: {cont[-1].strip()[:len(TRAJ_END_KEY)]}")
    assert cont[0].strip()[:len(TRAJ_BEGIN_KEY)] == TRAJ_BEGIN_KEY
    assert cont[-1].strip()[:len(TRAJ_END_KEY)] == TRAJ_END_KEY

    # 1. begin to parse joint
    joint_name = cont[0].split()[1]
    print(f"joint name {joint_name}")

    # 2. begin to parse components
    comp_begin_lst, comp_end_lst = find_begin_and_end_lst(
        cont, COMP_BEGIN_KEY, COMP_END_KEY)
    num_of_comp = len(comp_begin_lst)

    dof_lst = []
    for i in range(num_of_comp):
        begin = comp_begin_lst[i]
        end = comp_end_lst[i]
        axis, phase_lst, angle_lst = parse_component(cont[begin: end + 1])
        dof_lst.append({"axis": axis, "phase": phase_lst, "angle": angle_lst})

    return joint_name, dof_lst


def parse_constate(cont):
    '''
        Given file content, find all constate
        return all trajecotires
    '''
    num_of_lines = len(cont)
    assert cont[0].strip()[:len(STATE_BEGIN_KEY)] == STATE_BEGIN_KEY
    assert cont[-1].strip()[:len(STATE_END_KEY)] == STATE_END_KEY

    # find all trajectory begin/end indices
    traj_begin_lst, traj_end_lst = find_begin_and_end_lst(
        cont, TRAJ_BEGIN_KEY, TRAJ_END_KEY)

    constate = {}
    for i in range(len(traj_begin_lst)):

        begin = traj_begin_lst[i]
        end = traj_end_lst[i]
        print(
            f"-----------begin to parse trajectory {i}, being {begin} end {end} cont size {len(cont)}-----------")

        joint_name, sub_cont = parse_trajectory(cont[begin: end + 1])
        constate[joint_name] = sub_cont
    return constate


def load_sbc(sbc_path):
    with open(sbc_path, 'r') as f:
        cont = f.readlines()

        # find all constate begin lst and constate end lst
        constate_begin_lst, constate_end_lst = find_begin_and_end_lst(
            cont, STATE_BEGIN_KEY, STATE_END_KEY)

        # parse each con state
        constate_lst = []
        for i in range(len(constate_begin_lst)):
            print(cont[constate_begin_lst[i]])
            print(cont[constate_end_lst[i]])
            constate = parse_constate(
                cont[constate_begin_lst[i]: constate_end_lst[i] + 1])
            constate_lst.append(constate)

        return constate_lst
