import sys
from sbc_parser import load_sbc
import json
import numpy as np
from enum import IntEnum
from catmull import interploate_catmull_rom


class Joint(object):
    class JointType(IntEnum):
        NONE = 0
        REVOLUTE = 1
        SPHERICAL = 2
        FIXED = 3

    JOINT_TYPE_STR = ["none", "revolute", "spherical", "fixed"]
    JOINT_DOF = [7, 1, 4, 0]

    @staticmethod
    def decide_joint_type(joint_type_str):

        for joint_type, j_str in enumerate(Joint.JOINT_TYPE_STR):
            if j_str == joint_type_str:
                return Joint.JointType(joint_type)

        assert False, f"joint type str {joint_type_str} invalid"

    @staticmethod
    def decide_joint_dof(joint_type: JointType) -> int:
        assert isinstance(joint_type, Joint.JointType)
        return Joint.JOINT_DOF[int(joint_type)]

    def __init__(self, joint_id_, joint_name, joint_type_str):
        assert isinstance(joint_id_, int)
        self.joint_id = joint_id_
        self.joint_name = joint_name
        self.joint_type = Joint.decide_joint_type(joint_type_str)
        self.joint_dof = Joint.decide_joint_dof(self.joint_type)

    def get_joint_type(self):
        return self.joint_type

    def get_joint_name(self):
        return self.joint_name

    def get_joint_dof(self):
        return self.joint_dof

    def get_joint_id(self):
        return self.joint_id

    def set_offset(self, offset):
        self.offset = offset

    def get_offset(self):
        return self.offset


def get_skeleton_info(path):
    joint_lst = []
    with open(path, "r") as f:
        json_cont = json.load(f)
        joints = json_cont["Skeleton"]["Joints"]
        for single_joint in joints:
            # print(single_joint)
            id = single_joint["ID"]
            name = single_joint["Name"]
            joint_type = single_joint["Type"]

            print(f"{id} {name} {joint_type}")
            joint_lst.append(Joint(int(id), name, joint_type))
    joint_lst.sort(key=lambda x: x.get_joint_id())
    offset = 0
    for single_joint in joint_lst:
        single_joint.set_offset(offset)
        print(f"joint {single_joint.get_joint_name()} offset {offset}")
        offset += single_joint.get_joint_dof()

    joint_lst.sort(key=lambda x: x.get_joint_id())
    return joint_lst


def convert_sbcname_to_mimicname(sbcname):

    name_dict = {
        "Hip": "Leg",
        "Knee": "Foot",
        "Ankle": "FootTongue"
    }

    return name_dict[sbcname]


def convert_mimicname_to_sbcname(sbcname):

    name_dict = {
        "Leg": "Hip",
        "Foot": "Knee",
        "FootTongue": "Ankle"
    }

    return name_dict[sbcname]


sbc_path = "backWalk_havetorso_notoe.sbc"
model_path = "/home/xudong/Projects/DeepMimic/data/1127/characters/lower_halfbody_torso.json"
constate_lst = load_sbc(sbc_path)

assert len(constate_lst) == 1
state = constate_lst[0]

joint_lst = get_skeleton_info(model_path)
print(f"joint lst {joint_lst}")
print(f"state {state.keys()}")


def get_mimic_joint_suffix(name):
    LEFT_KEY = "Left"
    RIGHT_KEY = "Right"
    if name[:len(LEFT_KEY)] == LEFT_KEY:
        return name[len(LEFT_KEY):]
    elif name[:len(RIGHT_KEY)] == RIGHT_KEY:
        return name[len(RIGHT_KEY):]
    assert False


# return the side info in lower case
def get_mimic_joint_side(name):
    LEFT_KEY = "Left"
    RIGHT_KEY = "Right"
    if name[:len(LEFT_KEY)] == LEFT_KEY:
        return "left"
    elif name[:len(RIGHT_KEY)] == RIGHT_KEY:
        return "right"
    assert False


def get_sbcjoint_angle(cur_time, sbc_joint_name, mimic_joint_type, constate):
    assert type(cur_time) is np.float64, f"{type(cur_time)}"
    assert type(constate) is dict
    assert type(mimic_joint_type) is Joint.JointType

    angle = None
    if sbc_joint_name in constate:
        sbc_joint = constate[sbc_joint_name][0]
        assert sbc_joint["axis"][0] == 1

        angle = interploate_catmull_rom(
            cur_time, sbc_joint['phase'], sbc_joint['angle'])
    else:
        angle = 0

    if mimic_joint_type is Joint.JointType.REVOLUTE:
        return [angle]
    elif mimic_joint_type is Joint.JointType.SPHERICAL:
        return [np.cos(angle/2), np.sin(angle/2), 0, 0]

    return None


def generate_motion_by_stance(stance_side):
    assert stance_side == "right" or stance_side == "left"
    motion_lst = []

    # 1. right stance, left swing, 5 frames
    motion_samples = 5
    frame_time = 0.1    # 0.1s
    motion_timestamp_lst = np.linspace(0, 1, motion_samples)
    for frame_id in range(motion_samples):
        cur_time = motion_timestamp_lst[frame_id]
        motion_frame = [1]
        # 1. begin to form the motion vector in this time
        for mimic_joint in joint_lst:
            mimic_joint_name = mimic_joint.get_joint_name()
            if mimic_joint_name == "root":
                motion_frame = motion_frame + [0, 0.75, 0, 1, 0, 0, 0]
            elif mimic_joint_name == "torso":
                motion_frame = motion_frame + [1, 0, 0, 0]
            else:
                # Leg, Foot, FootTongue
                # Hip, Knee, Ankle
                mimic_suffix = get_mimic_joint_suffix(mimic_joint_name)
                mimic_side = get_mimic_joint_side(mimic_joint_name)
                # print(
                #     f"mimic joint {mimic_joint_name}, side {mimic_side}, suffix {mimic_suffix}")
                sbc_suffix = convert_mimicname_to_sbcname(mimic_suffix)
                sbc_side = "STANCE" if mimic_side == stance_side else "SWING"
                sbc_joint_fullname = f"{sbc_side}_{sbc_suffix}"

                # find the approprite sbc joint, calcualte the joint angle
                joint_angle = get_sbcjoint_angle(
                    cur_time, sbc_joint_fullname, mimic_joint.get_joint_type(), state)
                print(f"[log] joint {sbc_joint_fullname} angle {joint_angle}")
                motion_frame = motion_frame + joint_angle
        motion_lst.append(motion_frame)
    return motion_lst

motion_lst = []
motion_lst += generate_motion_by_stance("left")
motion_lst += generate_motion_by_stance("right")
motion_json = {
    "Loop": "wrap",
    "Frames": motion_lst
}
with open("/home/xudong/Projects/DeepMimic/data/1127/motions/test.json", 'w') as f:
    json.dump(motion_json, f)
