from ur_control import Robot
import peg_in_hole_visual_servoing
import json
import utils
import rospy
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('config')
args = parser.parse_args()

rospy.init_node('servo config')

r_b = Robot.from_ip('192.168.1.129')
r_c = Robot.from_ip('192.168.1.131')

scene, state = utils.load_scene()

intrinsics = [json.load(open(f'calib/cam_{c}_intrinsics.json')) for c in ('black', 'white')]

config = peg_in_hole_visual_servoing.config_from_demonstration(
    peg_robot=r_b, aux_robots=[r_c], scene_state=state,
    peg_tcp_node=scene.b_tcp, aux_tcp_nodes=[scene.c_tcp],
    camera_nodes=[scene.cam_black, scene.cam_white],
    image_topics=[f'/servo/camera_{c}/color/image_raw' for c in ('black', 'white')],
    Ks=[o['camera_matrix'] for o in intrinsics],
    dist_coeffs=[o['dist_coeffs'] for o in intrinsics],
)

json.dump(config, open(args.config, 'w'))
