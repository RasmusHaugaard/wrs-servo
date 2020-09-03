import json
import time
import argparse

import numpy as np
import rospy

from transform3d import Transform
from ur_control import Robot
import peg_in_hole_visual_servoing

import utils

parser = argparse.ArgumentParser()
parser.add_argument('config')
parser.add_argument('--seed', type=int, default=None)
parser.add_argument('--n', type=int, default=1)
parser.add_argument('--start-delay', type=float, default=0.)
args = parser.parse_args()

time.sleep(args.start_delay)
np.random.seed(args.seed)

if 'pinol' in args.config:
    max_err = 0.0035
elif 'm4' in args.config:
    max_err = 0.005
elif 'm3' in args.config:
    max_err = 0.0035
else:
    raise ValueError()

config = json.load(open(args.config))

rospy.init_node('servo')

r_b = Robot.from_ip('192.168.1.129')
r_c = Robot.from_ip('192.168.1.131')

scene, state = utils.load_scene()

# cam safe q
r_c.ctrl.moveJ([2.2288451194763184, -1.490114526157715, 1.5134428183185022,
                -0.8572123807719727, -1.3860304991351526, -0.5580604712115687])
for q, r in zip(config['robots_q_init'], [r_b, r_c]):
    r.ctrl.moveJ(q)

r_b.zero_ft_sensor()
base_t_tcp_b_init = r_b.base_t_tcp()

for _ in range(args.n):
    if rospy.is_shutdown():
        break

    yz = np.random.randn(2)
    yz = yz / np.linalg.norm(yz) * max_err
    tcp_err = Transform(p=(0, *yz))
    base_t_tcp_b = base_t_tcp_b_init @ tcp_err
    r_b.ctrl.moveL(base_t_tcp_b)

    peg_in_hole_visual_servoing.servo(
        peg_robot=r_b, aux_robots=[r_c], scene_state=state,
        peg_tcp_node=scene.b_tcp, aux_tcp_nodes=[scene.c_tcp],
        camera_nodes=[scene.cam_black, scene.cam_white],
        servo_config=config,
        insertion_direction_tcp=np.array((1, 0, 0)),
        alpha_target=0.95, timeout=10,
    )

    r_b.speed_until_ft((0.01, 0, 0))
    time.sleep(0.5)
    r_b.ctrl.moveL(r_b.base_t_tcp() @ Transform(p=(-0.03, 0, 0)))

r_b.ctrl.moveL(r_b.base_t_tcp() @ Transform(p=(-0.07, 0, 0)))
