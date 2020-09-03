from transform3d import Transform, SceneNode, SceneState
from collections import namedtuple

Scene = namedtuple('Scene', (
    'table',
    'b_base', 'b_tcp',
    'c_base', 'c_tcp',
    'cam_black', 'cam_white',
))


def load_scene():
    table, b_base, b_tcp, c_base, c_tcp, cam_black, cam_white = SceneNode.n(7)
    table.adopt(
        b_base.adopt(b_tcp),
        c_base.adopt(c_tcp.adopt(cam_black, cam_white))
    )
    state = SceneState()
    state[table] = Transform()
    state[b_base] = Transform.load('calib/table_t_base_b')
    state[c_base] = Transform.load('calib/table_t_base_c')
    state[cam_black] = Transform.load('calib/tcp_t_cam_black')
    state[cam_white] = Transform.load('calib/tcp_t_cam_white')

    return Scene(table, b_base, b_tcp, c_base, c_tcp, cam_black, cam_white), state
