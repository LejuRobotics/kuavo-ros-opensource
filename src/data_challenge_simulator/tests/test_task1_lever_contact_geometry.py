"""A lever grasp closes around the rod instead of pushing it with the palm."""
import ast
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from utils.scene1_v2_right_arm_ik import Scene1V2RightArmIK
from utils.hand_postures import single_gripper_targets


def test_palm_contacts_before_fingers_close_around_rod():
    tree = ast.parse((Path(__file__).resolve().parents[1] / 'examples/task1.py').read_text())
    ns = {'single_gripper_targets': single_gripper_targets, 'Rotation': Rotation}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == 'lever_hand_targets':
            exec(compile(ast.Module(body=[node], type_ignores=[]), '<hand>', 'exec'), ns)
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id in
                ('LEVER_GRIP_CENTER_IN_R7', 'contact_target', 'contact_rotation_world') for t in node.targets):
            exec(compile(ast.Module(body=[node], type_ignores=[]), '<target>', 'exec'), ns)
    ik = Scene1V2RightArmIK()
    ik.set_base_translation_world((.015, 0, 0))
    lower = ik.lower.copy(); lower[1] = -1.6; lower[4] = 0
    rotation = ns['contact_rotation_world']
    # A small palm-normal seating movement establishes palm contact first.
    target = np.array(ns['contact_target']) + rotation[:, 1] * .0015
    q = ik.solve(target, Rotation.from_matrix(rotation).as_quat(), ik.READY,
                 point_in_r7=ns['LEVER_GRIP_CENTER_IN_R7'], joint_bounds=(lower, ik.upper))
    for flexion in (0.6, 1.61):
        ik.task_obstacle_contacts(q, ns['lever_hand_targets']((flexion,) * 3))
        normals = {}
        for contact in ik.data.contact:
            names = [ik.model.geom(int(g)).name for g in (contact.geom1, contact.geom2)]
            if 'lever_handle_collision' not in names: continue
            other = next(n for n in names if n != 'lever_handle_collision')
            assert -0.004 < contact.dist < 0, (other, contact.dist)
            normals[other] = contact.frame[:3].copy()
        palm = normals.pop('task2_r_hand_base_collision')
        if flexion == 0.6:
            assert not normals
        else:
            opposed = {name.split('_')[2] for name, normal in normals.items()
                       if name.startswith('task2_r_') and np.dot(palm, normal) < -.5}
            assert len(opposed) >= 2, normals


def test_lever_collision_matches_visible_surface():
    ik = Scene1V2RightArmIK()
    visible = ik.model.geom('lever_handle')
    collision = ik.model.geom('lever_handle_collision')
    np.testing.assert_allclose(collision.size, visible.size, atol=1e-9)
    np.testing.assert_allclose(collision.pos, visible.pos, atol=1e-9)
    np.testing.assert_allclose(collision.quat, visible.quat, atol=1e-9)


def test_grasp_point_seats_on_visible_palm_instead_of_convex_hull_gap():
    import mujoco
    tree = ast.parse((Path(__file__).resolve().parents[1] / 'examples/task1.py').read_text())
    point = next(ast.literal_eval(node.value) for node in tree.body
                 if isinstance(node, ast.Assign) and any(
                     isinstance(t, ast.Name) and t.id == 'LEVER_GRIP_CENTER_IN_R7'
                     for t in node.targets))
    ik = Scene1V2RightArmIK()
    mujoco.mj_forward(ik.model, ik.data)
    rotation = ik.data.xmat[ik.r7_body_id].reshape(3, 3)
    wrist = ik.data.xpos[ik.r7_body_id]
    origin = wrist + rotation @ np.array([point[0], .2, point[2]])
    # Ray casting uses visible mesh triangles, not the collision convex hull.
    distance = mujoco.mj_rayMesh(ik.model, ik.data,
        ik.model.geom('task2_r_hand_base_collision').id, origin, -rotation[:, 1])
    assert distance > 0
    visible_surface_y = .2 - distance
    rod_surface_y = point[1] - ik.model.geom('lever_handle').size[0]
    assert abs(rod_surface_y - visible_surface_y) < .001
