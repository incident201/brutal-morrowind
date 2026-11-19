#!/usr/bin/env python3
"""Support the staff's charged jab from underneath while the right hand rolls the staff.

Runs on the imported staff set before the spear set is built from it.
"""
import argparse
from pathlib import Path

import numpy as np

from arm_steering import LocalPose, blend, ease, steer_arm
from import_staff_native_grasp import swing
from import_two_hand_viewmodel import (
    Grasp, ITEM, Pose, Rig, inverse, keys, load, matrix, matrix_quat, native_starts, nif, tracks, turn_about, unit,
)


def staff_back_hand(stream, channels, rig, vanilla, release):
    """Support the jab from underneath while the right hand rolls the staff.

    The shaft spins around its length during the lunge. Carrying the same roll
    into the supporting wrist would bring the elbow across the camera. Transport
    its contact frame along the shaft direction, and release on the original beat.
    """
    grasp = Grasp(vanilla, 'L', False)
    grasp.grip = matrix(turn_about((0, 1, 0), -165.), np.zeros(3)) @ grasp.grip
    grasp.grip[1, 3] = -36.
    grasp.pole = unit(np.array((-.8, -.58, .15)))
    grasp.wrist_bias = 6.
    reference = Pose(rig, channels, release + 39. / 33.).world(ITEM)
    recovery = Pose(rig, channels, release + 43. / 33.).world(ITEM)
    names = [e.string_data for e in stream.root.extra_datas if isinstance(e, nif.NiStringExtraData)]
    controls = dict(zip(names, stream.root.controllers))
    for i, t in enumerate(channels[grasp.names[-1]][0][:, 0]):
        frame = (float(t) - release) * 33.
        if 43. < frame < 56.:
            # The returning right wrist rolls the mesh 64 degrees in one
            # source frame, inside the still closed supporting palm. Keep the
            # shaft's direction and mounting point, then let its axial turn
            # catch up while the source regrip has the fingers open.
            pose = Pose(rig, channels, t)
            item = pose.world(ITEM)
            follow = swing(recovery[:3, 1], item[:3, 1]) @ recovery[:3, :3]
            difference = item[:3, :3].T @ follow
            roll = np.degrees(np.arctan2(difference[0, 2], difference[0, 0]))
            weight = 1. - ease((frame-45.)/11.)
            mounted = pose.local(ITEM)[:3, :3] @ turn_about((0, 1, 0), roll*weight)
            keys = controls[ITEM].data.rotations.keys
            q = matrix_quat(mounted)
            keys[i, 1:] = q if keys[i-1, 1:] @ q >= 0. else -q
        if not 20. < frame < 50.:
            continue
        w = ease(min((frame - 20.) / 8., (50. - frame) / 7.))
        pose = Pose(rig, channels, t)
        item = pose.world(ITEM).copy()
        item[:3, :3] = swing(reference[:3, 1], item[:3, 1]) @ reference[:3, :3]
        target = item @ grasp.grip
        shoulder = pose.world(grasp.names[1])[:3, 3]
        delta = target[:3, 3] - shoulder
        advance = max(0., np.linalg.norm(delta) - sum(grasp.segments) + 1.)
        offset = matrix(np.eye(3), shoulder - grasp.shoulder + unit(delta) * advance)
        chain, _ = grasp.fit(inverse(offset) @ item)
        local = {}
        parent = pose.world(rig.parents[grasp.names[0]])
        for joint, transform in zip(grasp.names, chain):
            world = offset @ transform
            local[joint] = inverse(parent) @ world
            parent = world
        goal = LocalPose(rig, {**{n: pose.local(n) for n in channels}, **local})
        hand = blend(pose.world(grasp.names[-1]), target, w)
        arm = steer_arm(pose, goal, grasp, hand, w)
        arm.update({n: blend(pose.local(n), value, w) for n, value in grasp.fingers.items()})
        for joint, value in arm.items():
            data = controls[joint].data
            q = matrix_quat(value[:3, :3])
            if data.rotations.keys[i-1, 1:] @ q < 0.:
                q = -q
            data.rotations.keys[i, 1:] = q
            data.translations.keys[i, 1:] = value[:3, 3]


def build(arms, native_skeleton, native_animation):
    source = arms / 'Animations/brutal_fp_staff/weapononehand.kf'
    stream = load(source)
    marks = {line.removeprefix('weapononehand: '): float(t)
             for t, label in keys(stream).keys for line in label.splitlines()}
    native_rig, native_tracks = Rig(native_skeleton), tracks(native_animation)
    vanilla = Pose(native_rig, native_tracks, native_starts(native_animation)['idle2w: start'])
    rig = Rig(arms / 'meshes/brutal_fp/xbase_anim.1st.nif')
    staff_back_hand(stream, tracks(source), rig, vanilla, marks['thrust min hit'])
    stream.save(source)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arms', type=Path, required=True)
    parser.add_argument('--native-skeleton', type=Path, required=True)
    parser.add_argument('--native-animation', type=Path, required=True)
    args = parser.parse_args()
    build(args.arms, args.native_skeleton, args.native_animation)
