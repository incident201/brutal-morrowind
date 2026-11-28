"""Adapt Morrowind's digitigrade gait for the visible first-person body."""
import argparse
from pathlib import Path

import numpy as np

from import_staff_native_grasp import swing
from retarget_kf import norm, quat_matrix
from slide_pose import Rig, nif, write_kf

PELVIS = 'Bip01 Pelvis'
CROUCH_DROP = np.array([0., 0., -22.])
SPRINT_RATE = 1.35
CROUCH_WALK_RATE = 1.45


def lower_crouch(rig, pose):
    """Lower the hips and keep each stock ankle position, foot angle and toe key."""
    world = rig.world(pose)
    parent = world[rig.parents[PELVIS]][:3, :3]
    rotation, translation = pose[PELVIS]
    pose[PELVIS] = rotation, translation + parent.T @ CROUCH_DROP
    for side in 'LR':
        thigh, calf, foot = [f'Bip01 {side} {joint}' for joint in ('Thigh', 'Calf', 'Foot')]
        hip, knee, ankle = (world[joint][:3, 3] for joint in (thigh, calf, foot))
        upper, lower = knee - hip, ankle - knee
        a, b = norm(upper), norm(lower)
        lowered = hip + CROUCH_DROP
        reach = ankle - lowered
        distance = norm(reach)
        if not abs(a - b) < distance < a + b:
            raise ValueError(f'Lowered beast crouch cannot reach the {side} foot')
        axis = reach / distance
        # Preserve the native bend plane as the hip and ankle get closer.
        bend = upper - axis * np.dot(upper, axis)
        bend /= norm(bend)
        along = (a*a - b*b + distance*distance) / (2 * distance)
        bent = lowered + axis * along + bend * np.sqrt(max(0., a*a - along*along))
        thigh_world = swing(upper, bent - lowered) @ world[thigh][:3, :3]
        calf_world = swing(lower, ankle - bent) @ world[calf][:3, :3]
        pose[thigh] = world[rig.parents[thigh]][:3, :3].T @ thigh_world, pose[thigh][1]
        pose[calf] = thigh_world.T @ calf_world, pose[calf][1]
        pose[foot] = calf_world.T @ world[foot][:3, :3], pose[foot][1]
    return pose


class BeastGait:
    def __init__(self, rig, animation):
        self.rig = rig
        stream = nif.NiStream()
        stream.load(animation)
        root = stream.root
        self.markers = {}
        for time, text in root.extra_data.keys:
            for line in text.lower().splitlines():
                if ': ' in line:
                    group, mark = line.strip().split(': ', 1)
                    self.markers.setdefault(group, {})[mark] = float(time)
        names = [data.string_data for data in root.extra_datas if isinstance(data, nif.NiStringExtraData)]
        self.tracks = {}
        for name, controller in zip(names, root.controllers, strict=True):
            if name not in rig.nodes or controller.data is None:
                continue
            rotations = controller.data.rotations.keys.copy()
            for index in range(1, len(rotations)):
                if np.dot(rotations[index-1, 1:], rotations[index, 1:]) < 0.:
                    rotations[index, 1:] *= -1.
            self.tracks[name] = rotations, controller.data.translations.keys

    def sample(self, time):
        pose = {}
        for name, (rotations, translations) in self.tracks.items():
            rotation, translation = self.rig.rest[name]
            if len(rotations):
                quat = np.array([np.interp(time, rotations[:, 0], column) for column in rotations[:, 1:].T])
                rotation = quat_matrix((quat / norm(quat))[[1, 2, 3, 0]])
            if len(translations):
                translation = np.array([np.interp(time, translations[:, 0], column) for column in translations[:, 1:4].T])
            pose[name] = rotation, translation
        # The actual player controller supplies world movement.
        rotation, translation = pose['Bip01']
        pose['Bip01'] = rotation, np.array([0., 0., translation[2]])
        return pose

    def write(self, output, source, target, rate, crouch=False):
        keys = self.markers[source]
        start = keys.get('loop start', keys['start'])
        end = keys.get('loop stop', keys['stop'])
        count = max(2, round((end - start) * 60.) + 1)
        times = np.linspace(0., (end - start) / rate, count)
        poses = [self.sample(t) for t in np.linspace(start, end, count)]
        if crouch:
            poses = [lower_crouch(self.rig, pose) for pose in poses]
        poses[-1] = poses[0]
        marks = [(0., 'start'), (0., 'loop start'), (times[-1], 'loop stop'), (times[-1], 'stop')]
        write_kf(output / (target + '.kf'), times, poses, marks)
        print(f'{target}: {times[-1]:.3f}s from {source}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for path in ('skeleton', 'animation', 'output'):
        parser.add_argument('--' + path, type=Path, required=True)
    args = parser.parse_args()
    gait = BeastGait(Rig(args.skeleton), args.animation)
    output = args.output / 'Animations/brutal_beast_body'
    gait.write(output, 'idlesneak', 'idlesneak', 1., crouch=True)
    for direction in ('forward', 'back', 'left', 'right'):
        group = 'sneak' + direction
        for suffix in ('', '1h'):
            target = group + suffix
            source = target if 'start' in gait.markers.get(target, {}) else group
            gait.write(output, source, target, CROUCH_WALK_RATE, crouch=True)
    for suffix in ('', '1h', '2c', '2w'):
        gait.write(output, 'runforward' + suffix, 'brutal_sprint' + suffix, SPRINT_RATE)


if __name__ == '__main__':
    main()
