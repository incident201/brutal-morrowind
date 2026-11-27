#!/usr/bin/env python3
"""Retarget the right arm of Dark Messiah's dagger throw to Morrowind's thrown weapons."""
import argparse
import json
from pathlib import Path

import numpy as np

from import_bow import blend, clip_poses, ease_in, source
from import_object_grab import ObjectGrabImporter
from import_viewmodel import Viewmodel


BASIC = 'first_animations_basic'
DAGGERS = 'first_animations_daggers'
WINDUP = 'anm_throw_Daggers_in'
CHARGED = 'anm_throw_Daggers_idle'
THROW = 'anm_throw_Daggers_out'
IDLE = 'anm_daggers_idle'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('vpk-root', 'work-dir', 'skeleton', 'grip-animation', 'camera-contract', 'arms'):
        parser.add_argument('--' + name, required=True, type=Path)
    args = parser.parse_args()

    vm = ObjectGrabImporter(args.vpk_root, args.work_dir, args.skeleton, args.grip_animation,
                            camera_contract=args.camera_contract, native_eye_height=True)
    folder = args.arms / 'Animations/brutal_fp_thrown'
    folder.mkdir(parents=True, exist_ok=True)
    mount = vm.local('Weapon Bone')
    shoulder = 'Bip01 R Clavicle'
    shoulder_parent = vm.rest[vm.parents[shoulder]][:3, :3]
    # Keep the point of the dart and the face of the star in the held shot.
    framing = shoulder_parent.T @ np.array([0., 9., -7.])

    def throw_poses(poses):
        # The camera and neck need keyed transforms for first-person pitch.
        # Leave the left arm and its equipment to their own animation layers.
        converted = []
        for pose in poses:
            result = {name: value for name, value in pose.items()
                      if not name.startswith('Bip01 L ') and name != 'Shield Bone'}
            rotation, position = result[shoulder]
            result[shoulder] = rotation, position + framing
            result['Weapon Bone'] = mount
            converted.append(result)
        return converted

    def clip(model, name):
        length = vm.duration(model, name)
        times, poses = clip_poses(vm, name, 0., length, model=model)
        return times, throw_poses(poses)

    idle_times, idle_poses = clip(BASIC, IDLE)
    ready = idle_poses[0]
    idle_length = float(idle_times[-1])
    vm.pending = [source(IDLE, idle_length, loop=True, model=BASIC)]
    Viewmodel.write(vm, folder, 'idle1t', [(0., idle_times, idle_poses)],
                    [(0., 'start'), (0., 'loop start'), (idle_length, 'loop stop'), (idle_length, 'stop')])

    segments, marks = [], []
    offset = 0.

    def add(times, poses, name, model, fade=0.):
        nonlocal offset
        length = float(times[-1])
        segments.append((offset, times, poses))
        vm.pending.append(source(name, length, runtime_fade_in=fade, model=model))
        offset += length

    def mark(name, at=None):
        marks.append((offset if at is None else at, name))

    mark('start')
    mark('equip start')
    equip = 'anm_daggers_draw'
    times, poses = clip(BASIC, equip)
    mark('equip attach', float(times[-1]) * .4)
    add(times, poses, equip, BASIC, .12)
    add(*blend(poses[-1], ready, .12), IDLE, BASIC)
    mark('equip stop')

    mark('shoot start')
    mark('brutal continuous release')
    windup_start = offset
    times, poses = clip(DAGGERS, WINDUP)
    add(times, ease_in(ready, times, poses, .14), WINDUP, DAGGERS, .08)
    mark('shoot min attack', windup_start + .08)
    charged = throw_poses([vm.pose_from_dm(vm.dm_world(DAGGERS, CHARGED, 0.))])[0]
    add(*blend(poses[-1], charged, .04), CHARGED, DAGGERS)
    mark('shoot max attack')
    mark('shoot min hit')

    start = offset
    times, poses = clip(DAGGERS, THROW)
    release = vm.sequence_event_time(DAGGERS, THROW, 2132, -1.)
    if not 0. < release < times[-1]:
        raise ValueError('Dark Messiah dagger throw is missing its release event')
    add(times, poses, THROW, DAGGERS)
    mark('shoot release', start + release)
    mark('shoot follow start', start + release)
    # The follow-through takes the hand below the frame before it returns armed.
    mark('shoot follow attach', start + float(times[-1]) * .56)
    add(*blend(poses[-1], ready, .14), IDLE, BASIC)
    mark('shoot follow stop')

    mark('unequip start')
    undraw = 'anm_daggers_undraw'
    times, poses = clip(BASIC, undraw)
    mark('unequip detach', offset + float(times[-1]) * .6)
    add(times, ease_in(ready, times, poses, .1), undraw, BASIC)
    mark('unequip stop')
    mark('stop')
    Viewmodel.write(vm, folder, 'throwweapon', segments, marks)
    report = {
        'converter': 'import_thrown.py',
        'groups': vm.groups[folder.name],
        'release_event': 2132,
        'source_release_seconds': release,
    }
    (args.arms / 'thrown-import.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
